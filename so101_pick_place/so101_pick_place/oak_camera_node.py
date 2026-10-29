"""OAK-D Pro を realsense2_camera と同じトピック構成で配信するノード（depthai 2.x、ROS パッケージ不要）。

配信（名前は D455 のときと同じにして下流を変更しない）:
  /camera/camera/color/image_raw                   rgb8   (既定 1280x720)
  /camera/camera/aligned_depth_to_color/image_raw   16UC1  [mm]  RGB に位置合わせ済み・同じ stamp
  /camera/camera/color/camera_info                 RGB の内部パラメータ（デバイスのキャリブレーションから）
  TF static: camera_link -> camera_color_optical_frame（光学座標系への回転のみ。camera_link = RGB センサ位置）

近距離向け設定: mono 400P + extended disparity + subpixel + LR check、IR ドットプロジェクタ ON。
"""

import depthai as dai
import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import TransformStamped
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import StaticTransformBroadcaster


class OakCameraNode(Node):
    def __init__(self):
        super().__init__('oak_camera')
        self.declare_parameter('width', 1280)
        self.declare_parameter('height', 720)
        self.declare_parameter('fps', 30)
        self.declare_parameter('mono_resolution', '400p')       # 400p: 近距離に強い / 800p: 精細
        self.declare_parameter('extended_disparity', True)      # 最小距離を半分に
        self.declare_parameter('subpixel', True)
        self.declare_parameter('lr_check', True)
        self.declare_parameter('confidence_threshold', 200)
        self.declare_parameter('ir_laser_dot', 0.8)              # 0..1 (0 で OFF)
        self.declare_parameter('ir_flood', 0.0)
        self.declare_parameter('camera_frame', 'camera_link')
        self.declare_parameter('optical_frame', 'camera_color_optical_frame')
        self.declare_parameter('camera_name', 'camera')
        self.declare_parameter('camera_namespace', 'camera')

        p = self.get_parameter
        self.w, self.h, self.fps = int(p('width').value), int(p('height').value), int(p('fps').value)
        self.optical_frame = p('optical_frame').value
        ns = f"/{p('camera_namespace').value}/{p('camera_name').value}"

        qos = QoSProfile(reliability=QoSReliabilityPolicy.RELIABLE, history=QoSHistoryPolicy.KEEP_LAST, depth=5)
        self.color_pub = self.create_publisher(Image, f'{ns}/color/image_raw', qos)
        self.depth_pub = self.create_publisher(Image, f'{ns}/aligned_depth_to_color/image_raw', qos)
        self.info_pub = self.create_publisher(CameraInfo, f'{ns}/color/camera_info', qos)
        self.depth_info_pub = self.create_publisher(CameraInfo, f'{ns}/aligned_depth_to_color/camera_info', qos)
        self.bridge = CvBridge()

        self.pipeline = self._build_pipeline()
        self.device = dai.Device(self.pipeline)
        self._setup_device()
        self.q_rgb = self.device.getOutputQueue('rgb', maxSize=4, blocking=False)
        self.q_depth = self.device.getOutputQueue('depth', maxSize=4, blocking=False)
        self.camera_info = self._camera_info()
        self._publish_static_tf(p('camera_frame').value)

        # depthai の device timestamp と ROS 時刻の差（起動時に一度合わせる）
        self._t_offset = None
        self._n = 0
        self._pending = {}   # seq -> (rgb_frame or None, depth_frame or None)
        self.timer = self.create_timer(0.002, self._poll)
        self.report = self.create_timer(5.0, self._report)
        self.get_logger().info(f'OAK-D ready: {self.device.getDeviceName()} usb={self.device.getUsbSpeed().name} '
                               f'{self.w}x{self.h}@{self.fps} mono={p("mono_resolution").value} '
                               f'ext_disp={p("extended_disparity").value} -> {ns}/...')

    # ------------------------------------------------------------------
    def _build_pipeline(self):
        p = self.get_parameter
        pipe = dai.Pipeline()
        cam = pipe.create(dai.node.ColorCamera)
        cam.setBoardSocket(dai.CameraBoardSocket.CAM_A)
        cam.setResolution(dai.ColorCameraProperties.SensorResolution.THE_1080_P)
        cam.setFps(self.fps)
        cam.setInterleaved(False)
        cam.setColorOrder(dai.ColorCameraProperties.ColorOrder.BGR)   # getCvFrame() は BGR で返る
        if (self.w, self.h) == (1280, 720):
            cam.setIspScale(2, 3)                     # 1920x1080 -> 1280x720
        elif (self.w, self.h) == (640, 360):
            cam.setIspScale(1, 3)
        cam.setPreviewSize(self.w, self.h)
        cam.setPreviewKeepAspectRatio(True)

        mono_res = {'400p': dai.MonoCameraProperties.SensorResolution.THE_400_P,
                    '480p': dai.MonoCameraProperties.SensorResolution.THE_480_P,
                    '720p': dai.MonoCameraProperties.SensorResolution.THE_720_P,
                    '800p': dai.MonoCameraProperties.SensorResolution.THE_800_P}[str(p('mono_resolution').value)]
        left = pipe.create(dai.node.MonoCamera); left.setBoardSocket(dai.CameraBoardSocket.CAM_B)
        right = pipe.create(dai.node.MonoCamera); right.setBoardSocket(dai.CameraBoardSocket.CAM_C)
        for m in (left, right):
            m.setResolution(mono_res); m.setFps(self.fps)

        stereo = pipe.create(dai.node.StereoDepth)
        stereo.setDefaultProfilePreset(dai.node.StereoDepth.PresetMode.HIGH_DENSITY)
        stereo.setLeftRightCheck(bool(p('lr_check').value))
        stereo.setExtendedDisparity(bool(p('extended_disparity').value))
        stereo.setSubpixel(bool(p('subpixel').value))
        stereo.initialConfig.setConfidenceThreshold(int(p('confidence_threshold').value))
        if bool(p('extended_disparity').value) and bool(p('subpixel').value):
            # ext + subpixel では disparity 範囲が median filter の上限(1024)を超えるため明示的に OFF（警告抑止）
            stereo.initialConfig.setMedianFilter(dai.MedianFilter.MEDIAN_OFF)
        stereo.setDepthAlign(dai.CameraBoardSocket.CAM_A)        # RGB に位置合わせ
        stereo.setOutputSize(self.w, self.h)
        left.out.link(stereo.left); right.out.link(stereo.right)

        xo_rgb = pipe.create(dai.node.XLinkOut); xo_rgb.setStreamName('rgb'); cam.preview.link(xo_rgb.input)
        xo_d = pipe.create(dai.node.XLinkOut); xo_d.setStreamName('depth'); stereo.depth.link(xo_d.input)
        return pipe

    def _setup_device(self):
        p = self.get_parameter
        try:
            self.device.setIrLaserDotProjectorIntensity(float(p('ir_laser_dot').value))
            self.device.setIrFloodLightIntensity(float(p('ir_flood').value))
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(f'IR setting failed: {exc}')

    def _camera_info(self):
        cal = self.device.readCalibration()
        M = np.array(cal.getCameraIntrinsics(dai.CameraBoardSocket.CAM_A, self.w, self.h), dtype=float)
        try:
            D = list(cal.getDistortionCoefficients(dai.CameraBoardSocket.CAM_A))
        except Exception:  # noqa: BLE001
            D = []
        info = CameraInfo()
        info.header.frame_id = self.optical_frame
        info.width, info.height = self.w, self.h
        info.distortion_model = 'rational_polynomial' if len(D) >= 8 else 'plumb_bob'
        info.d = [float(v) for v in (D[:8] if len(D) >= 8 else (D + [0.0] * 5)[:5])]
        info.k = M.flatten().tolist()
        info.r = np.eye(3).flatten().tolist()
        info.p = np.c_[M, np.zeros(3)].flatten().tolist()
        self.get_logger().info(f'camera_info: fx={M[0,0]:.1f} fy={M[1,1]:.1f} cx={M[0,2]:.1f} cy={M[1,2]:.1f}')
        return info

    def _publish_static_tf(self, camera_frame):
        self.static_tf = StaticTransformBroadcaster(self)
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = camera_frame
        t.child_frame_id = self.optical_frame
        # 光学フレーム: x 右, y 下, z 前  <- camera_link: x 前, y 左, z 上  (rpy = -90°, 0, -90°)
        t.transform.rotation.x, t.transform.rotation.y = -0.5, 0.5
        t.transform.rotation.z, t.transform.rotation.w = -0.5, 0.5
        self.static_tf.sendTransform(t)

    def _stamp(self, frame):
        # device monotonic timestamp -> ROS time（オフセットは最初のフレームで決める）
        ts = frame.getTimestamp().total_seconds()
        if self._t_offset is None:
            self._t_offset = self.get_clock().now().nanoseconds * 1e-9 - ts
        return ts + self._t_offset

    def _poll(self):
        rgb = self.q_rgb.tryGet()
        dep = self.q_depth.tryGet()
        for f, key in ((rgb, 'rgb'), (dep, 'depth')):
            if f is None:
                continue
            seq = f.getSequenceNum()
            entry = self._pending.setdefault(seq, {})
            entry[key] = f
            if 'rgb' in entry and 'depth' in entry:
                self._publish(seq, entry['rgb'], entry['depth'])
                for k in [k for k in self._pending if k <= seq]:
                    self._pending.pop(k, None)
        if len(self._pending) > 30:   # 片方しか来ないときの掃除
            for k in sorted(self._pending)[:-10]:
                self._pending.pop(k, None)

    def _publish(self, seq, rgb, dep):
        t = self._stamp(rgb)
        stamp = rclpy.time.Time(seconds=t).to_msg()
        img = rgb.getCvFrame()                    # BGR
        msg = self.bridge.cv2_to_imgmsg(img, encoding='bgr8')
        msg.header.stamp = stamp; msg.header.frame_id = self.optical_frame
        d = dep.getFrame()                        # uint16 mm
        if d.shape[:2] != img.shape[:2]:
            import cv2
            d = cv2.resize(d, (img.shape[1], img.shape[0]), interpolation=cv2.INTER_NEAREST)
        dmsg = self.bridge.cv2_to_imgmsg(d, encoding='16UC1')
        dmsg.header = msg.header
        self.camera_info.header.stamp = stamp
        self.color_pub.publish(msg); self.depth_pub.publish(dmsg)
        self.info_pub.publish(self.camera_info); self.depth_info_pub.publish(self.camera_info)
        self._n += 1

    def _report(self):
        self.get_logger().info(f'published {self._n/5.0:.1f} fps (rgb+aligned depth pairs)')
        self._n = 0

    def destroy_node(self):
        try:
            self.device.close()
        except Exception:  # noqa: BLE001
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = OakCameraNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except RuntimeError:
        if rclpy.ok():
            raise
    finally:
        node.destroy_node()
        if rclpy.ok():
            try:
                rclpy.shutdown()
            except Exception:  # noqa: BLE001
                pass


if __name__ == '__main__':
    main()
