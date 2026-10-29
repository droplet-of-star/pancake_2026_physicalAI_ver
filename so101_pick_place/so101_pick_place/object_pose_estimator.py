"""画像座標 (u, v) + 位置合わせ深度 -> カメラ座標 XYZ -> base_link 座標（作業台平面方式）。

入力:
  /so101_pick_place/object_pixel            PointStamped (x=u, y=v, z=面積)  object_detector から
  /so101_pick_place/object_bbox             RegionOfInterest  円盤の外接正方形（平面フィットの除外範囲）
  /so101_pick_place/handle_pixel            PointStamped  取っ手の把持画素
  /camera/camera/aligned_depth_to_color/image_raw   16UC1 [mm]（RGB と同じ光学フレーム・同じ stamp）
  /camera/camera/color/camera_info          fx, fy, cx, cy

出力:
  /so101_pick_place/object_point_camera         PointStamped  カメラ光学フレーム [m]
  /so101_pick_place/object_point_base           PointStamped  base_frame [m]（TF が引けるときだけ）
  /so101_pick_place/object_point_base_filtered  PointStamped  直近 filter_window フレームの中央値（controller はこちらを使う）
  /so101_pick_place/handle_point_base           PointStamped  取っ手の把持点 [m]（base_frame）
  /so101_pick_place/object_marker               visualization_msgs/Marker  RViz 確認用

深度の取り方: 物体表面の深度は OAK-D Pro では信用できない（無地の赤い面で IR ドットが誤マッチ）ため、
物体の外接矩形の周囲（作業台）の深度に平面を RANSAC フィットし、画素の視線と平面の交点を使う。
トレイ中心は縁の上端（plate_height_m）の面で切ってから台面に落とし、取っ手は板の高さ（handle_height_m）の面で切る。
"""
import math
import time
from collections import deque

import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import PointStamped
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image, RegionOfInterest
from std_msgs.msg import ColorRGBA
from tf2_geometry_msgs import do_transform_point
from tf2_ros import Buffer, TransformException, TransformListener
from visualization_msgs.msg import Marker


class ObjectPoseEstimator(Node):
    def __init__(self):
        super().__init__('object_pose_estimator')
        self.declare_parameter('depth_topic', '/camera/camera/aligned_depth_to_color/image_raw')
        self.declare_parameter('camera_info_topic', '/camera/camera/color/camera_info')
        self.declare_parameter('camera_frame', 'camera_color_optical_frame')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('depth_scale', 0.001)      # 16UC1 の 1 カウントあたりの m
        self.declare_parameter('min_depth_m', 0.15)
        self.declare_parameter('max_depth_m', 1.50)
        self.declare_parameter('max_stamp_diff_s', 0.05)  # pixel と depth の許容時刻差
        self.declare_parameter('depth_buffer_frames', 45)   # 深度フレームの保持数（30 fps で 1.5 s。検出ノードの処理遅れを吸収）
        self.declare_parameter('plane_annulus_scale', 1.6)   # 外接矩形の何倍の範囲から台の深度を取るか
        self.declare_parameter('plane_exclude_scale', 1.25)  # 物体除外領域（外接矩形の倍率）
        self.declare_parameter('plane_stride', 4)            # 画素の間引き
        self.declare_parameter('plane_ransac_iters', 80)
        self.declare_parameter('plane_inlier_m', 0.008)
        self.declare_parameter('plane_min_points', 1500)
        self.declare_parameter('object_height_m', 0.0)       # 台の平面から法線方向に持ち上げる量
        self.declare_parameter('base_z_min_m', -0.10)        # base 系での物体高さの妥当範囲（床などの誤検出を捨てる）
        self.declare_parameter('base_z_max_m', 0.30)
        self.declare_parameter('handle_height_m', 0.032)     # 取っ手の板の上面の高さ（台面から）。把持画素はこの高さの面で切る
        self.declare_parameter('plane_prefer_lowest', True)   # 有力候補のうち最も低い平面を台とみなす（リング内の高い物の上面を除く）
        self.declare_parameter('plate_height_m', 0.018)      # トレイ中心として検出する円は縁の上端（台面から 18 mm）。その高さの面で視線を切る（台面で切ると約 1.4 cm 奥にずれる）
        self.declare_parameter('plane_update_period_s', 0.3) # 平面の再フィット間隔（間は前回の平面を再利用）
        self.declare_parameter('plane_min_inlier_ratio', 0.25)
        self.declare_parameter('plane_max_rms_m', 0.012)
        self.declare_parameter('plane_max_angle_deg', 80.0)  # 平面法線と光軸のなす角の上限（台を真横から見ている状態を除外）
        self.declare_parameter('plane_ring_depth_gate_m', 0.10) # リング内の深度中央値±この範囲の点だけで平面推定（床など遠い面を除外）
        self.declare_parameter('filter_window', 15)          # base 座標の時間中央値フィルタ（フレーム数）。0 で無効

        p = self.get_parameter
        self.camera_frame = p('camera_frame').value
        self.base_frame = p('base_frame').value
        self.depth_scale = float(p('depth_scale').value)
        self.min_depth = float(p('min_depth_m').value)
        self.max_depth = float(p('max_depth_m').value)
        self.max_stamp_diff = float(p('max_stamp_diff_s').value)
        self.ann_scale = float(p('plane_annulus_scale').value)
        self.excl_scale = float(p('plane_exclude_scale').value)
        self.stride = max(1, int(p('plane_stride').value))
        self.ransac_iters = int(p('plane_ransac_iters').value)
        self.inlier_m = float(p('plane_inlier_m').value)
        self.plane_min_pts = int(p('plane_min_points').value)
        self.object_height = float(p('object_height_m').value)
        self.handle_height = float(p('handle_height_m').value); self.plate_height = float(p('plate_height_m').value)
        self.plane_prefer_lowest = bool(p('plane_prefer_lowest').value)
        self.base_z_min = float(p('base_z_min_m').value); self.base_z_max = float(p('base_z_max_m').value)
        self.bbox = None
        self._rng = np.random.default_rng(0)
        self._last_plane = None   # (n, d, rms, inlier_ratio)
        self._last_plane_t = 0.0
        self.plane_period = float(p('plane_update_period_s').value)
        self.plane_min_inl = float(p('plane_min_inlier_ratio').value)
        self.plane_max_rms = float(p('plane_max_rms_m').value)
        self.plane_max_angle = math.radians(float(p('plane_max_angle_deg').value))
        self.ring_gate = float(p('plane_ring_depth_gate_m').value)
        self.filter_window = int(p('filter_window').value)
        self._base_hist = deque(maxlen=max(1, self.filter_window))

        self.bridge = CvBridge()
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.fx = self.fy = self.cx = self.cy = None
        self.depths = deque(maxlen=int(self.get_parameter('depth_buffer_frames').value))   # (stamp_sec, frame_id, np.ndarray float32 [m]) 直近フレーム。検出の遅れ（rim_edge で最大 0.2 s）より長く保持する

        self.create_subscription(CameraInfo, p('camera_info_topic').value, self.on_camera_info, 10)
        self.create_subscription(Image, p('depth_topic').value, self.on_depth, 10)
        self.create_subscription(PointStamped, '/so101_pick_place/object_pixel', self.on_pixel, 10)
        self.create_subscription(RegionOfInterest, '/so101_pick_place/object_bbox', self.on_bbox, 10)
        self.create_subscription(PointStamped, '/so101_pick_place/handle_pixel', self.on_handle_pixel, 10)
        self.handle_base_pub = self.create_publisher(PointStamped, '/so101_pick_place/handle_point_base', 10)   # 取っ手の把持点 (base)。z 成分は把持点高さ
        self._last_handle_px = None
        self.cam_pub = self.create_publisher(PointStamped, '/so101_pick_place/object_point_camera', 10)
        self.base_pub = self.create_publisher(PointStamped, '/so101_pick_place/object_point_base', 10)
        self.base_filt_pub = self.create_publisher(PointStamped, '/so101_pick_place/object_point_base_filtered', 10)
        self.marker_pub = self.create_publisher(Marker, '/so101_pick_place/object_marker', 10)

        self._n_pixel = self._n_cam = self._n_base = 0
        self.create_timer(5.0, self._report)
        self.get_logger().info(
            f'object_pose_estimator: table-plane mode, depth {self.min_depth}-{self.max_depth} m '
            f'-> {self.camera_frame} / {self.base_frame}')

    # ------------------------------------------------------------------
    @staticmethod
    def _stamp_to_sec(stamp) -> float:
        return stamp.sec + stamp.nanosec * 1e-9

    def on_camera_info(self, msg: CameraInfo):
        if self.fx is None:
            self.get_logger().info(
                f'camera_info: fx={msg.k[0]:.1f} fy={msg.k[4]:.1f} cx={msg.k[2]:.1f} cy={msg.k[5]:.1f} '
                f'{msg.width}x{msg.height} frame={msg.header.frame_id}')
        self.fx, self.fy, self.cx, self.cy = msg.k[0], msg.k[4], msg.k[2], msg.k[5]

    def on_depth(self, msg: Image):
        try:
            if msg.encoding == '16UC1':
                raw = self.bridge.imgmsg_to_cv2(msg, desired_encoding='16UC1')
                depth = raw.astype(np.float32) * self.depth_scale
            elif msg.encoding == '32FC1':
                depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding='32FC1')
            else:
                self.get_logger().warn(f'unsupported depth encoding {msg.encoding}', throttle_duration_sec=5.0)
                return
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(f'cv_bridge(depth) failed: {exc}', throttle_duration_sec=5.0)
            return
        self.depths.append((self._stamp_to_sec(msg.header.stamp), msg.header.frame_id, depth))

    def depth_for(self, stamp_sec: float):
        """stamp に最も近い深度フレームを返す。許容差を超えたら None。"""
        if not self.depths:
            return None, None, None
        best = min(self.depths, key=lambda d: abs(d[0] - stamp_sec))
        dt = abs(best[0] - stamp_sec)
        if dt > self.max_stamp_diff:
            return None, None, dt
        return best[1], best[2], dt

    def on_bbox(self, msg: RegionOfInterest):
        self.bbox = (int(msg.x_offset), int(msg.y_offset), int(msg.width), int(msg.height))

    def fit_table_plane(self, depth: np.ndarray):
        """外接矩形の周囲（物体を除く）の深度点に平面をフィット。戻り値 (n, d, rms, inlier_ratio) または None。"""
        if self.bbox is None:
            self.get_logger().warn('plane: no bbox yet', throttle_duration_sec=2.0)
            return None
        h, w = depth.shape
        x, y, bw, bh = self.bbox
        cx_, cy_ = x + bw / 2.0, y + bh / 2.0
        def rect(scale):
            return (max(0, int(cx_ - bw * scale / 2)), max(0, int(cy_ - bh * scale / 2)),
                    min(w, int(cx_ + bw * scale / 2)), min(h, int(cy_ + bh * scale / 2)))
        ax0, ay0, ax1, ay1 = rect(self.ann_scale)
        ex0, ey0, ex1, ey1 = rect(self.excl_scale)
        s = self.stride
        sub = depth[ay0:ay1:s, ax0:ax1:s]
        vs, us = np.mgrid[ay0:ay1:s, ax0:ax1:s]
        keep = (sub > self.min_depth) & (sub < self.max_depth) & ~((us >= ex0) & (us < ex1) & (vs >= ey0) & (vs < ey1))
        if self.ring_gate > 0 and keep.sum() >= self.plane_min_pts:
            # リング内の大多数は作業台: 深度の中央値まわりだけ残す（遠い床・近い物体を除外）
            zmed = float(np.median(sub[keep]))
            gated = keep & (np.abs(sub - zmed) < self.ring_gate)
            if gated.sum() >= self.plane_min_pts:
                keep = gated
        Z = sub[keep]
        if Z.size < self.plane_min_pts:
            self.get_logger().warn(f'plane: too few table depth points around object ({Z.size} < {self.plane_min_pts}); '
                                   f'ring {ax0}-{ax1}x{ay0}-{ay1}. Object too close/large in view?', throttle_duration_sec=2.0)
            return None
        U, V = us[keep], vs[keep]
        P = np.c_[(U - self.cx) * Z / self.fx, (V - self.cy) * Z / self.fy, Z]
        cands = []   # (inliers, n, d)
        for _ in range(self.ransac_iters):
            i = self._rng.choice(len(P), 3, replace=False)
            n = np.cross(P[i[1]] - P[i[0]], P[i[2]] - P[i[0]])
            nn = np.linalg.norm(n)
            if nn < 1e-9:
                continue
            n /= nn
            d = -n @ P[i[0]]
            inl = int((np.abs(P @ n + d) < self.inlier_m).sum())
            cands.append((inl, n, d))
        if not cands:
            return None
        best_inl = max(c[0] for c in cands)
        if self.plane_prefer_lowest:
            # 有力候補（最大の 70% 以上）の中で、カメラから最も遠い = 最も低い平面。リングに入った物（ホットプレートの天板など）
            # の上面は台より高く近いので除ける。床はリングの深度ゲート（中央値 ±10 cm）で既に除外されている
            strong = [c for c in cands if c[0] >= 0.7 * best_inl]
            inl_, n, d = max(strong, key=lambda c: abs(c[2]))
        else:
            inl_, n, d = max(cands, key=lambda c: c[0])
        sel = np.abs(P @ n + d) < self.inlier_m
        Q = P[sel]
        if len(Q) < 50:
            return None
        cen = Q.mean(0)
        C = (Q - cen).T @ (Q - cen) / len(Q)
        _, Vv = np.linalg.eigh(C)
        n = Vv[:, 0]
        if n[2] > 0:          # 法線はカメラ側（-z 方向）を向ける
            n = -n
        d = -n @ cen
        r = Q @ n + d
        rms, inl = float(np.sqrt((r ** 2).mean())), float(sel.mean())
        ang = math.acos(min(1.0, abs(float(n[2]))))      # 法線と光軸の角
        if inl < self.plane_min_inl or rms > self.plane_max_rms or ang > self.plane_max_angle:
            self.get_logger().warn(f'table plane rejected: inliers={inl:.0%} rms={rms*1000:.1f}mm angle={math.degrees(ang):.0f}deg',
                                   throttle_duration_sec=2.0)
            return None
        return n, d, rms, inl

    def point_on_plane(self, u: float, v: float, plane, lift: float = 0.0, drop_to_plane: bool = False):
        """画素 (u, v) の視線と平面の交点（object_height だけ法線方向に持ち上げ）。
        lift > 0 のときは、台平面をカメラ側（上）へ lift だけ平行移動した面との交点を返す。
        drop_to_plane のときは、その交点を法線方向に lift だけ戻して台平面上の点として返す（x,y は視差補正済み、z は台面）。
        台より高い所にある点（取っ手の板など）を台平面で切ると、カメラから遠ざかる向きに
        高さ × (水平距離/カメラ高さ) だけずれるので、その点の実際の高さの面で切る。"""
        n, d, _, _ = plane
        n_up = None
        if lift:
            # 原点（カメラ）が法線の正側に来るよう符号をそろえ、カメラ側へ lift だけずらす
            if d < 0:
                n, d = -n, -d
            n_up = n
            d = d - lift
        ray = np.array([(u - self.cx) / self.fx, (v - self.cy) / self.fy, 1.0])
        denom = n @ ray
        if abs(denom) < 1e-6:
            return None
        t = -d / denom
        pt = ray * t
        if t <= 0 or not (self.min_depth < pt[2] < self.max_depth):
            self.get_logger().warn(f'ray/plane intersection out of range (z={pt[2]:.3f} m): skipped',
                                   throttle_duration_sec=2.0)
            return None
        if lift and drop_to_plane and n_up is not None:
            pt = pt - n_up * lift
        return pt + n * self.object_height

    def on_handle_pixel(self, msg: PointStamped):
        self._last_handle_px = (msg.point.x, msg.point.y, msg.point.z)

    def on_pixel(self, msg: PointStamped):
        self._n_pixel += 1
        if self.fx is None or not self.depths:
            self.get_logger().warn('waiting for camera_info / depth', throttle_duration_sec=5.0)
            return
        frame, depth, dt = self.depth_for(self._stamp_to_sec(msg.header.stamp))
        if depth is None:
            self.get_logger().warn(f'no depth frame within {self.max_stamp_diff*1000:.0f} ms (diff {dt*1000:.0f} ms)',
                                   throttle_duration_sec=5.0)
            return

        u, v = msg.point.x, msg.point.y
        now = time.monotonic()
        plane = self._last_plane
        if plane is None or now - self._last_plane_t > self.plane_period:
            fitted = self.fit_table_plane(depth)
            if fitted is not None:
                plane = self._last_plane = fitted
                self._last_plane_t = now
        pt = self.point_on_plane(u, v, plane, lift=self.plate_height, drop_to_plane=True) if plane is not None else None   # 縁の高さで切り、台面に落とす
        if pt is None:
            self.get_logger().warn(f'no 3D estimate at ({u:.0f},{v:.0f}): table plane {"ok" if plane else "none"}',
                                   throttle_duration_sec=2.0)
            return
        x, y, z = pt

        cam = PointStamped()
        cam.header.stamp = msg.header.stamp
        cam.header.frame_id = frame or self.camera_frame
        cam.point.x, cam.point.y, cam.point.z = float(x), float(y), float(z)
        self.cam_pub.publish(cam)
        self._n_cam += 1
        self.publish_marker(cam)

        # STEP 4: base_frame へ変換。コールバックを止めないよう待たずに引く。
        # 画像時刻の TF が無ければ最新 TF で代用（静止アームなら差は無い）。
        tf = None
        for when in (cam.header.stamp, rclpy.time.Time()):
            try:
                tf = self.tf_buffer.lookup_transform(self.base_frame, cam.header.frame_id, when)
                break
            except TransformException as exc:
                last_exc = exc
        if tf is None:
            self.get_logger().warn(f'TF {self.base_frame}<-{cam.header.frame_id} unavailable: {last_exc}',
                                   throttle_duration_sec=10.0)
            return
        base = do_transform_point(cam, tf)
        base.header.stamp = cam.header.stamp
        if not (self.base_z_min <= base.point.z <= self.base_z_max):
            self.get_logger().warn(f'object at base z={base.point.z:+.3f} m is outside [{self.base_z_min:+.2f}, {self.base_z_max:+.2f}] '
                                   '(floor or another red object?): skipped', throttle_duration_sec=2.0)
            return
        self.base_pub.publish(base)
        self._n_base += 1
        # 取っ手の把持点も同じ平面で base へ
        if self._last_handle_px is not None:
            hu, hv, _ = self._last_handle_px
            hp = self.point_on_plane(hu, hv, plane, lift=self.handle_height)
            if hp is not None:
                hc = PointStamped(); hc.header = cam.header
                hc.point.x, hc.point.y, hc.point.z = float(hp[0]), float(hp[1]), float(hp[2])
                hb = do_transform_point(hc, tf); hb.header.stamp = cam.header.stamp
                self.handle_base_pub.publish(hb)
        if self.filter_window > 0:
            self._base_hist.append((base.point.x, base.point.y, base.point.z))
            if len(self._base_hist) >= min(5, self.filter_window):
                med = np.median(np.array(self._base_hist), axis=0)
                f = PointStamped(); f.header = base.header
                f.point.x, f.point.y, f.point.z = float(med[0]), float(med[1]), float(med[2])
                self.base_filt_pub.publish(f)

    def publish_marker(self, pt: PointStamped):
        m = Marker()
        m.header = pt.header
        m.ns, m.id, m.type, m.action = 'object', 0, Marker.SPHERE, Marker.ADD
        m.pose.position = pt.point
        m.pose.orientation.w = 1.0
        m.scale.x = m.scale.y = m.scale.z = 0.03
        m.color = ColorRGBA(r=1.0, g=0.2, b=0.2, a=0.9)
        m.lifetime = Duration(seconds=0.5).to_msg()
        self.marker_pub.publish(m)

    def _report(self):
        if self._n_pixel:
            pl = self._last_plane
            extra = f' plane rms={pl[2]*1000:.1f}mm inliers={pl[3]:.0%}' if pl else ''
            self.get_logger().info(
                f'last 5 s: pixel={self._n_pixel} camera_xyz={self._n_cam} base_xyz={self._n_base}{extra}')
        self._n_pixel = self._n_cam = self._n_base = 0


def main(args=None):
    rclpy.init(args=args)
    node = ObjectPoseEstimator()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except RuntimeError:
        # Humble: SIGINT 直後に spin 内の take_message が RuntimeError を出すことがある
        if rclpy.ok():
            raise
    finally:
        if rclpy.ok():
            node.destroy_node()
            try:
                rclpy.shutdown()
            except Exception:  # noqa: BLE001  (already shut down by signal handler)
                pass


if __name__ == '__main__':
    main()
