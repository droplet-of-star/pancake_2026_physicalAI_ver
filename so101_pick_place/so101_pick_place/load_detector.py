#!/usr/bin/env python3
"""トレイの上に物（パンケーキ・アメ・ケーキ）が載ったかを、自前データで微調整した YOLOv8 で判定するノード。

画像全体で推論し、
  - `tray` クラスの枠（最も確信度の高い 1 つ）を /so101_pick_place/tray_bbox に出す
    （object_detector が円探索の ROI に使う。枠は円盤だけを囲む約束）
  - それ以外のクラスのうち、トレイの円の内側に中心がある検出を「載せ物」とみなす
結果は JSON 文字列で /so101_pick_place/tray_load に出す:
  {"loaded": true, "loaded_by": ["yolo"], "n_inside": 1, "tray_seen": true,
   "items": [{"label": "cake", "conf": 0.83, "u": 512.0, "v": 180.0, "inside": true, "ignored": false, "box": [...]}],
   "age_s": 0.03, "infer_ms": 40, "model": "sweets_v2.pt"}
トレイの円は object_detector の外接円（/so101_pick_place/object_bbox）が新鮮ならそれ、無ければ tray 枠に内接する円を使う。
controller の `wait_load` / `auto` はこのトピックを見て、載ったことを確認してから pick & place を始める。

PyTorch は専用 venv（~/.venvs/yolo-gpu）に隔離してあるため、scripts/run_load_detector.sh から起動する。
"""
import json
import os
import time

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, RegionOfInterest
from std_msgs.msg import String


class LoadDetector(Node):
    def __init__(self):
        super().__init__('load_detector')
        p = self.declare_parameter
        p('image_topic', '/camera/camera/color/image_raw')
        p('bbox_topic', '/so101_pick_place/object_bbox')
        p('model', 'sweets_v2.pt')               # 重みのファイル名（model_dir 内）または絶対パス
        p('model_dir', '~/.so101_models')
        p('trigger_labels', [''])                # loaded 判定に使うラベル（空なら円内の全検出）
        p('conf', 0.25)
        p('imgsz', 416)
        p('rate_hz', 2.0)
        p('inside_scale', 1.0)                   # 検出の中心が 半径 × この倍率 の内側なら「載っている」
        p('bbox_max_age_s', 1.0)
        p('max_box_frac', 0.7)                   # 検出枠の長辺が トレイ直径 × これ を超えるものはトレイ自体とみなして無視
        p('tray_conf', 0.3)                      # tray 枠として採用する確信度の下限
        p('device', 'auto')                      # auto: CUDA があれば GPU、無ければ CPU
        p('publish_debug', True)
        g = lambda n: self.get_parameter(n).value
        self.model_name = str(g('model')); self.model_dir = os.path.expanduser(str(g('model_dir')))
        self.trigger = [c.lower() for c in g('trigger_labels') if c]
        self.conf = float(g('conf')); self.imgsz = int(g('imgsz'))
        self.inside_scale = float(g('inside_scale')); self.bbox_max_age = float(g('bbox_max_age_s'))
        self.max_box_frac = float(g('max_box_frac')); self.tray_conf = float(g('tray_conf'))
        self.debug = bool(g('publish_debug'))
        self.device = str(g('device'))
        if self.device == 'auto':
            try:
                import torch
                self.device = '0' if torch.cuda.is_available() else 'cpu'
            except Exception:  # noqa: BLE001
                self.device = 'cpu'

        self.img = None; self.img_time = 0.0; self.img_enc = 'bgr8'
        self.bbox = None; self.bbox_time = 0.0
        self.create_subscription(Image, str(g('image_topic')), self.on_image, 1)
        self.create_subscription(RegionOfInterest, str(g('bbox_topic')), self.on_bbox, 10)
        self.pub = self.create_publisher(String, '/so101_pick_place/tray_load', 10)
        self.dbg_pub = self.create_publisher(Image, '/so101_pick_place/load_debug_image', 1)
        self.tray_pub = self.create_publisher(RegionOfInterest, '/so101_pick_place/tray_bbox', 10)   # 学習モデルによるトレイの枠（フル解像度の画素）

        self.model = self.load_model()
        self.timer = self.create_timer(1.0 / max(0.1, float(g('rate_hz'))), self.tick)
        self.n_infer = 0; self.last_report = time.time()
        self.get_logger().info(f'load_detector: model={self.model_name} trigger={self.trigger or "(any object inside tray)"} '
                               f'conf={self.conf} tray_conf={self.tray_conf} imgsz={self.imgsz} device={self.device}')

    # ------------------------------------------------------------------
    def load_model(self):
        from ultralytics import YOLO   # 遅延 import（venv 違いの起動エラーを分かりやすく）
        path = self.model_name if os.path.isabs(self.model_name) else os.path.join(self.model_dir, self.model_name)
        if not os.path.exists(path):
            raise FileNotFoundError(f'model not found: {path}（学習済み重みを model_dir に置くか、model に絶対パスを指定）')
        return YOLO(path)

    def on_image(self, msg: Image):
        a = np.frombuffer(msg.data, np.uint8).reshape(msg.height, msg.width, -1)
        self.img = a; self.img_enc = msg.encoding; self.img_time = time.time()

    def on_bbox(self, msg: RegionOfInterest):
        self.bbox = (int(msg.x_offset), int(msg.y_offset), int(msg.width), int(msg.height)); self.bbox_time = time.time()

    # ------------------------------------------------------------------
    def tick(self):
        now = time.time()
        if self.img is None:
            return
        out = {'loaded': False, 'n_inside': 0, 'items': [], 'tray_seen': False, 'model': os.path.basename(self.model_name)}
        bgr = self.img if self.img_enc == 'bgr8' else cv2.cvtColor(self.img, cv2.COLOR_RGB2BGR)
        H, W = bgr.shape[:2]
        t0 = time.time()
        res = self.model.predict(bgr, conf=min(self.conf, self.tray_conf), imgsz=self.imgsz, device=self.device, verbose=False)[0]
        infer_ms = (time.time() - t0) * 1000.0
        names = res.names
        items = []; tray_best = None
        for b in res.boxes:
            x0, y0, x1, y1 = [float(v) for v in b.xyxy[0].tolist()]
            label = str(names[int(b.cls[0])]); conf = float(b.conf[0])
            if label.lower() == 'tray':
                if conf >= self.tray_conf and (tray_best is None or conf > tray_best[0]):
                    tray_best = (conf, x0, y0, x1, y1)
                continue
            if conf < self.conf:
                continue
            items.append({'label': label, 'conf': round(conf, 3), 'u': round((x0 + x1) / 2.0, 1), 'v': round((y0 + y1) / 2.0, 1),
                          'inside': False, 'box': [round(x0, 1), round(y0, 1), round(x1, 1), round(y1, 1)]})
        # トレイの基準円: object_detector の外接円（新鮮なら）、無ければ学習モデルの tray 枠に内接する円
        have_bbox = self.bbox is not None and now - self.bbox_time <= self.bbox_max_age
        if have_bbox:
            x, y, w, h = self.bbox; cx, cy, r = x + w / 2.0, y + h / 2.0, max(w, h) / 2.0
        elif tray_best is not None:
            _, tx0, ty0, tx1, ty1 = tray_best; cx, cy, r = (tx0 + tx1) / 2.0, (ty0 + ty1) / 2.0, min(tx1 - tx0, ty1 - ty0) / 2.0
        else:
            out['reason'] = 'tray not seen'; out['infer_ms'] = round(infer_ms)
            self.pub.publish(String(data=json.dumps(out)))
            return
        if tray_best is not None:
            _, tx0, ty0, tx1, ty1 = tray_best
            roi = RegionOfInterest(); roi.x_offset, roi.y_offset = int(max(0, tx0)), int(max(0, ty0))
            roi.width, roi.height = int(max(1, tx1 - tx0)), int(max(1, ty1 - ty0)); roi.do_rectify = False
            self.tray_pub.publish(roi)
        for it in items:
            bw, bh = it['box'][2] - it['box'][0], it['box'][3] - it['box'][1]
            it['inside'] = ((it['u'] - cx) ** 2 + (it['v'] - cy) ** 2) ** 0.5 <= r * self.inside_scale
            it['ignored'] = max(bw, bh) > self.max_box_frac * 2.0 * r
        inside_items = [it for it in items if it['inside'] and not it['ignored'] and (not self.trigger or it['label'].lower() in self.trigger)]
        loaded_by = ['yolo'] if inside_items else []
        out.update({'loaded': bool(inside_items), 'loaded_by': loaded_by, 'n_inside': len(inside_items), 'items': items, 'tray_seen': True,
                    'age_s': round(now - self.img_time, 3), 'infer_ms': round(infer_ms)})
        self.pub.publish(String(data=json.dumps(out)))
        self.n_infer += 1
        if now - self.last_report > 5.0:
            self.last_report = now
            self.get_logger().info(f'last 5 s: infer={self.n_infer} ({infer_ms:.0f} ms) loaded={out["loaded"]} '
                                   + (', '.join(f'{it["label"]}:{it["conf"]:.2f}{"*" if it["inside"] and not it["ignored"] else ""}' for it in items) or 'no detections'))
            self.n_infer = 0
        if self.debug and self.dbg_pub.get_subscription_count() > 0:
            self.publish_debug(bgr, cx, cy, r, tray_best, items, out['loaded'], infer_ms)

    def publish_debug(self, bgr, cx, cy, r, tray_best, items, loaded, infer_ms):
        dbg = bgr.copy()
        cv2.circle(dbg, (int(cx), int(cy)), int(r), (0, 200, 255), 2)
        if tray_best is not None:
            conf, tx0, ty0, tx1, ty1 = tray_best
            cv2.rectangle(dbg, (int(tx0), int(ty0)), (int(tx1), int(ty1)), (255, 200, 0), 2)
            cv2.putText(dbg, f'tray {conf:.2f}', (int(tx0), max(12, int(ty0) - 4)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 200, 0), 1)
        for it in items:
            c = (0, 255, 0) if (it['inside'] and not it['ignored']) else (0, 0, 255)
            bx = [int(v) for v in it['box']]
            cv2.rectangle(dbg, (bx[0], bx[1]), (bx[2], bx[3]), c, 2)
            cv2.putText(dbg, f'{it["label"]} {it["conf"]:.2f}', (bx[0], max(12, bx[1] - 4)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, c, 1)
        cv2.putText(dbg, f'LOADED: {loaded} ({infer_ms:.0f} ms)', (8, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.6,
                    (0, 255, 0) if loaded else (0, 0, 255), 2)
        msg = Image(); msg.height, msg.width = dbg.shape[:2]; msg.encoding = 'bgr8'; msg.step = dbg.shape[1] * 3
        msg.data = dbg.tobytes(); msg.header.stamp = self.get_clock().now().to_msg(); msg.header.frame_id = 'camera_color_optical_frame'
        self.dbg_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = LoadDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.destroy_node(); rclpy.shutdown()


if __name__ == '__main__':
    main()
