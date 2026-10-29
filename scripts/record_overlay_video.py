#!/usr/bin/env python3
"""動作中のカメラ映像を、検出結果（トレイの円・取っ手）、YOLO の枠、controller の状態、推定座標つきで動画に保存する。

  python3 scripts/record_overlay_video.py                      # ~/videos/pickplace_YYYYmmdd_HHMMSS.mp4 に保存、Ctrl+C で終了
  python3 scripts/record_overlay_video.py --out run1.mp4 --fps 15 --duration 180
  python3 scripts/record_overlay_video.py --layout single      # 検出画像だけ（横長にしない）

レイアウト side: 左 = object_detector のデバッグ画像（円・取っ手・中心）、右 = load_detector のデバッグ画像（YOLO の枠、LOADED 判定）。
下の帯に controller の status（直近 4 行）、トレイ中心と把持点の base 座標、載せ物の判定、関節角、経過時間を描く。
YOLO のデバッグ画像は購読者がいるときだけ配信されるので、この録画を始めると自動的に出る。
"""
import argparse
import json
import os
import sys
import threading
import time

import cv2
import numpy as np
import rclpy
from geometry_msgs.msg import PointStamped
from rclpy.node import Node
from sensor_msgs.msg import Image, JointState
from std_msgs.msg import String

JOINTS = ['shoulder_pan', 'shoulder_lift', 'elbow_flex', 'wrist_flex', 'wrist_roll', 'gripper']
SHORT = ['pan', 'lift', 'elbow', 'wflex', 'wroll', 'grip']


class Recorder(Node):
    def __init__(self, a):
        super().__init__('overlay_recorder')
        self.a = a; self.lock = threading.Lock()
        self.img = {}; self.t_img = {}
        self.status = []; self.plate = None; self.handle_pt = None; self.load = None; self.js = None
        self.create_subscription(Image, '/camera/camera/color/image_raw', lambda m: self.on_img('raw', m), 1)
        self.create_subscription(Image, '/so101_pick_place/debug_image', lambda m: self.on_img('det', m), 1)
        self.create_subscription(Image, '/so101_pick_place/load_debug_image', lambda m: self.on_img('load', m), 1)
        self.create_subscription(String, '/so101_pick_place/status', self.on_status, 20)
        self.create_subscription(String, '/so101_pick_place/tray_load', self.on_load, 10)
        self.create_subscription(PointStamped, '/so101_pick_place/object_point_base_filtered', lambda m: setattr(self, 'plate', (m.point.x, m.point.y, m.point.z)), 10)
        self.create_subscription(PointStamped, '/so101_pick_place/handle_point_base', lambda m: setattr(self, 'handle_pt', (m.point.x, m.point.y, m.point.z)), 10)
        self.create_subscription(JointState, '/so101_follower/joint_states', self.on_js, 10)

    def on_img(self, k, m):
        a = np.frombuffer(m.data, np.uint8).reshape(m.height, m.width, -1)
        if m.encoding == 'rgb8':
            a = cv2.cvtColor(a, cv2.COLOR_RGB2BGR)
        with self.lock:
            self.img[k] = a; self.t_img[k] = time.time()

    def on_status(self, m):
        with self.lock:
            self.status.append((time.time(), m.data)); self.status = self.status[-6:]

    def on_load(self, m):
        try:
            self.load = json.loads(m.data)
        except ValueError:
            pass

    def on_js(self, m):
        d = dict(zip(m.name, m.position)); self.js = [d.get(j) for j in JOINTS]

    def compose(self, t0):
        with self.lock:
            img = dict(self.img); t_img = dict(self.t_img); status = list(self.status)
        now = time.time()
        fresh = lambda k: k in img and now - t_img[k] < 1.0
        base = img.get('det') if fresh('det') else img.get('raw')
        if base is None:
            return None
        H, W = base.shape[:2]
        if self.a.layout == 'side':
            right = img.get('load') if fresh('load') else None
            if right is None:
                right = np.zeros_like(base); cv2.putText(right, 'load_detector: no image', (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (128, 128, 128), 2)
            elif right.shape[:2] != (H, W):
                right = cv2.resize(right, (W, H))
            top = np.hstack([base, right])
        else:
            top = base
        TW = top.shape[1]; band = np.zeros((self.a.band, TW, 3), np.uint8); band[:] = (30, 30, 30)
        y = 22
        for ts, s in status[-4:]:
            s = s[:150]
            col = (0, 255, 0) if 'SUCCESS' in s or 'HELD' in s or 'LOADED' in s else (0, 0, 255) if 'FAIL' in s or 'MISSED' in s or 'implausible' in s else (255, 255, 255)
            cv2.putText(band, f'[{ts - t0:6.1f}s] {s}', (8, y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, col, 1); y += 20
        info = []
        if self.plate: info.append('plate (%+.3f, %+.3f, %+.3f)' % self.plate)
        if self.handle_pt: info.append('handle (%+.3f, %+.3f, %+.3f)' % self.handle_pt)
        if self.load:
            items = ', '.join(f"{it['label']} {it['conf']:.2f}" for it in self.load.get('items', []) if it.get('inside') and not it.get('ignored'))
            info.append(f"load: {'YES' if self.load.get('loaded') else 'no'} [{items or '-'}] by {','.join(self.load.get('loaded_by', []))}")
        cv2.putText(band, ' | '.join(info)[:200], (8, y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1); y += 20
        if self.js and all(v is not None for v in self.js):
            cv2.putText(band, 'joints(deg) ' + ' '.join(f'{j}={np.degrees(v):+.0f}' for j, v in zip(SHORT, self.js)), (8, y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 200), 1)
        cv2.putText(band, time.strftime('%Y-%m-%d %H:%M:%S') + f'  t={now - t0:.1f}s', (TW - 300, self.a.band - 8), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (160, 160, 160), 1)
        return np.vstack([top, band])


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--out', default=os.path.expanduser(time.strftime('~/videos/pickplace_%Y%m%d_%H%M%S.mp4')))
    ap.add_argument('--fps', type=float, default=15.0); ap.add_argument('--duration', type=float, default=0.0, help='秒。0 = Ctrl+C まで')
    ap.add_argument('--layout', choices=['side', 'single'], default='side'); ap.add_argument('--band', type=int, default=130)
    a = ap.parse_args()
    os.makedirs(os.path.dirname(a.out) or '.', exist_ok=True)
    rclpy.init(); node = Recorder(a)
    def _spin():
        try:
            rclpy.spin(node)
        except Exception:  # noqa: BLE001  (Ctrl+C で rclpy 側が先に shutdown したときの ExternalShutdownException を黙らせる)
            pass
    th = threading.Thread(target=_spin, daemon=True); th.start()
    t0 = time.time(); writer = None; n = 0
    print(f'recording -> {a.out}  ({a.fps} fps, layout {a.layout}). Ctrl+C で終了')
    try:
        while rclpy.ok():
            frame = node.compose(t0)
            if frame is not None:
                if writer is None:
                    h, w = frame.shape[:2]
                    fourcc = cv2.VideoWriter_fourcc(*('avc1' if a.out.endswith('.mp4') else 'MJPG'))   # H.264（標準の動画プレーヤーで再生可能。mp4v は GNOME Videos で再生できなかった）
                    writer = cv2.VideoWriter(a.out, fourcc, a.fps, (w, h))
                    if not writer.isOpened():
                        print('VideoWriter を開けません（mp4v 非対応なら --out xxx.avi を試す）'); return 1
                    print(f'frame {w}x{h}')
                writer.write(frame); n += 1
            if a.duration and time.time() - t0 >= a.duration:
                break
            time.sleep(max(0.0, (n + 1) / a.fps - (time.time() - t0)))
    except KeyboardInterrupt:
        pass
    finally:
        if writer is not None:
            writer.release()
        print(f'saved {a.out}: {n} frames, {n / a.fps:.1f} s')
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
