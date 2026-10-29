#!/usr/bin/env python3
"""OAK-D のカラー画像から、物体検出の学習用の写真を保存する。

保存するもの（出力ディレクトリの下）:
  images/<tag>_<番号>.png   カメラのフル画像（現在の解像度のまま）
  crops/<tag>_<番号>.png    トレイの外接円 × roi_margin の正方形切り出し（build_yolo_dataset.py が追加サンプルに使う）
  meta.csv                  ファイル名, tag, 時刻, 円の中心 (u,v), 半径, 切り出し位置 (x0,y0,x1,y1)

使い方:
  # Enter を押すたびに 1 枚保存（q + Enter で終了）
  python3 scripts/capture_training_images.py --tag candy
  # 2 秒おきに自動で 30 枚（その間に物の位置や向きを変える）
  python3 scripts/capture_training_images.py --tag cake --interval 2 --count 30
  # トレイが検出されていなくてもフル画像だけ保存する
  python3 scripts/capture_training_images.py --tag empty --no-crop

前提: oak_camera と object_detector が動いていること（切り出しにトレイの外接円 /so101_pick_place/object_bbox を使う）。
アームは実行時と同じ観測姿勢にしておく（controller に observe か home を送る）と、学習画像と実行時の見え方がそろう。
"""
import argparse
import csv
import os
import sys
import threading
import time

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, RegionOfInterest


class Capture(Node):
    def __init__(self, args):
        super().__init__('capture_training_images')
        self.args = args
        self.img = None; self.img_time = 0.0; self.enc = 'bgr8'
        self.bbox = None; self.bbox_time = 0.0
        self.create_subscription(Image, args.image_topic, self.on_image, 1)
        self.create_subscription(RegionOfInterest, args.bbox_topic, self.on_bbox, 10)
        os.makedirs(os.path.join(args.out, 'images'), exist_ok=True)
        os.makedirs(os.path.join(args.out, 'crops'), exist_ok=True)
        self.meta_path = os.path.join(args.out, 'meta.csv')
        new = not os.path.exists(self.meta_path)
        self.meta = open(self.meta_path, 'a', newline='')
        self.writer = csv.writer(self.meta)
        if new:
            self.writer.writerow(['file', 'tag', 'time', 'cu', 'cv', 'r', 'x0', 'y0', 'x1', 'y1', 'width', 'height'])
        existing = [f for f in os.listdir(os.path.join(args.out, 'images')) if f.startswith(args.tag + '_')]
        self.n = len(existing)

    def on_image(self, msg: Image):
        self.img = np.frombuffer(msg.data, np.uint8).reshape(msg.height, msg.width, -1); self.enc = msg.encoding; self.img_time = time.time()

    def on_bbox(self, msg: RegionOfInterest):
        self.bbox = (msg.x_offset, msg.y_offset, msg.width, msg.height); self.bbox_time = time.time()

    def save_one(self):
        if self.img is None or time.time() - self.img_time > 1.0:
            print('  画像が届いていません（oak_camera は動いていますか）'); return False
        bgr = self.img if self.enc == 'bgr8' else cv2.cvtColor(self.img, cv2.COLOR_RGB2BGR)
        H, W = bgr.shape[:2]
        name = f'{self.args.tag}_{self.n:04d}.png'
        cv2.imwrite(os.path.join(self.args.out, 'images', name), bgr)
        row = [name, self.args.tag, f'{time.time():.2f}', '', '', '', '', '', '', '', W, H]
        tray = self.bbox is not None and time.time() - self.bbox_time < 1.0
        if tray and not self.args.no_crop:
            x, y, w, h = self.bbox; cu, cv_, r = x + w / 2.0, y + h / 2.0, max(w, h) / 2.0
            m = r * self.args.roi_margin
            x0, y0, x1, y1 = int(max(0, cu - m)), int(max(0, cv_ - m)), int(min(W, cu + m)), int(min(H, cv_ + m))
            cv2.imwrite(os.path.join(self.args.out, 'crops', name), bgr[y0:y1, x0:x1])
            row[3:10] = [f'{cu:.1f}', f'{cv_:.1f}', f'{r:.1f}', x0, y0, x1, y1]
            info = f'tray circle ({cu:.0f},{cv_:.0f}) r={r:.0f} -> crop {x1 - x0}x{y1 - y0}'
        else:
            info = 'tray NOT detected: full image only' if not self.args.no_crop else 'full image only'
        self.writer.writerow(row); self.meta.flush()
        self.n += 1
        print(f'  saved {name}  ({info})')
        return True


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--out', default=os.path.expanduser('~/.so101_models/dataset_raw/session_' + time.strftime('%Y%m%d')))
    ap.add_argument('--tag', default='img', help='ファイル名の先頭に付ける語（candy / cake / both / empty など）')
    ap.add_argument('--interval', type=float, default=0.0, help='秒。0 なら Enter で 1 枚ずつ')
    ap.add_argument('--count', type=int, default=0, help='--interval のとき保存する枚数（0 = Ctrl+C まで）')
    ap.add_argument('--roi_margin', type=float, default=1.3)
    ap.add_argument('--no-crop', action='store_true')
    ap.add_argument('--image_topic', default='/camera/camera/color/image_raw')
    ap.add_argument('--bbox_topic', default='/so101_pick_place/object_bbox')
    args = ap.parse_args()

    rclpy.init()
    node = Capture(args)
    th = threading.Thread(target=rclpy.spin, args=(node,), daemon=True); th.start()
    print(f'出力: {args.out}  tag={args.tag}  既存 {node.n} 枚')
    t0 = time.time()
    while node.img is None and time.time() - t0 < 5.0:
        time.sleep(0.1)
    if node.img is None:
        print('画像が届きません。oak_camera を確認してください。'); rclpy.shutdown(); return 1
    print(f'画像 {node.img.shape[1]}x{node.img.shape[0]} 受信中。トレイ検出: {"あり" if node.bbox else "なし（object_detector / 位置を確認）"}')
    try:
        if args.interval > 0:
            k = 0
            while args.count == 0 or k < args.count:
                if node.save_one():
                    k += 1
                time.sleep(args.interval)
        else:
            print('Enter で保存、q + Enter で終了')
            for line in sys.stdin:
                if line.strip().lower() == 'q':
                    break
                node.save_one()
    except KeyboardInterrupt:
        pass
    print(f'合計 {node.n} 枚 ({args.tag})。meta: {node.meta_path}')
    node.meta.close(); rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
