#!/usr/bin/env python3
"""高解像度の写真（スマホ等）を OAK と同程度の大きさに縮小する。YOLO のラベルは正規化座標なのでそのまま有効。

  python3 scripts/downscale_images.py ~/.so101_models/dataset_raw/session_20261025            # 長辺 640 px に
  python3 scripts/downscale_images.py <dir> --long-side 960 --prefix IMG_

元のファイルは <dir>/images_orig/ に移してから、同名で縮小版を images/ に書く（JPEG 品質 92）。
長辺がすでに --long-side 以下の画像は触らない。
"""
import argparse
import os
import shutil

import cv2


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('dir'); ap.add_argument('--long-side', type=int, default=640); ap.add_argument('--prefix', default='')
    a = ap.parse_args()
    src = os.path.join(a.dir, 'images'); orig = os.path.join(a.dir, 'images_orig'); os.makedirs(orig, exist_ok=True)
    n = 0
    for f in sorted(os.listdir(src)):
        if a.prefix and not f.startswith(a.prefix):
            continue
        if not f.lower().endswith(('.jpg', '.jpeg', '.png')):
            continue
        p = os.path.join(src, f); im = cv2.imread(p)
        if im is None:
            continue
        H, W = im.shape[:2]; L = max(H, W)
        if L <= a.long_side:
            continue
        sc = a.long_side / L
        small = cv2.resize(im, (int(round(W * sc)), int(round(H * sc))), interpolation=cv2.INTER_AREA)
        shutil.move(p, os.path.join(orig, f))
        cv2.imwrite(p, small, [cv2.IMWRITE_JPEG_QUALITY, 92] if f.lower().endswith(('.jpg', '.jpeg')) else [])
        n += 1; print(f'{f}: {W}x{H} -> {small.shape[1]}x{small.shape[0]}')
    print(f'downscaled {n} images; originals in {orig}')


if __name__ == '__main__':
    main()
