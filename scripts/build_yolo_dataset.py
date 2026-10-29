#!/usr/bin/env python3
"""アノテーション済みフォルダから YOLO 学習用データセット（train/val 分割 + data.yaml）を作る。

  python3 scripts/build_yolo_dataset.py ~/.so101_models/dataset_raw/session_20261025 --out ~/.so101_models/datasets/sweets_v1

- 画像単位で 80/20 に分割（撮影元 oak / phone ごとに層化、乱数固定）。
- labels/<名前>.skip がある画像は除外。ラベルが空の画像は「物なし」の負例として含める。
- meta.csv にトレイの円がある学習画像は、推論時と同じ「外接円 × 1.3 の正方形切り出し」も追加の学習サンプルにする
  （ラベルは切り出しに合わせて変換・クリップ）。検証画像には切り出しを加えない（評価を水増ししない）。
- クラスは classes.txt の順（pancake, candy, cake）。
"""
import argparse
import csv
import os
import random
import shutil

import cv2


def read_labels(p):
    if not os.path.exists(p):
        return []
    return [tuple(map(float, l.split())) for l in open(p) if len(l.split()) == 5]


def write_labels(p, rows):
    open(p, 'w').write(''.join(f'{int(c)} {cx:.6f} {cy:.6f} {w:.6f} {h:.6f}\n' for c, cx, cy, w, h in rows))


def crop_with_labels(img, rows, x0, y0, x1, y1, min_keep=0.3):
    H, W = img.shape[:2]; cw, ch = x1 - x0, y1 - y0
    out = []
    for c, cx, cy, w, h in rows:
        bx0, by0, bx1, by1 = (cx - w / 2) * W, (cy - h / 2) * H, (cx + w / 2) * W, (cy + h / 2) * H
        ix0, iy0, ix1, iy1 = max(bx0, x0), max(by0, y0), min(bx1, x1), min(by1, y1)
        if ix1 <= ix0 or iy1 <= iy0:
            continue
        if (ix1 - ix0) * (iy1 - iy0) < min_keep * (bx1 - bx0) * (by1 - by0):
            continue   # 切り出しで大部分が切れた枠は捨てる
        out.append((c, ((ix0 + ix1) / 2 - x0) / cw, ((iy0 + iy1) / 2 - y0) / ch, (ix1 - ix0) / cw, (iy1 - iy0) / ch))
    return img[y0:y1, x0:x1], out


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('src'); ap.add_argument('--out', required=True); ap.add_argument('--val', type=float, default=0.2)
    ap.add_argument('--seed', type=int, default=0); ap.add_argument('--roi_margin', type=float, default=1.3); ap.add_argument('--no-crops', action='store_true')
    a = ap.parse_args()
    src, out = os.path.expanduser(a.src), os.path.expanduser(a.out)
    classes = open(os.path.join(src, 'classes.txt')).read().split()
    imgs = sorted(f for f in os.listdir(os.path.join(src, 'images')) if f.lower().endswith(('.png', '.jpg', '.jpeg')))
    stem = lambda f: os.path.splitext(f)[0]
    imgs = [f for f in imgs if not os.path.exists(os.path.join(src, 'labels', stem(f) + '.skip'))]
    circles = {}
    mp = os.path.join(src, 'meta.csv')
    if os.path.exists(mp):
        for r in csv.DictReader(open(mp)):
            if r.get('r'):
                circles[r['file']] = (float(r['cu']), float(r['cv']), float(r['r']))
    # 層化分割
    random.seed(a.seed); groups = {}
    for f in imgs:
        groups.setdefault('oak' if f.startswith('pancake_') else 'phone', []).append(f)
    train, val = [], []
    for g, fs in groups.items():
        random.shuffle(fs); k = max(1, int(round(len(fs) * a.val))); val += fs[:k]; train += fs[k:]
    if os.path.exists(out):
        shutil.rmtree(out)
    for split in ('train', 'val'):
        os.makedirs(os.path.join(out, 'images', split)); os.makedirs(os.path.join(out, 'labels', split))
    stats = {'train': [0, 0, 0], 'val': [0, 0, 0]}   # images, boxes, crops
    for split, files in (('train', train), ('val', val)):
        for f in files:
            rows = read_labels(os.path.join(src, 'labels', stem(f) + '.txt'))
            shutil.copy(os.path.join(src, 'images', f), os.path.join(out, 'images', split, f))
            write_labels(os.path.join(out, 'labels', split, stem(f) + '.txt'), rows)
            stats[split][0] += 1; stats[split][1] += len(rows)
            if split == 'train' and not a.no_crops and f in circles:
                img = cv2.imread(os.path.join(src, 'images', f)); H, W = img.shape[:2]
                cu, cv_, r = circles[f]; m = r * a.roi_margin
                x0, y0, x1, y1 = int(max(0, cu - m)), int(max(0, cv_ - m)), int(min(W, cu + m)), int(min(H, cv_ + m))
                if x1 - x0 >= 64 and y1 - y0 >= 64:
                    crop, crows = crop_with_labels(img, rows, x0, y0, x1, y1)
                    name = stem(f) + '_crop.png'
                    cv2.imwrite(os.path.join(out, 'images', split, name), crop)
                    write_labels(os.path.join(out, 'labels', split, stem(f) + '_crop.txt'), crows)
                    stats[split][2] += 1; stats[split][1] += len(crows)
    with open(os.path.join(out, 'data.yaml'), 'w') as fh:
        fh.write(f'path: {out}\ntrain: images/train\nval: images/val\nnames:\n' + ''.join(f'  {i}: {n}\n' for i, n in enumerate(classes)))
    with open(os.path.join(out, 'split.txt'), 'w') as fh:
        fh.write('train\n' + '\n'.join(train) + '\nval\n' + '\n'.join(val) + '\n')
    for split in ('train', 'val'):
        n, b, c = stats[split]; print(f'{split}: {n} images (+{c} tray crops), {b} boxes')
    print('classes:', classes); print('data.yaml:', os.path.join(out, 'data.yaml'))


if __name__ == '__main__':
    main()
