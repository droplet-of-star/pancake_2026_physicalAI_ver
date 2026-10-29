#!/usr/bin/env python3
"""OpenCV だけで動く簡易アノテーションツール（矩形、YOLO 形式）。CVAT が arm64 で動かないときの代替。

  python3 scripts/annotate_boxes.py ~/.so101_models/dataset_raw/session_20261025            # images/ を順に表示
  python3 scripts/annotate_boxes.py <dir> --subdir crops                                     # crops/ を使う
  python3 scripts/annotate_boxes.py <dir> --classes pancake candy cake

保存先: <dir>/labels/<画像名>.txt（YOLO: class cx cy w h 正規化）。既にあるラベル（下書き含む）は読み込んで表示する。

操作:
  マウス左ドラッグ      矩形を描く（現在のクラスで追加）
  左クリック 2 回       ドラッグが効かない環境向け: 1 回目で角、2 回目で対角（移動量 4px 未満のクリックは「点」扱い）
  1〜9                  現在のクラスを選ぶ（最後に描いた矩形のクラスも変える）
  右クリック            クリック位置の矩形を削除
  d / c                 最後の矩形を削除 / 全部削除
  n または Space / p    次の画像 / 前の画像（移動時に自動保存）
  x                     この画像を「除外」に印（labels/<名前>.skip を作る。学習に使わない）
  s / q                 保存 / 保存して終了
"""
import argparse
import os
import sys

import cv2

COLORS = [(0, 255, 0), (0, 200, 255), (255, 128, 0), (255, 0, 255), (0, 128, 255), (200, 200, 0), (128, 0, 255), (0, 255, 200), (255, 255, 255)]


class Annotator:
    def __init__(self, d, subdir, classes, scale, debug=False):
        print('classes:', ' '.join(f'{i+1}:{c}' for i, c in enumerate(classes)))
        self.img_dir = os.path.join(d, subdir); self.lbl_dir = os.path.join(d, 'labels'); os.makedirs(self.lbl_dir, exist_ok=True)
        self.files = sorted(f for f in os.listdir(self.img_dir) if f.lower().endswith(('.png', '.jpg', '.jpeg')))
        if not self.files:
            sys.exit(f'画像がありません: {self.img_dir}')
        self.classes = classes; self.fixed_scale = scale; self.scale = 1.0; self.idx = 0; self.cls = 0
        self.boxes = []   # [cls, x0, y0, x1, y1] 画像座標
        self.drag = None; self.anchor = None; self.debug = debug; self.win = 'annotate (h: help in terminal)'
        open(os.path.join(d, 'classes.txt'), 'w').write('\n'.join(classes) + '\n')
        cv2.namedWindow(self.win, cv2.WINDOW_AUTOSIZE); cv2.setMouseCallback(self.win, self.on_mouse)

    # ---------------- io
    def lbl_path(self, f): return os.path.join(self.lbl_dir, os.path.splitext(f)[0] + '.txt')
    def skip_path(self, f): return os.path.join(self.lbl_dir, os.path.splitext(f)[0] + '.skip')

    def load(self):
        f = self.files[self.idx]; self.img = cv2.imread(os.path.join(self.img_dir, f)); H, W = self.img.shape[:2]
        # 表示倍率: --scale 0 なら画面に収まるよう自動（最大 1400x850）
        self.scale = self.fixed_scale if self.fixed_scale > 0 else min(1400.0 / W, 850.0 / H, 2.0)
        self.boxes = []
        p = self.lbl_path(f)
        if os.path.exists(p):
            for line in open(p):
                v = line.split()
                if len(v) == 5:
                    c, cx, cy, w, h = int(v[0]), *map(float, v[1:])
                    self.boxes.append([c, (cx - w / 2) * W, (cy - h / 2) * H, (cx + w / 2) * W, (cy + h / 2) * H])

    def save(self):
        f = self.files[self.idx]; H, W = self.img.shape[:2]
        lines = [f'{c} {(x0 + x1) / 2 / W:.6f} {(y0 + y1) / 2 / H:.6f} {abs(x1 - x0) / W:.6f} {abs(y1 - y0) / H:.6f}' for c, x0, y0, x1, y1 in self.boxes]
        open(self.lbl_path(f), 'w').write('\n'.join(lines) + ('\n' if lines else ''))

    # ---------------- ui
    def add_box(self, x0, y0, x1, y1):
        H, W = self.img.shape[:2]
        self.boxes.append([self.cls, max(0, min(x0, x1)), max(0, min(y0, y1)), min(W, max(x0, x1)), min(H, max(y0, y1))])
        if self.debug:
            print(f'  box added: {self.boxes[-1]}')

    def on_mouse(self, ev, x, y, flags, _):
        x, y = x / self.scale, y / self.scale
        if self.debug and ev != cv2.EVENT_MOUSEMOVE:
            print(f'  mouse ev={ev} at ({x:.0f},{y:.0f}) drag={self.drag} anchor={self.anchor}')
        if ev == cv2.EVENT_LBUTTONDOWN:
            self.drag = [x, y, x, y]
        elif ev == cv2.EVENT_MOUSEMOVE and self.drag:
            self.drag[2], self.drag[3] = x, y
        elif ev == cv2.EVENT_LBUTTONUP and self.drag:
            x0, y0, x1, y1 = self.drag; self.drag = None
            if abs(x1 - x0) > 4 and abs(y1 - y0) > 4:
                self.add_box(x0, y0, x1, y1); self.anchor = None
            else:
                # ほぼ動かないクリック: 2 クリック方式（1 回目 = 角、2 回目 = 対角）
                if self.anchor is None:
                    self.anchor = (x, y)
                else:
                    ax, ay = self.anchor; self.anchor = None
                    if abs(x - ax) > 4 and abs(y - ay) > 4:
                        self.add_box(ax, ay, x, y)
        elif ev == cv2.EVENT_RBUTTONDOWN:
            for i in range(len(self.boxes) - 1, -1, -1):
                c, x0, y0, x1, y1 = self.boxes[i]
                if x0 <= x <= x1 and y0 <= y <= y1:
                    del self.boxes[i]; break

    def draw(self):
        # 枠や文字は「表示用に縮小・拡大した後」の画像に一定の太さで描く（元画像に描くと、スマホ写真では縮小で線が消える）
        f = self.files[self.idx]; sc = self.scale
        img = self.img if sc == 1.0 else cv2.resize(self.img, None, fx=sc, fy=sc, interpolation=cv2.INTER_AREA if sc < 1 else cv2.INTER_LINEAR)
        P = lambda x, y: (int(round(x * sc)), int(round(y * sc)))
        for c, x0, y0, x1, y1 in self.boxes:
            col = COLORS[c % len(COLORS)]
            cv2.rectangle(img, P(x0, y0), P(x1, y1), col, 2)
            tx, ty = P(x0, y0); cv2.putText(img, self.classes[c] if c < len(self.classes) else str(c), (tx, max(12, ty - 4)), cv2.FONT_HERSHEY_SIMPLEX, 0.55, col, 2)
        if self.drag:
            x0, y0, x1, y1 = self.drag; cv2.rectangle(img, P(x0, y0), P(x1, y1), (255, 255, 255), 1)
        if self.anchor:
            ax, ay = self.anchor; cv2.drawMarker(img, P(ax, ay), (255, 255, 255), cv2.MARKER_CROSS, 24, 2)
        skipped = os.path.exists(self.skip_path(f))
        head = f'[{self.idx + 1}/{len(self.files)}] {f}  class: {self.cls + 1}:{self.classes[self.cls]}  boxes: {len(self.boxes)}' + ('  [SKIP]' if skipped else '')
        cv2.putText(img, head, (6, 18), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 0, 0), 3); cv2.putText(img, head, (6, 18), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 1)
        cv2.imshow(self.win, img)

    def run(self):
        print(__doc__); self.load()
        while True:
            self.draw(); k = cv2.waitKey(20) & 0xFF
            if k == 255:
                continue
            if k in (ord('q'), 27):
                self.save(); break
            if k == ord('s'):
                self.save(); print('saved', self.files[self.idx])
            elif k in (ord('n'), ord(' ')):
                self.save(); self.idx = min(len(self.files) - 1, self.idx + 1); self.load()
            elif k == ord('p'):
                self.save(); self.idx = max(0, self.idx - 1); self.load()
            elif k == ord('d') and self.boxes:
                self.boxes.pop()
            elif k == ord('c'):
                self.boxes = []
            elif k == ord('x'):
                p = self.skip_path(self.files[self.idx])
                if os.path.exists(p): os.remove(p); print('un-skip', self.files[self.idx])
                else: open(p, 'w').close(); print('skip', self.files[self.idx])
            elif ord('1') <= k <= ord('9') and k - ord('1') < len(self.classes):
                self.cls = k - ord('1')
                if self.boxes: self.boxes[-1][0] = self.cls
            elif k == ord('h'):
                print(__doc__)
        cv2.destroyAllWindows()
        n_lbl = sum(1 for f in self.files if os.path.exists(self.lbl_path(f))); n_skip = sum(1 for f in self.files if os.path.exists(self.skip_path(f)))
        print(f'labels: {n_lbl}/{len(self.files)} files, skipped: {n_skip}  -> {self.lbl_dir}')


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('dir'); ap.add_argument('--subdir', default='images'); ap.add_argument('--classes', nargs='+', default=['pancake', 'candy', 'cake'])
    ap.add_argument('--debug', action='store_true', help='マウス/キーのイベントを端末に表示')
    ap.add_argument('--scale', type=float, default=0.0, help='表示倍率。0 = 画面に合わせて自動（640x360 は 2 倍、スマホ写真は縮小）')
    a = ap.parse_args()
    # classes.txt があればそれを優先（下書きスクリプトが tray などを追加している場合に合わせる）
    cp = os.path.join(a.dir, 'classes.txt')
    if os.path.exists(cp) and '--classes' not in sys.argv:
        a.classes = open(cp).read().split() or a.classes
    Annotator(a.dir, a.subdir, a.classes, a.scale, a.debug).run()


if __name__ == '__main__':
    main()
