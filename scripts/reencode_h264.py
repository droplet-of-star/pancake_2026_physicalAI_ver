#!/usr/bin/env python3
"""OpenCV で書いた mp4v の動画を H.264（avc1）に変換する。ffmpeg が無い環境用。
  python3 scripts/reencode_h264.py 入力.mp4 [出力.mp4]
"""
import os, sys, cv2
src = sys.argv[1]; dst = sys.argv[2] if len(sys.argv) > 2 else os.path.splitext(src)[0] + '_h264.mp4'
c = cv2.VideoCapture(src); fps = c.get(cv2.CAP_PROP_FPS) or 15.0; w, h = int(c.get(3)), int(c.get(4))
out = cv2.VideoWriter(dst, cv2.VideoWriter_fourcc(*'avc1'), fps, (w, h)); n = 0
while True:
    ok, f = c.read()
    if not ok: break
    out.write(f); n += 1
out.release(); print(f'{dst}: {n} frames, {n / fps:.1f} s, {os.path.getsize(dst) / 1e6:.1f} MB')
