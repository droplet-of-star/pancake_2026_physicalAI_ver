#!/usr/bin/env python3
"""OAK-D Pro の取付値を、D455 用 35° ブラケットの既知姿勢から計算して camera_mount.yaml に書く。

前提: D455 と同じブラケット。OAK は底面の三脚ねじでブラケットに固定（底面がブラケットに接し、
レンズはグリッパ先端側を向く）。新しい穴は元の D455 の穴から「カメラ前方（内側＝グリッパ側）」へ
d_forward [mm]、横へ d_lateral [mm]（カメラから見て左が正）ずれている。

  python3 scripts/compute_oak_mount.py --d-forward 15 --hole-to-front 11.25 --lens-height 14.75 [--d-lateral 0] [--write]
"""
import argparse, pathlib
import numpy as np, yaml
from scipy.spatial.transform import Rotation as R

# D455 のねじ穴フレーム (gripper_link 系)。2026-10-20 CAD から算出した値。
D455_SCREW_XYZ = np.array([-0.001, 0.037, -0.0212])
D455_SCREW_RPY = np.array([3.1415, 0.9599, -1.5709])

ap = argparse.ArgumentParser()
ap.add_argument('--d-forward', type=float, required=True, help='新しい穴の、元の穴からのカメラ前方向オフセット [mm]（グリッパ側が正）')
ap.add_argument('--d-lateral', type=float, default=0.0, help='横方向オフセット [mm]（カメラから見て左が正）')
ap.add_argument('--hole-to-front', type=float, default=11.25, help='OAK 底面ねじ穴中心から前面（レンズ面）までの距離 [mm]')
ap.add_argument('--lens-height', type=float, default=14.75, help='OAK 底面から RGB レンズ中心までの高さ [mm]')
ap.add_argument('--lens-lateral', type=float, default=0.0, help='ねじ穴中心から RGB レンズまでの横オフセット [mm]（左が正）')
ap.add_argument('--write', action='store_true', help='so101_pick_place/config/camera_mount.yaml に書き込む')
a = ap.parse_args()

Rs = R.from_euler('xyz', D455_SCREW_RPY).as_matrix()      # ねじ穴フレーム軸 (x 前, y 左, z 上) in gripper_link
# 新しい穴 → OAK の RGB レンズ位置（ねじ穴フレーム内）
lens_local = np.array([a.d_forward + a.hole_to_front, a.d_lateral + a.lens_lateral, a.lens_height]) * 1e-3
lens_g = D455_SCREW_XYZ + Rs @ lens_local
rpy = R.from_matrix(Rs).as_euler('xyz')   # 姿勢はブラケットと同じ
print('camera_link (OAK RGB レンズ) in gripper_link:')
print('  xyz =', np.round(lens_g, 4).tolist(), ' rpy =', np.round(rpy, 4).tolist(), ' (deg', np.round(np.degrees(rpy), 1).tolist(), ')')
if a.write:
    p = pathlib.Path(__file__).resolve().parents[1] / 'so101_pick_place/config/camera_mount.yaml'
    m = yaml.safe_load(open(p, encoding='utf-8')) or {}
    m.update({'camera_model': 'oak_d_pro', 'camera_parent': 'gripper_link',
              'camera_xyz': [round(float(v), 4) for v in lens_g], 'camera_rpy': [round(float(v), 4) for v in rpy],
              'measured': True,
              'note': (f'OAK-D Pro, D455 用 35° ブラケット流用。新穴オフセット 前 {a.d_forward} mm / 横 {a.d_lateral} mm、'
                       f'穴→前面 {a.hole_to_front} mm、レンズ高 {a.lens_height} mm')})
    yaml.safe_dump(m, open(p, 'w', encoding='utf-8'), allow_unicode=True, sort_keys=False)
    print('written:', p)
