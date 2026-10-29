#!/usr/bin/env python3
"""motor bridge が動いたまま、現在の姿勢を指定関節の 0 として calibration yaml を更新する。

  python3 scripts/rezero_from_bridge.py --joints shoulder_lift,elbow_flex,wrist_flex
  （トルクを切らずに済む。更新後は bridge と robot_tf を再起動する）
"""
import argparse, math, pathlib, shutil, time
from datetime import datetime
import rclpy, yaml
from rclpy.node import Node
from sensor_msgs.msg import JointState

TICKS = 4096
ap = argparse.ArgumentParser()
ap.add_argument('--joints', default='shoulder_lift,elbow_flex,wrist_flex')
ap.add_argument('--file', default=str(pathlib.Path.home() / '.so101_follower_calibration.yaml'))
ap.add_argument('--samples', type=int, default=20)
a = ap.parse_args()
joints = a.joints.split(',')
path = pathlib.Path(a.file); cal = yaml.safe_load(open(path, encoding='utf-8'))

class Grab(Node):
    def __init__(self):
        super().__init__('rezero'); self.msgs = []
        self.create_subscription(JointState, '/so101_follower/joint_states', lambda m: self.msgs.append(m), 10)
rclpy.init(); n = Grab(); t = time.time()
while rclpy.ok() and len(n.msgs) < a.samples and time.time() - t < 5: rclpy.spin_once(n, timeout_sec=0.2)
rclpy.try_shutdown()
if not n.msgs: raise SystemExit('joint_states が来ていません（bridge が動いていますか）')
avg = {}
for m in n.msgs:
    for name, pos in zip(m.name, m.position): avg.setdefault(name, []).append(pos)
avg = {k: sum(v) / len(v) for k, v in avg.items()}

shutil.copy2(path, path.with_name(path.name + '.' + datetime.now().strftime('%Y%m%d_%H%M%S') + '.bak'))
print('現在角 (deg):', {k: round(math.degrees(v), 1) for k, v in avg.items()})
for j in joints:
    c = cal[j]; home = int(c['homing_offset']); d = int(c.get('direction', 1))
    delta_ticks = d * avg[j] * TICKS / (2 * math.pi)            # 旧 home 基準の現在位置 [ticks]
    new_home = int(round(home + delta_ticks)) % TICKS
    # 可動範囲（旧 home 基準 unwrapped）→ 実エンコーダ値 → 新 home 基準 unwrapped
    def rel(raw):
        return ((raw - new_home + TICKS // 2) % TICKS) - TICKS // 2
    lo_raw, hi_raw = int(c['range_min']) % TICKS, int(c['range_max']) % TICKS
    lo, hi = sorted((new_home + rel(lo_raw), new_home + rel(hi_raw)))
    c['range_min'], c['range_max'], c['homing_offset'] = int(lo), int(hi), int(new_home)
    print(f'  {j:14s} home {home} -> {new_home}  (現在角 {math.degrees(avg[j]):+.1f}° を 0 に)  範囲 {(lo-new_home)*360/TICKS:+.0f}°..{(hi-new_home)*360/TICKS:+.0f}°')
cal['generated'] = datetime.now().isoformat()
cal['method'] = cal.get('method', '') + f' | rezero_from_bridge {",".join(joints)}'
yaml.safe_dump(cal, open(path, 'w', encoding='utf-8'), allow_unicode=True, sort_keys=False)
print('書き込み:', path, '\n次: bridge を Ctrl+C（アームを支える）→ hold で再起動、robot_tf.launch.py も再起動')
