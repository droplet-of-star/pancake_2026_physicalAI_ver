#!/usr/bin/env python3
"""1 関節だけを現在角から相対角だけ、ゆっくり（補間して）動かす。他の関節は現在角を保持。

  python3 scripts/move_joint.py --joint wrist_flex --delta 15 --duration 4
  python3 scripts/move_joint.py --joint shoulder_lift --delta -10 --duration 4

安全策: 1 回の delta は ±30° まで。動作中に Ctrl+C すると、その時点の現在角を目標にして止める。
motor bridge（hold）と /so101_follower/joint_states が必要。
"""
import argparse, math, time
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

JOINTS = ['shoulder_pan', 'shoulder_lift', 'elbow_flex', 'wrist_flex', 'wrist_roll', 'gripper']
ap = argparse.ArgumentParser()
ap.add_argument('--joint', required=True, choices=JOINTS)
ap.add_argument('--delta', type=float, required=True, help='相対角 [deg]（±30 まで）')
ap.add_argument('--duration', type=float, default=4.0, help='移動時間 [s]（最低 2 s）')
a = ap.parse_args()
if abs(a.delta) > 30: raise SystemExit('安全のため delta は ±30° まで')
dur = max(2.0, a.duration)

class Mover(Node):
    def __init__(self):
        super().__init__('move_joint'); self.js = None
        self.create_subscription(JointState, '/so101_follower/joint_states', lambda m: setattr(self, 'js', m), 10)
        self.pub = self.create_publisher(JointState, '/so101_follower/joint_commands', 10)
    def cmd(self, pos):
        m = JointState(); m.header.stamp = self.get_clock().now().to_msg(); m.name = list(pos); m.position = [pos[n] for n in m.name]
        self.pub.publish(m)

rclpy.init(); n = Mover(); t0 = time.time()
while rclpy.ok() and n.js is None and time.time() - t0 < 5: rclpy.spin_once(n, timeout_sec=0.2)
if n.js is None: raise SystemExit('joint_states が来ていません（bridge は動いていますか）')
# 動かす関節だけを送る（bridge はメッセージに含まれる関節だけ更新するので、他関節の指令値は変わらない＝垂れの取り込みを防ぐ）
start = {a.joint: dict(zip(n.js.name, n.js.position))[a.joint]}; target = {a.joint: start[a.joint] + math.radians(a.delta)}
print(f'{a.joint}: {math.degrees(start[a.joint]):+.1f}° -> {math.degrees(target[a.joint]):+.1f}°  ({dur:.1f} s)  他関節の指令値は据え置き')
t0 = time.time()
try:
    while rclpy.ok():
        s = min(1.0, (time.time() - t0) / dur); s = 0.5 - 0.5 * math.cos(math.pi * s)   # 滑らかな加減速
        pos = {k: start[k] + (target[k] - start[k]) * s for k in start}
        n.cmd(pos); rclpy.spin_once(n, timeout_sec=0.02)
        if time.time() - t0 >= dur + 0.5: break
    cur = dict(zip(n.js.name, n.js.position))
    print('到達:', f'{a.joint} = {math.degrees(cur[a.joint]):+.1f}°')
except KeyboardInterrupt:
    cur = {a.joint: dict(zip(n.js.name, n.js.position))[a.joint]}; n.cmd(cur); print('\n中断: 現在角で停止')
finally:
    if rclpy.ok():
        n.destroy_node(); rclpy.try_shutdown()
