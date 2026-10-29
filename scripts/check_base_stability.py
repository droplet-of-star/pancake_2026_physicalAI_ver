#!/usr/bin/env python3
"""STEP 4 完了条件の確認: アーム姿勢を変えても静止したお皿の base_link 座標が安定しているか。

使い方（カメラ、robot_tf.launch.py、motor bridge、pick_place.launch.py が動いている状態で）:
    python3 scripts/check_base_stability.py            # Ctrl+C で終了、姿勢ごとの統計を表示
各「姿勢」は joint_states が 1 秒以上静止している区間として自動で区切る。
"""
import math, time
import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped
from sensor_msgs.msg import JointState


class Checker(Node):
    def __init__(self):
        super().__init__('check_base_stability')
        self.create_subscription(PointStamped, '/so101_pick_place/object_point_base', self.on_base, 10)
        self.create_subscription(PointStamped, '/so101_pick_place/object_point_camera', self.on_cam, 10)
        self.create_subscription(JointState, '/so101_follower/joint_states', self.on_js, 10)
        self.q = None; self.q_prev = None; self.last_move = time.time(); self.pose_id = 0
        self.samples = {}   # pose_id -> list of (x,y,z)
        self.cam_last = None
        self.create_timer(2.0, self.report)

    def on_js(self, msg):
        q = np.array(msg.position)
        # 静止時の基準姿勢 q_ref と比較する（ゆっくりした動きでも検出できる）
        if self.q_prev is None:
            self.q_prev = q
        if np.max(np.abs(q - self.q_prev)) > math.radians(1.0):
            if time.time() - self.last_move > 1.0:
                self.pose_id += 1        # 新しい動き始め → 次の静止区間は新しい姿勢
            self.last_move = time.time()
            self.q_prev = q            # 動いている間は基準を追従させる
        self.q = q

    def on_cam(self, msg):
        self.cam_last = (msg.point.x, msg.point.y, msg.point.z)

    def on_base(self, msg):
        if self.q is None or time.time() - self.last_move < 1.0:
            return   # 動作中は捨てる
        self.samples.setdefault(self.pose_id, []).append((msg.point.x, msg.point.y, msg.point.z))

    def report(self):
        if not self.samples:
            self.get_logger().info('waiting for object_point_base + joint_states ...')
            return
        lines = []
        means = []
        for pid, pts in sorted(self.samples.items()):
            a = np.array(pts); m = a.mean(0); s = a.std(0) * 1000
            means.append(m)
            lines.append(f'  pose {pid}: n={len(a):4d}  base xyz=({m[0]:+.3f},{m[1]:+.3f},{m[2]:+.3f}) m  std=({s[0]:.1f},{s[1]:.1f},{s[2]:.1f}) mm')
        if len(means) > 1:
            M = np.array(means); spread = (M.max(0) - M.min(0)) * 1000
            lines.append(f'  >>> 姿勢間のばらつき (max-min): x={spread[0]:.0f} y={spread[1]:.0f} z={spread[2]:.0f} mm')
        q = '' if self.q is None else ' joints(deg)=' + ','.join(f'{math.degrees(v):.0f}' for v in self.q)
        print('\n'.join(['--- base_link での検出位置 ---'] + lines + [f'  camera xyz now={self.cam_last}{q}']), flush=True)


def main():
    rclpy.init()
    n = Checker()
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            n.destroy_node(); rclpy.shutdown()


if __name__ == '__main__':
    main()
