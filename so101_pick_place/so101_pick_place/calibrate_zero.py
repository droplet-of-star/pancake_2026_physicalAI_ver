#!/usr/bin/env python3
"""SO-101 follower の姿勢基準キャリブレーション。

既存の so101_calibrate は「可動端の中点 = 0」とするため、URDF のゼロ姿勢と一致しない。
このスクリプトは
  1. トルクを切り、アームを URDF のゼロ姿勢（docs/images/urdf_zero_pose.png）に手で合わせてもらい、
     そのエンコーダ値を homing_offset として記録する（= URDF と実機の 0 を一致させる）。
  2. 続けて全関節をゆっくり可動端まで動かしてもらい、range_min / range_max を記録する。
  3. ~/.so101_follower_calibration.yaml に書く（既存ファイルは日時付きで退避）。

使い方（motor bridge は止めておく。ポートが競合する）:
  ros2 run so101_pick_place so101_calibrate_zero --ros-args -p port:=/dev/ttyACM0
"""
import pathlib
import select
import shutil
import sys
import threading
import time
from datetime import datetime

import rclpy
import yaml
from rclpy.node import Node
from so101_hw_interface.motors import Motor, MotorNormMode
from so101_hw_interface.motors.feetech.feetech import FeetechMotorsBus
from so101_hw_interface.bus_lock import acquire_bus_lock

JOINTS = {
    'shoulder_pan': 1, 'shoulder_lift': 2, 'elbow_flex': 3,
    'wrist_flex': 4, 'wrist_roll': 5, 'gripper': 6,
}
DEFAULT_PATH = pathlib.Path.home() / '.so101_follower_calibration.yaml'
TICKS = 4096

ZERO_POSE_TEXT = """
=== 基準姿勢（URDF の全関節 0）に合わせてください。水準器（スマホの水準器アプリ可）を使うこと ===
  図: docs/images/urdf_zero_pose.png
  - 土台の旋回 (shoulder_pan): アームが土台の正面を向く（土台の辺と平行に）
  - 上腕 (shoulder_lift): 鉛直。上腕の側面に水準器を当てて 90°
  - 前腕 (elbow_flex): 水平に前方。前腕の上面に水準器を当てて 0°
  - 手首曲げ (wrist_flex): グリッパ本体の上面が水平（前腕の延長で真っ直ぐ前方）
  - 手首ロール (wrist_roll): 可動爪が上・固定爪が下（爪が上下に開閉する向き）。
      このときカメラはグリッパの左側（ロボットの後ろから前を見て左）に、長辺が鉛直の縦向きで付く
  - グリッパ (gripper): 閉じた状態
トルクは切ってあります。姿勢が決まったら手を離さず支えたまま Enter。
"""

class ZeroCalibrator(Node):
    def __init__(self):
        super().__init__('so101_calibrate_zero')
        self.declare_parameter('port', '/dev/ttyACM0')
        self.declare_parameter('outfile', str(DEFAULT_PATH))
        # 指定関節の可動範囲だけ再記録する（homing_offset は既存ファイルを維持）。例: -p range_only_joints:="['shoulder_lift']"
        self.declare_parameter('range_only_joints', [''])
        # true: 基準姿勢（homing_offset）だけ取り直し、可動範囲は既存ファイルの生値を維持する
        self.declare_parameter('zero_only', False)
        self.port = self.get_parameter('port').value
        self.range_only = [j for j in self.get_parameter('range_only_joints').value if j]
        self.zero_only = bool(self.get_parameter('zero_only').value)
        self.outfile = pathlib.Path(self.get_parameter('outfile').value).expanduser()
        motors = {n: Motor(i, 'sts3215', MotorNormMode.DEGREES) for n, i in JOINTS.items()}
        self._bus_lock = acquire_bus_lock(self.port)   # 二重起動防止
        self.bus = FeetechMotorsBus(self.port, motors)
        self.get_logger().info(f'Connecting to {self.port} ...')
        self.bus.connect()
        self.bus.disable_torque()
        self.get_logger().info('Torque disabled. アームを手で支えてください。')

    def read_raw(self):
        return {n: int(v) for n, v in self.bus.sync_read('Present_Position', normalize=False).items()}

    def run(self):
        existing = {}
        if self.outfile.exists():
            with open(self.outfile, 'r', encoding='utf-8') as fp:
                existing = yaml.safe_load(fp) or {}
        if self.range_only:
            missing = [j for j in self.range_only if j not in existing]
            if missing:
                print(f'既存ファイルに {missing} の homing_offset がありません。通常モードで実行してください。'); return
            home = {n: int(existing[n]['homing_offset']) for n in JOINTS if n in existing}
            targets = self.range_only
            print(f'\n=== 可動範囲の再記録: {targets}（基準姿勢は既存の値を維持） ===')
        else:
            print(ZERO_POSE_TEXT)
            input('基準姿勢にしたら Enter > ')
            samples = [self.read_raw() for _ in range(10)]
            home = {n: int(round(sum(s[n] for s in samples) / len(samples))) for n in JOINTS}
            print('homing_offset(raw):', home)
            targets = list(JOINTS)
            if self.zero_only:
                if any(n not in existing for n in JOINTS):
                    print('既存ファイルに可動範囲がありません。zero_only なしで実行してください。'); return
                calib = {}
                for n in JOINTS:
                    c = dict(existing[n])
                    # 旧 home 基準の unwrapped 範囲 → 実エンコーダ値 → 新 home 基準の unwrapped 範囲
                    lo_raw = (int(c['range_min'])) % TICKS; hi_raw = (int(c['range_max'])) % TICKS
                    d = lambda raw: ((raw - home[n] + TICKS // 2) % TICKS) - TICKS // 2
                    c['range_min'], c['range_max'] = home[n] + d(lo_raw), home[n] + d(hi_raw)
                    if c['range_min'] > c['range_max']:
                        c['range_min'], c['range_max'] = c['range_max'], c['range_min']
                    c['homing_offset'] = int(home[n]); calib[n] = c
                print('\n=== 結果（可動範囲は維持、基準姿勢のみ更新） ===')
                for n, c in calib.items():
                    lo = (c['range_min'] - c['homing_offset']) * 360 / TICKS; hi = (c['range_max'] - c['homing_offset']) * 360 / TICKS
                    flag = '' if lo <= 0 <= hi else '  [警告] 基準姿勢が可動範囲の外'
                    print(f'  {n:14s} home {c["homing_offset"]:5d}  = {lo:+6.0f}° .. {hi:+6.0f}°{flag}')
                self._save(calib); return

        print('\n=== 可動範囲の記録 ===\n'
              f'対象関節 {targets} を 1 つずつ、ゆっくり両方の可動端まで動かしてください（ぶつけない範囲で）。\n'
              '記録は自動で続きます。終わったら Enter。\n')
        # 差分空間 (home 基準、±2048 に正規化) で min/max を追う。0/4095 をまたいでも正しく記録できる。
        delta = lambda raw, h: ((int(raw) - int(h) + 2048) % 4096) - 2048
        dmin = {n: 0 for n in JOINTS}; dmax = {n: 0 for n in JOINTS}
        stop = threading.Event()

        def sampler():
            while not stop.is_set():
                try:
                    raw = self.read_raw()
                except Exception:  # noqa: BLE001
                    time.sleep(0.05); continue
                for n, v in raw.items():
                    if n not in home: continue
                    d = delta(v, home[n])
                    dmin[n] = min(dmin[n], d); dmax[n] = max(dmax[n], d)
                time.sleep(0.02)

        th = threading.Thread(target=sampler, daemon=True); th.start()
        t0 = time.time()
        while True:
            if select.select([sys.stdin], [], [], 1.0)[0]:
                sys.stdin.readline(); break
            line = '  '.join(f'{n[:8]}:{dmin[n]*360/TICKS:+4.0f}..{dmax[n]*360/TICKS:+4.0f}°' for n in targets)
            print(f'\r  {time.time()-t0:4.0f}s  {line}', end='', flush=True)
        stop.set(); th.join(timeout=1.0); print()

        calib = {}
        for n in JOINTS:
            if n in targets:
                # unwrapped 表記: home + 差分（0 未満や 4095 超もあり得る。bridge は差分空間で扱う）
                calib[n] = {'range_min': int(home[n] + dmin[n]), 'range_max': int(home[n] + dmax[n]),
                            'homing_offset': int(home[n]),
                            'direction': int(existing.get(n, {}).get('direction', 1))}
            else:
                calib[n] = dict(existing[n])
        problems = []
        for n in targets:
            c = calib[n]; span = (c['range_max'] - c['range_min']) * 360 / TICKS
            if span < 20:
                problems.append(f'{n}: 可動範囲が {span:.0f}° しか記録されていない')
            if span > 350:
                problems.append(f'{n}: 可動範囲が {span:.0f}°（ほぼ全周）。両端で止めずに回し続けていないか確認')
        print('\n=== 結果 ===')
        for n, c in calib.items():
            lo = (c['range_min'] - c['homing_offset']) * 360 / TICKS
            hi = (c['range_max'] - c['homing_offset']) * 360 / TICKS
            mark = '*' if n in targets else ' '
            print(f' {mark}{n:14s} home {c["homing_offset"]:5d}  range {c["range_min"]:5d}..{c["range_max"]:5d}'
                  f'  = {lo:+6.0f}° .. {hi:+6.0f}° (基準姿勢を 0°)')
        for p in problems:
            print('  [警告] ' + p)
        if problems and input('警告があります。それでも保存しますか? [y/N] > ').strip().lower() != 'y':
            print('保存せずに終了します。'); return

        self._save(calib)

    def _save(self, calib):
        if self.outfile.exists():
            bak = self.outfile.with_name(self.outfile.name + '.' + datetime.now().strftime('%Y%m%d_%H%M%S') + '.bak')
            shutil.copy2(self.outfile, bak); print(f'既存ファイルを退避: {bak}')
        self.outfile.parent.mkdir(parents=True, exist_ok=True)
        with open(self.outfile, 'w', encoding='utf-8') as fp:
            yaml.safe_dump({'generated': datetime.now().isoformat(),
                            'method': 'urdf_zero_pose (so101_pick_place calibrate_zero); range は home 基準の unwrapped 値',
                            **calib}, fp, sort_keys=False, allow_unicode=True)
        self.get_logger().info(f'Calibration written to {self.outfile}')
        print('\n次: motor bridge を startup_pose:=hold で起動し、RViz で各関節の動く向きが実機と一致するか確認。\n'
              '逆向きの関節は yaml の direction を -1 にする。')

    def close(self):
        try:
            self.bus.disconnect(disable_torque=True)
        except Exception:  # noqa: BLE001
            pass


def main(args=None):
    rclpy.init(args=args)
    node = ZeroCalibrator()
    try:
        node.run()
    except KeyboardInterrupt:
        print('\n中断しました（保存していません）。')
    finally:
        node.close()
        if rclpy.ok():
            node.destroy_node(); rclpy.shutdown()


if __name__ == '__main__':
    main()
