"""Pick & Place 制御ノード（観測 → 把持 → 配置の状態機械、FK/IK、実測補正、安全検査）。

コマンド（std_msgs/String を /so101_pick_place/cmd に publish）:
  observe    : 観測姿勢でトレイを探し（見えなければ肩旋回で探索）、トレイ中心と取っ手の把持点（base_link）を記憶する
  plan/hover : 記憶した中心の approach_offset_z_m 上空へ TCP を移動（plan は IK と経路検査だけ）
  pregrasp / descend / grasp : 取っ手の上空で止まる / 把持姿勢で止まる / 把持して持ち上げる
  lift / place / open / close / home : 単独動作
  pick_place : observe → grasp（失敗なら観測からやり直し）→ place → home
  wait_load  : load_detector が「載っている」と load_confirm_s 続けて判定するまで観測姿勢で待つ
  auto       : wait_load → pick_place。auto_loop は stop まで繰り返す
  stop       : 現在角で停止（動作を中断）
状態は /so101_pick_place/status (String) に出す。dry_run=true のときは関節指令を出さずログだけ。

安全:
  - 全移動は現在角から目標角へ cos 補間、最大速度 max_joint_speed_deg_s、各関節はキャリブ範囲内
  - TCP 目標の z が table_z_m + min_tcp_clearance_m より低ければ拒否
  - 移動中に joint_states が途切れたら停止
  - motor bridge の hold と同じトピック /so101_follower/joint_commands を使う（bridge はメッセージ内の関節だけ更新）
"""
import json
import math
import os
import threading
import time

import numpy as np
import rclpy
import yaml
from geometry_msgs.msg import PointStamped
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import JointState, CameraInfo
from std_msgs.msg import String

from so101_pick_place.kinematics import ARM_JOINTS, SO101Kinematics, limits_from_calibration


class PickPlaceController(Node):
    def __init__(self):
        super().__init__('pick_place_controller')
        p = self.declare_parameter
        p('dry_run', True)
        p('base_frame', 'base_link')
        p('approach_offset_z_m', 0.10)
        p('approach_tilt_deg', 20.0)
        p('approach_tilt_max_deg', 50.0)   # 解けないときはこの角まで傾きを増やして再試行
        p('pregrasp_distance_m', 0.06)     # 接近方向に沿って手前に下がる距離（上空待機点）
        p('ik_pos_tol_m', 0.005)
        p('ik_dir_tol_deg', 10.0)      # 接近方向の傾きの許容誤差
        p('table_z_m', 0.04)
        p('table_z_use_observed', False)     # True: 観測したトレイ中心の z を台面に使う / False: table_z_m（観測は平面の誤りで 1〜2 cm 上ずれすることがある）
        p('table_z_warn_m', 0.015)          # 観測が table_z_m からこれ以上ずれたら警告
        p('min_tcp_clearance_m', 0.05)
        p('tcp_offset_m', 0.09)
        p('tcp_lateral_m', 0.0)
        p('max_joint_speed_deg_s', 15.0)
        p('settle_time_s', 1.5)
        p('observe_time_s', 3.0)
        p('correction_iterations', 3)
        p('correction_gain', 0.8)
        p('observation_pose_deg', [0.0, -30.0, 60.0, 50.0, -88.6])   # pan, lift, elbow, wrist_flex, wrist_roll（params.yaml で上書き）
        p('calib_file', '~/.so101_follower_calibration.yaml')
        # サーボ内部の位置制限（レジスタ Min/Max_Position_Limit、基準姿勢からの角度）。IK の関節範囲に適用。
        # 2026-10-24: 肩 -19° の解を出して内部下限 -11° に当たり、補正が空回りした。
        p('shoulder_lift_min_deg', -10.0)
        p('shoulder_pan_max_deg', 64.0)
        p('grasp_height_m', 0.031)
        p('gripper_close_deg', 1.0)
        p('gripper_open_deg', 35.0)
        p('gripper_time_s', 1.5)
        p('grasp_check_min_angle_deg', 3.0)   # 閉じ角がこれ未満 → 何も挟んでいない
        p('grasp_check_min_load', 100.0)      # 負荷（Present_Load 生値）がこれ未満 → 挟んでいない
        p('grasp_retries', 1)                 # 失敗時のやり直し回数（観測からやり直す）
        p('handle_topic', '/so101_pick_place/handle_point_base')
        p('lift_height_m', 0.06)
        p('place_handle_axis_xy', [0.0, 1.0])
        p('place_tilt_min_deg', 0.0)        # 置くときの接近傾きの最小値（0 = 工具軸を鉛直に。ロールしてもトレイが傾かない）
        p('place_position_xyz', [-0.12, -0.2, 0.04])   # 置き位置（把持点、base）。z は table_height() で上書き
        p('observe_pan_search_deg', [0.0, -30.0, 30.0])   # 観測で見えないときに肩旋回を振る順番 [deg]（- = 左）
        p('observe_state_file', '~/.so101_pick_place_last_pan')   # 前回見つかった旋回角を再起動後も覚えるファイル
        p('observe_hint_time_s', 1.0)        # 見切れヒントだけがこの秒数続いたら、その側へ早めに振る
        p('observe_pan_gain', 1.5)           # 見切れヒントの画像内角度 → 肩旋回の修正量の倍率（カメラは旋回軸から離れているため >1）
        p('observe_pan_max_deg', 60.0)       # 探索で振る肩旋回の最大値 [deg]
        p('observe_max_tries', 5)            # 1 回の観測で試す姿勢の最大数
        p('table_z_tolerance_m', 0.05)       # 観測した台面高さが table_z_m からこれ以上ずれていたら誤検出（床など）として捨てる
        p('handle_dist_min_m', 0.05)         # 計測した取っ手点とトレイ中心の距離の妥当範囲。外れたら向きも信用せず観測し直す
        p('handle_dist_max_m', 0.16)
        p('handle_grasp_radius_m', 0.108)    # 把持点 = トレイ中心 + この距離 × 計測した取っ手の向き（CAD: 板の中央 約 108 mm）。0 なら計測点をそのまま使う
        p('load_topic', '/so101_pick_place/tray_load')   # load_detector の判定（JSON 文字列）
        p('load_confirm_s', 2.0)             # 「載っている」がこの秒数続いたら開始
        p('load_wait_timeout_s', 0.0)        # 待ちの上限 [s]（0 = stop まで）
        p('load_max_age_s', 1.5)             # これより古い判定は無効
        p('require_load_before_grasp', True)  # 把持の直前にも載せ物検出が「載っている」でなければ掴み始めない
        p('post_grasp_shift_xy', [0.0, -0.02])   # 把持後、持ち上げる前に水平移動する量 [m]（base: +x 左, -y 奥）
        p('joint_commands_topic', '/so101_follower/joint_commands')
        p('joint_states_topic', '/so101_follower/joint_states')
        p('object_topic', '/so101_pick_place/object_point_base_filtered')
        g = lambda n: self.get_parameter(n).value
        self.dry_run = bool(g('dry_run'))
        self.approach = float(g('approach_offset_z_m')); self.table_z = float(g('table_z_m'))
        self.tilt = math.radians(float(g('approach_tilt_deg'))); self.tilt_max = math.radians(float(g('approach_tilt_max_deg'))); self.pre_dist = float(g('pregrasp_distance_m'))
        self.grasp_h = float(g('grasp_height_m')); self.lift_h = float(g('lift_height_m'))
        self.g_chk_ang = math.radians(float(g('grasp_check_min_angle_deg'))); self.g_chk_load = float(g('grasp_check_min_load')); self.g_retries = int(g('grasp_retries'))
        self.g_open = math.radians(float(g('gripper_open_deg'))); self.g_close = math.radians(float(g('gripper_close_deg'))); self.g_time = float(g('gripper_time_s'))
        self.place_xyz = np.array(list(g('place_position_xyz')), float); self.place_axis = np.array(list(g('place_handle_axis_xy')) + [0.0], float)
        self.place_tilt_min = math.radians(float(g('place_tilt_min_deg')))
        self.grasped_axis_tool = None   # 把持した瞬間の取っ手の向き（gripper_link 系）。置くときの向き合わせに使う
        self.pan_search = [float(v) for v in g('observe_pan_search_deg')]; self.last_pan_offset = self.pan_search[0] if self.pan_search else 0.0
        self.state_file = os.path.expanduser(str(g('observe_state_file'))); self.hint_time = float(g('observe_hint_time_s'))
        self.pan_gain = float(g('observe_pan_gain')); self.pan_max = float(g('observe_pan_max_deg')); self.obs_tries = int(g('observe_max_tries'))
        self.table_tol = float(g('table_z_tolerance_m')); self.cam_fx = None; self.cam_w = None
        self.create_subscription(CameraInfo, '/camera/camera/color/camera_info', self.on_cam_info, 10)
        self.post_shift = np.array(list(g('post_grasp_shift_xy')) + [0.0], float)
        self.table_use_obs = bool(g('table_z_use_observed')); self.table_warn = float(g('table_z_warn_m'))
        self.hd_min = float(g('handle_dist_min_m')); self.hd_max = float(g('handle_dist_max_m')); self.handle_radius = float(g('handle_grasp_radius_m'))
        self.require_load = bool(g('require_load_before_grasp'))
        self.load_confirm = float(g('load_confirm_s')); self.load_timeout = float(g('load_wait_timeout_s')); self.load_max_age = float(g('load_max_age_s'))
        self.load_state = None; self.load_time = 0.0
        self.create_subscription(String, str(g('load_topic')), self.on_load, 10)
        try:
            v = float(open(self.state_file).read().strip())
            if abs(v) <= 90.0:
                self.last_pan_offset = v
        except (OSError, ValueError):
            pass
        self.partial_hist = []
        self.create_subscription(PointStamped, '/so101_pick_place/object_partial_pixel', self.on_partial, 10)
        self.ik_pos_tol = float(g('ik_pos_tol_m')); self.ik_dir_tol = float(g('ik_dir_tol_deg'))
        self.clearance = float(g('min_tcp_clearance_m')); self.speed = math.radians(float(g('max_joint_speed_deg_s')))
        self.settle = float(g('settle_time_s')); self.observe_time = float(g('observe_time_s'))
        self.n_corr = int(g('correction_iterations')); self.k_corr = float(g('correction_gain'))
        obs = list(g('observation_pose_deg'))
        self.obs_pose = dict(zip(ARM_JOINTS, [math.radians(v) for v in obs]))
        self.calib = yaml.safe_load(open(os.path.expanduser(str(g('calib_file')))))

        self.kin = None
        self.urdf_sub = self.create_subscription(
            String, '/robot_description', self.on_urdf,
            QoSProfile(depth=1, reliability=QoSReliabilityPolicy.RELIABLE, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL))
        self.js = None; self.js_time = 0.0; self.js_effort = {}
        self.create_subscription(JointState, str(g('joint_states_topic')), self.on_js, 10)
        self.obj = None; self.obj_time = 0.0; self.obj_hist = []
        self.handle_pt = None; self.handle_hist = []
        self.create_subscription(PointStamped, str(g('handle_topic')), self.on_handle, 10)
        self.remembered_handle = None
        self.create_subscription(PointStamped, str(g('object_topic')), self.on_obj, 10)
        self.create_subscription(String, '/so101_pick_place/cmd', self.on_cmd, 10)
        self.cmd_pub = self.create_publisher(JointState, str(g('joint_commands_topic')), 10)
        self.status_pub = self.create_publisher(String, '/so101_pick_place/status', 10)
        self.remembered = None       # 記憶したお皿位置 (base)
        self.busy = False; self.abort = False
        self.get_logger().info(f'pick_place_controller: dry_run={self.dry_run} approach={self.approach} m '
                               f'obs_pose(deg)={obs}  waiting for /robot_description ...')

    # ------------------------------------------------------------------ callbacks
    def on_urdf(self, msg: String):
        if self.kin is None:
            self.kin = SO101Kinematics(msg.data, tcp_offset=float(self.get_parameter('tcp_offset_m').value),
                                       tcp_lateral=float(self.get_parameter('tcp_lateral_m').value),
                                       joint_limits_rad=limits_from_calibration(self.calib))
            lim = self.kin.limits
            lo, hi = lim['shoulder_lift']; lim['shoulder_lift'] = (max(lo, math.radians(float(self.get_parameter('shoulder_lift_min_deg').value))), hi)
            lo, hi = lim['shoulder_pan']; lim['shoulder_pan'] = (lo, min(hi, math.radians(float(self.get_parameter('shoulder_pan_max_deg').value))))
            self.get_logger().info('kinematics ready (URDF from /robot_description); limits(deg): '
                                   + ', '.join(f'{n}=[{math.degrees(a):.0f},{math.degrees(b):.0f}]' for n, (a, b) in lim.items()))

    def on_cam_info(self, msg: CameraInfo):
        if msg.k[0] > 0:
            self.cam_fx = float(msg.k[0]); self.cam_w = int(msg.width)

    def on_load(self, msg: String):
        try:
            self.load_state = json.loads(msg.data); self.load_time = time.time()
        except ValueError:
            pass

    def on_partial(self, msg: PointStamped):
        self.partial_hist.append((time.time(), float(msg.point.z)))
        self.partial_hist = self.partial_hist[-60:]

    def on_js(self, msg: JointState):
        self.js = dict(zip(msg.name, msg.position)); self.js_time = time.time()
        self.js_effort = dict(zip(msg.name, msg.effort)) if len(msg.effort) == len(msg.name) else {}

    def on_obj(self, msg: PointStamped):
        self.obj = np.array([msg.point.x, msg.point.y, msg.point.z]); self.obj_time = time.time()
        self.obj_hist.append((self.obj_time, self.obj.copy()))
        self.obj_hist = [h for h in self.obj_hist if self.obj_time - h[0] < 10.0]

    def on_handle(self, msg: PointStamped):
        self.handle_pt = np.array([msg.point.x, msg.point.y, msg.point.z]); t = time.time()
        self.handle_hist.append((t, self.handle_pt.copy())); self.handle_hist = [h for h in self.handle_hist if t - h[0] < 10.0]

    def radial_approach(self, target, tilt=None):
        """目標の真上から、目標→ロボット中心の水平方向へ tilt だけ傾けた接近方向。"""
        t = self.tilt if tilt is None else tilt
        # 工具軸は「下向き + 土台→目標の水平方向」に傾ける（腕を前へ伸ばして上から掴む自然な姿勢）。
        # 逆向き（工具が土台側を向く）は手首曲げ限界に当たり、ほとんど解けない（2026-10-24 到達性マップ）。
        hx, hy = float(target[0]), float(target[1]); nrm = math.hypot(hx, hy)
        if nrm < 1e-6:
            return (0.0, 0.0, -1.0)
        return (hx / nrm * math.sin(t), hy / nrm * math.sin(t), -math.cos(t))

    def ik_any_tilt(self, tcp_target, q0, fixed_roll, tilt_min=None):
        """傾き tilt..tilt_max を小さい順に試し、最初に解けた (q, pe, de, tilt) を返す。"""
        tol = dict(pos_tol=self.ik_pos_tol, dir_tol_deg=self.ik_dir_tol)
        t_lo = self.tilt if tilt_min is None else tilt_min
        tilts = list(np.arange(t_lo, self.tilt_max + 1e-6, math.radians(5.0)))
        best = None
        for t in tilts:
            ad = self.radial_approach(tcp_target, t)
            for seed in (q0, self.obs_pose):
                q, pe, de, ok = self.kin.ik(tcp_target, approach_dir=ad, q0=seed, fixed={'wrist_roll': fixed_roll}, **tol)
                if ok:
                    return q, pe, de, t
                if best is None or pe + 0.001 * de < best[1] + 0.001 * best[2]:
                    best = (q, pe, de, t)
        return best[0], best[1], best[2], None

    def status(self, text: str):
        self.get_logger().info(text); self.status_pub.publish(String(data=text))

    def on_cmd(self, msg: String):
        cmd = msg.data.strip().lower()
        if cmd == 'stop':
            self.abort = True; self.status('stop requested'); return
        if self.busy:
            self.status(f'busy; ignoring "{cmd}"'); return
        if self.kin is None or self.js is None:
            self.status('not ready (URDF or joint_states missing)'); return
        if cmd == 'plan':
            self.do_hover(plan_only=True); return
        if cmd not in ('observe', 'hover', 'home', 'open', 'close', 'pregrasp', 'descend', 'lift', 'grasp', 'place', 'pick_place',
                       'wait_load', 'auto', 'auto_loop'):
            self.status(f'unknown command "{cmd}" (observe|plan|hover|grasp|place|pick_place|wait_load|auto|auto_loop|open|close|home|stop)'); return
        # 動作はワーカースレッドで実行（コールバック内でブロックすると joint_states の受信が止まるため）
        self.busy = True; self.abort = False
        threading.Thread(target=self._run, args=(cmd,), daemon=True).start()

    def _run(self, cmd: str):
        try:
            if cmd == 'observe':
                self.do_observe()
            elif cmd == 'hover':
                self.do_hover()
            elif cmd == 'home':
                self.safe_move(self.obs_pose, 'home (observation pose)', correct=False)
            elif cmd == 'open':
                self.gripper(self.g_open, 'open')
            elif cmd == 'close':
                self.gripper(self.g_close, 'close')
            elif cmd == 'pregrasp':
                self.do_grasp(stop_at_pregrasp=True)
            elif cmd == 'descend':
                self.do_grasp(stop_at_descend=True)
            elif cmd == 'lift':
                self.do_lift()
            elif cmd == 'grasp':
                self.do_grasp()
            elif cmd == 'place':
                self.do_place()
            elif cmd == 'pick_place':
                self.do_pick_place()
            elif cmd == 'wait_load':
                self.do_wait_load()
            elif cmd == 'auto':
                if self.do_wait_load():
                    self.do_pick_place()
            elif cmd == 'auto_loop':
                n = 0
                while not self.abort:
                    if not self.do_wait_load():
                        break
                    n += 1; self.status(f'auto_loop: cycle {n}')
                    self.do_pick_place()
                self.status(f'auto_loop: stopped after {n} cycle(s)')
        except Exception as exc:  # noqa: BLE001
            self.status(f'ERROR: {exc}')
        finally:
            self.busy = False

    # ------------------------------------------------------------------ motion
    def send(self, q: dict):
        m = JointState(); m.header.stamp = self.get_clock().now().to_msg()
        m.name = list(q); m.position = [float(q[n]) for n in m.name]
        if not self.dry_run:
            self.cmd_pub.publish(m)

    def check_limits(self, q: dict):
        for n, v in q.items():
            lo, hi = self.kin.limits.get(n, (-math.pi, math.pi))
            if not (lo - 1e-6 <= v <= hi + 1e-6):
                raise RuntimeError(f'{n}={math.degrees(v):.1f}deg outside limits [{math.degrees(lo):.0f},{math.degrees(hi):.0f}]')

    def move_to(self, target: dict, label: str):
        """現在角から target へ cos 補間で移動。戻り値: 到達後の実測角。"""
        self.check_limits(target)
        start = {n: self.js[n] for n in target}
        dmax = max(abs(target[n] - start[n]) for n in target)
        dur = max(1.5, dmax / self.speed)
        self.status(f'{label}: moving {math.degrees(dmax):.1f}deg max over {dur:.1f}s'
                    + (' [dry_run: no command sent]' if self.dry_run else ''))
        t0 = time.time()
        while rclpy.ok():
            if self.abort:
                self.send({n: self.js[n] for n in target}); self.status('aborted: holding current pose'); return None
            if time.time() - self.js_time > 0.5:
                self.send({n: self.js[n] for n in target}); raise RuntimeError('joint_states timeout during motion')
            s = min(1.0, (time.time() - t0) / dur); s = 0.5 - 0.5 * math.cos(math.pi * s)
            self.send({n: start[n] + (target[n] - start[n]) * s for n in target})
            time.sleep(0.02)
            if time.time() - t0 >= dur:
                break
        t1 = time.time()
        while rclpy.ok() and time.time() - t1 < self.settle:
            time.sleep(0.05)
        err = {n: math.degrees(self.js[n] - target[n]) for n in target}
        self.status(f'{label}: done. error(deg) ' + ', '.join(f'{n}={e:+.1f}' for n, e in err.items()))
        return dict(self.js)

    def move_with_correction(self, target: dict, label: str):
        """目標角へ移動後、実測との差を指令に上乗せして繰り返す（サーボの重力偏差を吸収）。"""
        cmd = dict(target); prev_reached = None
        for it in range(self.n_corr + 1):
            reached = self.move_to(cmd, f'{label} [{it}]')
            if reached is None:
                return None
            err = {n: reached[n] - target[n] for n in target}
            if max(abs(e) for e in err.values()) < math.radians(1.0):
                break
            if prev_reached is not None:
                # 指令を変えたのに実測がほとんど動かない関節 → サーボ限界/内部制限/機械的干渉。補正を打ち切る
                stuck = [n for n in target if abs(reached[n] - prev_reached[n]) < math.radians(0.5) and abs(err[n]) > math.radians(2.0)]
                if stuck:
                    self.status(f'{label}: correction stopped, joints not responding: {stuck} (servo limit or obstruction?)')
                    break
            prev_reached = dict(reached)
            if it < self.n_corr:
                cmd = {n: cmd[n] - self.k_corr * err[n] for n in target}
                try:
                    self.check_limits(cmd)
                except RuntimeError:
                    self.status('correction would exceed joint limits; stopping correction'); break
        return reached

    # ------------------------------------------------------------------ behaviours
    def table_height(self):
        """把持・配置に使う台面の高さ。既定は固定値 table_z_m。観測値が大きくずれていれば警告（平面フィットの誤りの可能性）。"""
        if self.remembered is not None and self.table_use_obs:
            obs = float(self.remembered[2])
            if abs(obs - self.table_z) <= self.table_warn:
                return obs
            # 大きく外れていれば平面フィットの誤りの可能性が高いので固定値に戻す
            self.status(f'table height: observed {obs:+.3f} m differs from table_z_m {self.table_z:+.3f} by {abs(obs - self.table_z)*100:.1f} cm (plane fit?); using table_z_m')
        return self.table_z

    def do_pick_place(self):
        """観測 → 把持（やり直しあり）→ 配置 → 観測姿勢。成功なら True。"""
        ok = False
        for attempt in range(self.g_retries + 1):
            if self.abort:
                break
            if self.do_observe() and self.do_grasp():
                ok = True; break
            self.status(f'pick_place: grasp attempt {attempt + 1} failed')
        if ok and self.do_place():
            self.safe_move(self.obs_pose, 'home', correct=False)
            self.status('pick_place: SUCCESS'); return True
        self.safe_move(self.obs_pose, 'home', correct=False)
        self.status('pick_place: FAILED'); return False

    def do_wait_load(self):
        """観測姿勢でトレイを見つけ、load_detector が「載っている」と load_confirm_s 続けて判定するまで待つ。
        トレイが見つからなければ数秒おきに探し直す。stop で中断、load_wait_timeout_s で打ち切り。"""
        t_start = time.time(); t_loaded = None; last_msg = 0.0; next_observe = 0.0
        while rclpy.ok() and not self.abort:
            now = time.time()
            if now >= next_observe:
                # トレイを見つけて観測姿勢に留まる（見つからなければ 5 s 後に再探索）
                if self.do_observe():
                    next_observe = float('inf')
                else:
                    next_observe = now + 5.0
                    if self.load_timeout and now - t_start > self.load_timeout:
                        self.status('wait_load: timeout (tray not found)'); return False
                    continue
            st = self.load_state; fresh = st is not None and now - self.load_time <= self.load_max_age
            loaded = bool(fresh and st.get('loaded'))
            items = [it for it in (st or {}).get('items', []) if it.get('inside')] if fresh else []
            desc = ', '.join(f"{it['label']} {it['conf']:.2f}" for it in items) or ('nothing on tray' if fresh and st.get('tray_seen') else
                                                                                   'no fresh detector output' if not fresh else 'tray not seen by detector')
            if loaded:
                if t_loaded is None:
                    t_loaded = now; self.status(f'wait_load: object detected on tray ({desc}); confirming for {self.load_confirm:.1f} s')
                elif now - t_loaded >= self.load_confirm:
                    self.status(f'wait_load: LOADED ({desc}) -> start'); return True
            else:
                if t_loaded is not None:
                    self.status(f'wait_load: detection lost ({desc}); keep waiting')
                t_loaded = None
            if now - last_msg > 5.0:
                last_msg = now; self.status(f'wait_load: waiting for an object on the tray ... ({desc})')
            if self.load_timeout and now - t_start > self.load_timeout:
                self.status('wait_load: timeout'); return False
            time.sleep(0.1)
        self.status('wait_load: aborted'); return False

    def observe_pose(self, pan_offset_deg):
        """観測姿勢の肩旋回だけを pan_offset_deg ずらした姿勢（関節範囲内にクリップ）。"""
        q = dict(self.obs_pose)
        lo, hi = self.kin.limits['shoulder_pan']
        q['shoulder_pan'] = float(np.clip(self.obs_pose['shoulder_pan'] + math.radians(pan_offset_deg), lo, hi))
        return q

    def do_observe(self):
        """観測姿勢でトレイを探す。見えなければ見切れヒント（画面端の赤の位置）から肩旋回の修正量を計算して振る。
        ヒントが無ければ observe_pan_search_deg の順に振る。台面高さが想定から外れた検出（床の赤い物など）は捨てる。"""
        cur = float(self.last_pan_offset); tried = []
        fallback = [o for o in self.pan_search if abs(o - cur) > 1e-6]
        pts = np.empty((0, 3)); t0 = time.time(); hint = None
        for _ in range(self.obs_tries):
            tried.append(cur)
            label = 'observe: go to observation pose' + (f' (pan {cur:+.0f}deg)' if abs(cur) > 0.5 else '')
            if self.safe_move(self.observe_pose(cur), label, correct=False) is None:
                return False
            self.obj_hist = []; self.handle_hist = []; self.partial_hist = []
            t0 = time.time(); hint = None
            while rclpy.ok() and time.time() - t0 < self.observe_time:
                time.sleep(0.05)
                if time.time() - t0 >= self.hint_time and not any(h[0] >= t0 for h in self.obj_hist):
                    hz = [z for (ts, z) in self.partial_hist if ts >= t0]
                    if len(hz) >= 5:
                        hint = float(np.median(hz)); break
            pts = np.array([h[1] for h in self.obj_hist if h[0] >= t0])
            if len(pts) >= 10:
                z_med = float(np.median(pts[:, 2]))
                if abs(z_med - self.table_z) <= self.table_tol:
                    self.last_pan_offset = cur
                    try:
                        open(self.state_file, 'w').write(f'{cur:.1f}')
                    except OSError:
                        pass
                    break
                self.status(f'observe: red object at pan {cur:+.0f}deg has height {z_med:+.3f} m, not the table ({self.table_z:+.3f}±{self.table_tol:.2f}); ignored')
                pts = np.empty((0, 3)); hint = None
            # 次の旋回角
            nxt = None
            if hint is not None and self.cam_fx:
                ang = math.degrees(math.atan((hint * self.cam_w / 2.0) / self.cam_fx))   # 画像内の水平角（+ = 画像右 = ロボットの左）
                step = -self.pan_gain * ang
                if abs(step) < 3.0:
                    step = math.copysign(3.0, step if step else -1.0)
                nxt = float(np.clip(cur + step, -self.pan_max, self.pan_max))
                if any(abs(nxt - t) < 2.0 for t in tried):     # 既に見た角度に戻るなら、同じ向きへさらに 5° 進める
                    nxt = float(np.clip(nxt + math.copysign(5.0, step), -self.pan_max, self.pan_max))
                    if any(abs(nxt - t) < 2.0 for t in tried):
                        nxt = None
                where = 'right' if hint > 0 else 'left'
                self.status(f'observe: object not seen at pan {cur:+.0f}deg (partly visible at image {where}, {ang:+.0f}deg)'
                            + (f'; next pan {nxt:+.0f}deg' if nxt is not None else ''))
            else:
                self.status(f'observe: object not seen at pan {cur:+.0f}deg ({len(pts)} samples)')
            if nxt is None:
                while fallback and any(abs(fallback[0] - t) < 2.0 for t in tried):
                    fallback.pop(0)
                if not fallback:
                    break
                nxt = fallback.pop(0)
            cur = nxt
        if len(pts) < 10:
            self.status(f'observe: object not seen enough ({len(pts)} samples) at pan offsets {[round(t) for t in tried]}. Place the plate in view.'); return False
        self.remembered = np.median(pts, axis=0)
        sd = pts.std(axis=0) * 1000
        self.status(f'observe: plate at base ({self.remembered[0]:+.3f}, {self.remembered[1]:+.3f}, {self.remembered[2]:+.3f}) m '
                    f'from {len(pts)} samples, std(mm)=({sd[0]:.0f},{sd[1]:.0f},{sd[2]:.0f})')
        hp = np.array([h[1] for h in self.handle_hist if h[0] >= t0])
        if len(hp) >= 10:
            self.remembered_handle = np.median(hp, axis=0)
            ax = self.remembered_handle - self.remembered; ang = math.degrees(math.atan2(ax[1], ax[0]))
            dist = float(math.hypot(ax[0], ax[1]))
            if not (self.hd_min <= dist <= self.hd_max):
                # 取っ手がトレイの内側/遠すぎる所に出た = 円フィットか取っ手検出の誤り（載せ物で縁が隠れたときなど）
                self.status(f'observe: handle point {dist*100:.1f} cm from plate center is implausible (expect {self.hd_min*100:.0f}-{self.hd_max*100:.0f} cm); treating as not detected')
                self.remembered_handle = None
                return False
            if self.handle_radius > 0:
                # 距離は CAD 値で置き換え、向きだけ計測値を使う（画素→3D の高さの違いによる距離誤差を避ける）
                unit = np.array([ax[0], ax[1], 0.0]) / max(1e-9, dist)
                corrected = self.remembered + unit * self.handle_radius; corrected[2] = self.remembered_handle[2]
                self.status(f'observe: handle measured {dist*100:.1f} cm from center -> grasp point set at {self.handle_radius*100:.1f} cm along the same direction')
                self.remembered_handle = corrected
            self.status(f'observe: handle grasp point at base ({self.remembered_handle[0]:+.3f}, {self.remembered_handle[1]:+.3f}, {self.remembered_handle[2]:+.3f}) m, '
                        f'handle axis angle {ang:+.0f}deg (from +X), {len(hp)} samples')
        else:
            self.remembered_handle = None
            self.status('observe: handle not detected (grasp needs it)')
        return True

    def do_hover(self, plan_only=False):
        if self.remembered is None:
            self.status('hover: no remembered plate position. Run "observe" first.'); return
        tgt = self.remembered + np.array([0.0, 0.0, self.approach])
        if tgt[2] < self.table_z + self.clearance:
            self.status(f'hover: target z={tgt[2]:.3f} too low; refused'); return
        q0 = {n: self.js[n] for n in ARM_JOINTS}
        q, pe, de, tilt = self.ik_any_tilt(tgt, q0, q0['wrist_roll']); ok = tilt is not None
        self.status(f'hover: IK target TCP ({tgt[0]:+.3f},{tgt[1]:+.3f},{tgt[2]:+.3f}) -> pos_err={pe*1000:.1f}mm dir_err={de:.1f}deg ok={ok} '
                    f'q(deg)=' + ', '.join(f'{n}={math.degrees(v):+.0f}' for n, v in q.items()))
        if not ok:
            self.status('hover: IK failed (unreachable). Not moving.'); return
        tcp, _ = self.kin.tcp(q)
        if tcp[2] < self.table_z + self.clearance:
            self.status('hover: solution TCP too low; refused'); return
        mins = self.kin.path_min_heights(q0, q)
        self.status('hover: path min heights (m) ' + ', '.join(f'{k}={v:.3f}' for k, v in mins.items()) + f' (limit {self.table_z + self.clearance:.3f})')
        if min(mins.values()) < self.table_z + self.clearance:
            self.status('hover: path would come too close to the table; refused'); return
        if plan_only:
            self.status('plan: OK (not moving). send "hover" to execute'); return
        reached = self.move_with_correction(q, 'hover')
        if reached is None:
            return
        tcp_r, a = self.kin.tcp({n: reached[n] for n in ARM_JOINTS})
        self.status(f'hover: reached. TCP(actual by FK) ({tcp_r[0]:+.3f},{tcp_r[1]:+.3f},{tcp_r[2]:+.3f}) m, '
                    f'offset from target ({(tcp_r-tgt)[0]*1000:+.0f},{(tcp_r-tgt)[1]*1000:+.0f},{(tcp_r-tgt)[2]*1000:+.0f}) mm, approach dir {a.round(2)}')

    # ------------------------------------------------------------------ gripper / grasp / place
    def gripper(self, angle: float, label: str):
        lo, hi = self.kin.limits.get('gripper', (-math.pi, math.pi))
        angle = max(lo, min(hi, angle))
        self.status(f'gripper {label}: -> {math.degrees(angle):.0f}deg' + (' [dry_run]' if self.dry_run else ''))
        start = self.js['gripper']; t0 = time.time(); dur = max(0.5, self.g_time)
        while time.time() - t0 < dur:
            if self.abort:
                return False
            sgm = min(1.0, (time.time() - t0) / dur); sgm = 0.5 - 0.5 * math.cos(math.pi * sgm)
            self.send({'gripper': start + (angle - start) * sgm}); time.sleep(0.02)
        time.sleep(0.5)
        self.status(f'gripper {label}: actual {math.degrees(self.js["gripper"]):.0f}deg')
        return True

    def solve_pose(self, tcp_target, handle_axis, label, q_seed=None, tool_axis=None, tilt_min=None):
        """TCP 目標 + 放射状接近（傾き自動）+ ロールの関節解。失敗なら None。戻り値 (q, tilt)。
        tool_axis（gripper_link 系）を与えると、そのベクトルが base で handle_axis の向き（符号込み）になるロールを選ぶ。
        与えなければ、指の閉じ方向が handle_axis と直交するロール（180° の曖昧さは現在値に近い方）。"""
        q0 = q_seed or {n: self.js[n] for n in ARM_JOINTS}
        q, pe, de, tilt = self.ik_any_tilt(tcp_target, q0, q0['wrist_roll'], tilt_min=tilt_min)
        if tilt is None:
            self.status(f'{label}: IK failed (best pos_err={pe*1000:.1f}mm dir_err={de:.1f}deg)'); return None
        if tool_axis is not None:
            roll, ang_err = self.kin.solve_roll_signed(q, tool_axis, handle_axis, prefer=q0['wrist_roll'])
            note = f'handle dir {ang_err:.0f}deg off target'
        else:
            roll, perp_err = self.kin.solve_roll(q, handle_axis, prefer=q0['wrist_roll'])
            note = f'closing-dir {90 - perp_err:.0f}deg off perpendicular'
        q['wrist_roll'] = roll
        tcp, a = self.kin.tcp(q)
        self.status(f'{label}: tilt {math.degrees(tilt):.0f}deg, q(deg)=' + ', '.join(f'{n}={math.degrees(v):+.0f}' for n, v in q.items())
                    + f' | TCP err {np.linalg.norm(tcp - tcp_target)*1000:.1f}mm, {note}')
        return q, tilt

    def ensure_clear(self, label):
        """いま TCP が台面近く（把持姿勢など）なら、同じ姿勢のまま真上に lift_height だけ上げてから大きな移動をする。"""
        q0 = {n: self.js[n] for n in ARM_JOINTS}
        tcp, _ = self.kin.tcp(q0)
        if tcp[2] >= self.table_z + self.clearance:
            return True
        up = tcp + np.array([0.0, 0.0, self.lift_h])
        q, pe, de, tilt = self.ik_any_tilt(up, q0, q0['wrist_roll'])
        if tilt is None:
            self.status(f'{label}: cannot lift clear of table (IK)'); return False
        q['wrist_roll'] = q0['wrist_roll']
        self.status(f'{label}: TCP low ({tcp[2]:.3f} m); lifting {self.lift_h*100:.0f} cm first')
        return self.move_to(q, f'{label}/lift-clear') is not None

    def safe_move(self, q, label, correct=True):
        if not self.ensure_clear(label):
            return None
        q0 = {n: self.js[n] for n in ARM_JOINTS}
        mins = self.kin.path_min_heights(q0, q)
        if min(mins.values()) < self.table_z + self.clearance * 0.5:
            self.status(f'{label}: path too low ' + str({k: round(v, 3) for k, v in mins.items()}) + '; refused'); return None
        return self.move_with_correction(q, label) if correct else self.move_to(q, label)

    def tray_loaded_now(self):
        """載せ物検出の最新判定（新鮮なもの）が「載っている」なら True。"""
        st = self.load_state
        return bool(st is not None and time.time() - self.load_time <= self.load_max_age and st.get('loaded'))

    def do_grasp(self, stop_at_pregrasp=False, stop_at_descend=False):
        if self.require_load and not self.tray_loaded_now():
            st = self.load_state; fresh = st is not None and time.time() - self.load_time <= self.load_max_age
            self.status('grasp: nothing detected on the tray' + ('' if fresh else ' (no fresh load_detector output)') + '; not grasping')
            return False
        if self.remembered_handle is None or self.remembered is None:
            self.status('grasp: no remembered handle. Run observe first.'); return False
        handle_axis = self.remembered_handle - self.remembered; handle_axis[2] = 0.0
        # 台面の高さは観測値（トレイ中心の平面交点 z）を使う。設定値 table_z_m は検査用の下限にのみ使う
        table_now = self.table_height()
        gp = self.remembered_handle.copy(); gp[2] = table_now + self.grasp_h
        self.status(f'grasp: table height {table_now:+.3f} m -> grasp TCP z {gp[2]:+.3f} m')
        if not self.gripper(self.g_open, 'open'):
            return False
        sol = self.solve_pose(gp, handle_axis, 'grasp/target')
        if sol is None:
            return False
        q_g, tilt = sol
        pre = gp - np.array(self.radial_approach(gp, tilt)) * self.pre_dist     # 接近方向に沿って手前
        sol = self.solve_pose(pre, handle_axis, 'grasp/pregrasp', q_seed=q_g)
        if sol is None:
            return False
        q_pre, _ = sol
        if self.safe_move(q_pre, 'grasp/pregrasp') is None:
            return False
        if stop_at_pregrasp:
            tcp, _ = self.kin.tcp({n: self.js[n] for n in ARM_JOINTS})
            self.status(f'pregrasp: holding above handle. TCP(FK) ({tcp[0]:+.3f},{tcp[1]:+.3f},{tcp[2]:+.3f}) m; target grasp point ({gp[0]:+.3f},{gp[1]:+.3f},{gp[2]:+.3f}). Send "grasp" to continue.')
            return True
        q_g['wrist_roll'] = q_pre['wrist_roll']
        if self.move_with_correction(q_g, 'grasp/descend') is None:
            return False
        if stop_at_descend:
            tcp, a = self.kin.tcp({n: self.js[n] for n in ARM_JOINTS})
            self.status(f'descend: holding at grasp pose, gripper open. TCP(FK) ({tcp[0]:+.3f},{tcp[1]:+.3f},{tcp[2]:+.3f}) m vs target ({gp[0]:+.3f},{gp[1]:+.3f},{gp[2]:+.3f}); '
                        f'closing dir {self.kin.closing_dir({n: self.js[n] for n in ARM_JOINTS}).round(2)}, handle axis {(handle_axis/np.linalg.norm(handle_axis)).round(2)}')
            return True
        if not self.gripper(self.g_close, 'close'):
            return False
        ang = self.js['gripper']; load = float(self.js_effort.get('gripper', 0.0)); load = load - 1024 if load >= 1024 else load
        held = ang >= self.g_chk_ang and abs(load) >= self.g_chk_load
        self.status(f'grasp check: gripper {math.degrees(ang):.1f}deg load {abs(load):.0f} -> {"HELD" if held else "MISSED"}')
        if not held:
            self.gripper(self.g_open, 'open')
            self.do_lift()
            return False
        # 取っ手の向き（中心→取っ手）をグリッパ座標系で記憶（持っている間は不変。置くときの向き合わせに使う）
        R = self.kin.fk({n: self.js[n] for n in ARM_JOINTS}, 'gripper_link')[:3, :3]
        self.grasped_axis_tool = R.T @ (handle_axis / (np.linalg.norm(handle_axis) + 1e-12))
        if np.linalg.norm(self.post_shift[:2]) > 1e-4:
            # 持ち上げる前に、掴んだ高さのまま水平に少し動かす（ユーザー指定: 奥へ 2 cm）
            shifted = gp + self.post_shift
            sol = self.solve_pose(shifted, handle_axis, 'grasp/shift', q_seed=q_g)
            if sol is None:
                return False
            q_s, _ = sol; q_s['wrist_roll'] = q_pre['wrist_roll']
            if self.move_with_correction(q_s, 'grasp/shift') is None:
                return False
            q_g = q_s; gp = shifted
        lift = gp + np.array([0, 0, self.lift_h])
        sol = self.solve_pose(lift, handle_axis, 'grasp/lift', q_seed=q_g)
        if sol is None:
            return False
        q_l, _ = sol; q_l['wrist_roll'] = q_pre['wrist_roll']
        if self.move_to(q_l, 'grasp/lift') is None:
            return False
        self.status('grasp: done (holding)')
        return True

    def do_lift(self):
        """現在の TCP を姿勢を保ったまま lift_height だけ真上へ（把持後の持ち上げ、単独実行用）。"""
        q0 = {n: self.js[n] for n in ARM_JOINTS}
        tcp, _ = self.kin.tcp(q0); up = tcp + np.array([0.0, 0.0, self.lift_h])
        q, pe, de, tilt = self.ik_any_tilt(up, q0, q0['wrist_roll'])
        if tilt is None:
            self.status(f'lift: IK failed (pos_err={pe*1000:.1f}mm dir_err={de:.1f}deg)'); return False
        q['wrist_roll'] = q0['wrist_roll']
        if self.move_to(q, 'lift') is None:
            return False
        tcp2, _ = self.kin.tcp({n: self.js[n] for n in ARM_JOINTS})
        self.status(f'lift: done. TCP z {tcp[2]:+.3f} -> {tcp2[2]:+.3f} m; gripper {math.degrees(self.js["gripper"]):.1f}deg')
        return True

    def do_place(self):
        """place_position_xyz（把持点の絶対座標）へ、取っ手を place_handle_axis_xy の向きにして置く。"""
        table_now = self.table_height()
        pp = self.place_xyz.copy()
        self.status(f'place: absolute target ({pp[0]:+.3f},{pp[1]:+.3f})')
        # 取っ手の向き: 把持時に記憶した取っ手ベクトル（gripper_link 系）が base で place_handle_axis_xy を向くロールを選ぶ（符号込み）。
        # 記憶が無い（place 単独実行など）ときは、指の閉じ方向が取っ手軸と直交する条件だけで、符号は現在値に近い方
        axis = self.place_axis
        tool_axis = self.grasped_axis_tool
        self.status(f'place: handle direction fixed to ({axis[0]:+.1f},{axis[1]:+.1f}) in base (+y = toward robot)'
                    + ('' if tool_axis is not None else ' [no grasp record: sign free]'))
        pp[2] = table_now + self.grasp_h + 0.01   # 置くときは 1 cm 高め（押し付け防止）
        sol = self.solve_pose(pp, axis, 'place/target', tool_axis=tool_axis, tilt_min=self.place_tilt_min)
        if sol is None:
            return False
        q_p, tilt = sol
        above = pp + np.array([0, 0, self.lift_h])
        sol = self.solve_pose(above, axis, 'place/above', q_seed=q_p, tool_axis=tool_axis, tilt_min=self.place_tilt_min)
        if sol is None:
            return False
        q_a, _ = sol
        # まず今のロールのまま上空へ運び、工具軸がほぼ鉛直な上空でロールだけ回す（運搬中にトレイが傾かない）
        q_a_keep = dict(q_a); q_a_keep['wrist_roll'] = self.js['wrist_roll']
        if self.safe_move(q_a_keep, 'place/above', correct=False) is None:
            return False
        if abs(q_a['wrist_roll'] - q_a_keep['wrist_roll']) > math.radians(2.0):
            if self.move_to(q_a, 'place/orient handle') is None:
                return False
        q_p['wrist_roll'] = q_a['wrist_roll']
        if self.move_with_correction(q_p, 'place/descend') is None:
            return False
        if not self.gripper(self.g_open, 'open'):
            return False
        if self.move_to(q_a, 'place/retreat') is None:
            return False
        self.status('place: done')
        return True


def main(args=None):
    rclpy.init(args=args)
    node = PickPlaceController()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except RuntimeError:
        if rclpy.ok():
            raise
    finally:
        if rclpy.ok():
            node.destroy_node()
            try:
                rclpy.shutdown()
            except Exception:  # noqa: BLE001
                pass


if __name__ == '__main__':
    main()
