"""SO-101 の軽量運動学（URDF を直接読む FK と数値 IK）。MoveIt を使わずに STEP 5 を進めるためのもの。

- FK: base_link -> 任意リンク（gripper_link, camera_link, tcp）
- IK: 5 関節（pan, lift, elbow, wrist_flex, wrist_roll）で「TCP 位置 + 接近方向」を満たす解を
       scipy.optimize.least_squares（関節範囲つき）で求める。wrist_roll は接近軸まわりの回転なので自由。
- TCP: gripper_link 原点から指先方向（gripper_link の -z）に tcp_offset だけ進んだ点。
"""
import math
import xml.etree.ElementTree as ET

import numpy as np
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation as R

ARM_JOINTS = ['shoulder_pan', 'shoulder_lift', 'elbow_flex', 'wrist_flex', 'wrist_roll']


def _T(xyz, rpy):
    M = np.eye(4)
    M[:3, :3] = R.from_euler('xyz', rpy).as_matrix()
    M[:3, 3] = xyz
    return M


def _rot(axis, q):
    M = np.eye(4)
    M[:3, :3] = R.from_rotvec(np.asarray(axis, float) / np.linalg.norm(axis) * q).as_matrix()
    return M


class SO101Kinematics:
    def __init__(self, urdf_xml: str, base='base_link', tcp_offset=0.09, joint_limits_rad=None, tcp_lateral=0.0):
        root = ET.fromstring(urdf_xml)
        self.joints = {}      # child_link -> dict(name, parent, T, axis, type, lower, upper)
        for j in root.findall('joint'):
            o = j.find('origin'); ax = j.find('axis'); lim = j.find('limit')
            xyz = [float(v) for v in (o.get('xyz', '0 0 0') if o is not None else '0 0 0').split()]
            rpy = [float(v) for v in (o.get('rpy', '0 0 0') if o is not None else '0 0 0').split()]
            self.joints[j.find('child').get('link')] = dict(
                name=j.get('name'), parent=j.find('parent').get('link'), T=_T(xyz, rpy),
                axis=[float(v) for v in ax.get('xyz').split()] if ax is not None else None,
                type=j.get('type'),
                lower=float(lim.get('lower')) if lim is not None and lim.get('lower') else -math.pi,
                upper=float(lim.get('upper')) if lim is not None and lim.get('upper') else math.pi)
        self.base = base
        self.tcp_offset = float(tcp_offset)
        self.tcp_lateral = float(tcp_lateral)   # gripper_link x 方向（指の閉じ方向）のオフセット。固定爪の内側面＋板半幅に合わせる
        # 関節範囲: URDF とキャリブ範囲の共通部分
        self.limits = {}
        for link, j in self.joints.items():
            if j['type'] == 'revolute':
                lo, hi = j['lower'], j['upper']
                if joint_limits_rad and j['name'] in joint_limits_rad:
                    lo = max(lo, joint_limits_rad[j['name']][0]); hi = min(hi, joint_limits_rad[j['name']][1])
                self.limits[j['name']] = (lo, hi)

    # ------------------------------------------------------------------
    def chain(self, link):
        out = []
        while link != self.base:
            j = self.joints[link]; out.append(j); link = j['parent']
        return out[::-1]

    def fk(self, q: dict, link='gripper_link'):
        """base_link から link への 4x4。q は関節名 -> rad。"""
        M = np.eye(4)
        for j in self.chain(link):
            M = M @ j['T']
            if j['type'] == 'revolute':
                M = M @ _rot(j['axis'], q.get(j['name'], 0.0))
        return M

    def path_min_heights(self, q_from: dict, q_to: dict, n=41):
        """関節補間経路上で TCP / camera_link / gripper_link の最低高さ [m] を返す（移動前の安全検査用）。"""
        mins = {'tcp': 9.0, 'camera': 9.0, 'gripper': 9.0}
        for s in np.linspace(0.0, 1.0, n):
            q = {k: q_from[k] + (q_to[k] - q_from[k]) * s for k in q_to}
            mins['tcp'] = min(mins['tcp'], float(self.tcp(q)[0][2]))
            mins['camera'] = min(mins['camera'], float(self.fk(q, 'camera_link')[2, 3]))
            mins['gripper'] = min(mins['gripper'], float(self.fk(q, 'gripper_link')[2, 3]))
        return mins

    def closing_dir(self, q: dict):
        """指が閉じる方向（gripper_link の +x、base 系、単位ベクトル）。"""
        return self.fk(q, 'gripper_link')[:3, 0]

    def solve_roll(self, q: dict, handle_axis, prefer=None):
        """wrist_roll だけを変えて、指の閉じる方向を handle_axis（base 系、水平）と直交させる。
        候補は 2 つ（180° 違い）。prefer（rad）に近い方を返す。関節範囲内の解だけ。"""
        lo, hi = self.limits.get('wrist_roll', (-math.pi, math.pi))
        h = np.asarray(handle_axis, float); h = h / (np.linalg.norm(h) + 1e-12)
        best = None
        for r in np.linspace(lo, hi, 721):
            qq = dict(q); qq['wrist_roll'] = float(r)
            c = abs(float(self.closing_dir(qq) @ h))
            cost = c + (0.0 if prefer is None else 0.05 * abs(r - prefer) / math.pi)
            if best is None or cost < best[0]:
                best = (cost, float(r), c)
        return best[1], math.degrees(math.acos(min(1.0, best[2])))

    def solve_roll_signed(self, q: dict, axis_tool, desired_dir, prefer=None):
        """wrist_roll だけを変えて、グリッパに固定されたベクトル axis_tool（gripper_link 系）が
        base 系で desired_dir（水平）の向きになる角を返す。向きの符号まで合わせる（解は 1 つ）。
        戻り値 (roll, 残り角度誤差 deg)。"""
        lo, hi = self.limits.get('wrist_roll', (-math.pi, math.pi))
        a = np.asarray(axis_tool, float); a = a / (np.linalg.norm(a) + 1e-12)
        d = np.asarray(desired_dir, float).copy(); d[2] = 0.0; d = d / (np.linalg.norm(d) + 1e-12)
        best = None
        for r in np.linspace(lo, hi, 721):
            qq = dict(q); qq['wrist_roll'] = float(r)
            v = self.fk(qq, 'gripper_link')[:3, :3] @ a; v[2] = 0.0; v = v / (np.linalg.norm(v) + 1e-12)
            c = 1.0 - float(v @ d)
            cost = c + (0.0 if prefer is None else 0.02 * abs(r - prefer) / math.pi)
            if best is None or cost < best[0]:
                best = (cost, float(r), c)
        return best[1], math.degrees(math.acos(max(-1.0, min(1.0, 1.0 - best[2]))))

    def tcp(self, q: dict):
        """TCP 位置と接近方向（単位ベクトル、base 系）。接近方向 = gripper_link の -z。"""
        M = self.fk(q, 'gripper_link')
        approach = -M[:3, 2]
        return M[:3, 3] + approach * self.tcp_offset + M[:3, 0] * self.tcp_lateral, approach

    # ------------------------------------------------------------------
    def ik(self, target_pos, approach_dir=(0, 0, -1), q0=None, fixed=None, w_dir=0.05, max_iter=200,
           pos_tol=0.005, dir_tol_deg=5.0):
        """TCP を target_pos に、接近方向を approach_dir に向ける 5 関節解。
        fixed: 固定する関節 {name: rad}（例: wrist_roll）。戻り値 (q dict, pos_err_m, dir_err_deg, success)。"""
        q0 = dict(q0 or {}); fixed = dict(fixed or {})
        free = [n for n in ARM_JOINTS if n not in fixed]
        x0 = np.array([q0.get(n, 0.0) for n in free])
        lo = np.array([self.limits.get(n, (-math.pi, math.pi))[0] for n in free])
        hi = np.array([self.limits.get(n, (-math.pi, math.pi))[1] for n in free])
        x0 = np.clip(x0, lo + 1e-6, hi - 1e-6)
        tgt = np.asarray(target_pos, float); ad = np.asarray(approach_dir, float); ad /= np.linalg.norm(ad)

        def resid(x):
            q = dict(fixed); q.update(zip(free, x))
            p, a = self.tcp(q)
            return np.r_[p - tgt, w_dir * (a - ad)]

        sol = least_squares(resid, x0, bounds=(lo, hi), max_nfev=max_iter * 10, xtol=1e-10, ftol=1e-10)
        q = dict(fixed); q.update(zip(free, sol.x))
        p, a = self.tcp(q)
        pos_err = float(np.linalg.norm(p - tgt))
        dir_err = math.degrees(math.acos(max(-1.0, min(1.0, float(a @ ad)))))
        return q, pos_err, dir_err, (pos_err < pos_tol and dir_err < dir_tol_deg)


def limits_from_calibration(calib: dict):
    """calibrate_zero の yaml（range は home 基準 unwrapped）から関節範囲 [rad] を作る。"""
    out = {}
    for n, c in calib.items():
        if not isinstance(c, dict) or 'homing_offset' not in c:
            continue
        d = int(c.get('direction', 1)); h = int(c['homing_offset'])
        lo = d * (int(c['range_min']) - h) * 2 * math.pi / 4096
        hi = d * (int(c['range_max']) - h) * 2 * math.pi / 4096
        out[n] = (min(lo, hi), max(lo, hi))
    return out
