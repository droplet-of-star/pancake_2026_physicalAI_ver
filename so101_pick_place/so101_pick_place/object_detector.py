"""赤いトレイ（円盤 + 取っ手）の検出ノード。

RGB 画像を購読し、HSV で赤色（Hue 0 付近と 180 付近の 2 区間）を抽出して最大輪郭を取り、
トレイ外周の円を求める。円の求め方（優先順）:
  1. rim_edge  : 灰色画像の Hough 円 + 色の整合（「すぐ内側は赤系」「赤画素の大半が円内」）。
                 ラップの反射やパンケーキで赤マスクが欠けても外周を捉える（2026-10-26）。
                 探索範囲は load_detector の tray 枠（新鮮なら）か赤領域の外接円から決める。
  2. rim_circle: 赤輪郭の点に RANSAC で円を当てる（rim_edge が解けないときの代用）。
  3. 内接円    : 輪郭マスクの距離変換の最大点（どちらも解けないとき）。
取っ手は、円を少し広げた外側に残る赤領域の重心方向とし、その軸上の「縁〜先端の handle_grasp_fraction」の点を把持画素にする。

配信:
  /so101_pick_place/object_pixel         PointStamped  x = u [px], y = v [px], z = 輪郭面積 [px^2]（フル解像度）。header は入力画像のまま
  /so101_pick_place/handle_pixel         PointStamped  x = u, y = v（把持画素）, z = 取っ手方向角 [rad]（画像系、円盤中心→取っ手先端）
  /so101_pick_place/object_bbox          RegionOfInterest  円の外接正方形（推定ノードの除外範囲、load_detector の基準円）
  /so101_pick_place/object_partial_pixel PointStamped  見切れたときのヒント（x,y: 赤領域の重心 px、z: 水平位置 -1..+1）
  /so101_pick_place/debug_image, /so101_pick_place/mask

未検出・見切れのときは object_pixel を配信しない（下流は「見えていなければ動かない」）。
"""
import math
import os
import time

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import PointStamped
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import Image, RegionOfInterest


class ObjectDetector(Node):
    def __init__(self):
        super().__init__('object_detector')
        self.declare_parameter('image_topic', '/camera/camera/color/image_raw')
        self.declare_parameter('debug_image_topic', '/so101_pick_place/debug_image')
        self.declare_parameter('hsv_lower1', [0, 120, 70])
        self.declare_parameter('hsv_upper1', [10, 255, 255])
        self.declare_parameter('hsv_lower2', [170, 120, 70])
        self.declare_parameter('hsv_upper2', [180, 255, 255])
        self.declare_parameter('min_area_px', 1500)
        self.declare_parameter('morph_kernel_px', 5)
        self.declare_parameter('process_scale', 0.5)   # 処理解像度（CPU 負荷軽減）
        self.declare_parameter('publish_debug', True)
        self.declare_parameter('border_margin_px', 8)   # 円がこの余白内で画面端に触れたら未検出扱い（円盤の見切れ）
        # 円の半径の許容範囲（処理解像度の画素）。rim_edge / rim_circle 共通
        self.declare_parameter('disk_radius_px_min', 40)
        self.declare_parameter('disk_radius_px_max', 400)
        # rim_circle（代用）: 輪郭点への RANSAC 円フィット
        self.declare_parameter('rim_ransac_iters', 150)
        self.declare_parameter('rim_inlier_px', 4.0)
        # rim_edge: Hough 円 + 色整合
        self.declare_parameter('rim_edge_canny', [60, 120])
        self.declare_parameter('rim_edge_hough_votes', 25)     # Hough 円の投票しきい値（param2）。小さいほど候補が増える
        self.declare_parameter('rim_edge_topk', 15)            # 投票上位いくつを色の整合で評価するか
        self.declare_parameter('rim_edge_tol_px', 2.0)         # 縁の支持率: 円周から Canny エッジまでの許容距離
        self.declare_parameter('rim_edge_roi_scale', 1.8)      # 赤領域の外接円 × これ を探索範囲にする（tray 枠が無いとき）
        self.declare_parameter('rim_edge_in_min', 0.4)         # 円のすぐ内側（r-9..r-2 px）の赤系割合の下限（載せ物の縁の円を捨てる）
        self.declare_parameter('rim_edge_contain_min', 0.55)   # 赤系画素のうち円の内側（r+4 px 以内）にある割合の下限（小さすぎる円を捨てる）
        self.declare_parameter('tray_bbox_topic', '/so101_pick_place/tray_bbox')   # 学習モデル（load_detector）のトレイ枠。新鮮なら赤領域より優先して円探索の ROI にする
        self.declare_parameter('tray_bbox_max_age_s', 1.0)
        # 取っ手検出: 円を handle_exclude_scale 倍に広げた外側の赤領域の重心を取っ手とみなす
        self.declare_parameter('handle_exclude_scale', 1.15)
        self.declare_parameter('handle_grasp_fraction', 0.6)   # 円盤縁から取っ手先端までの何割の位置を把持点にするか
        self.declare_parameter('handle_min_area_px', 800)
        self.declare_parameter('debug_save_dir', '')           # 空でなければ、検出フレームの生画像とデバッグ画像を 0.5 s ごとにここへ保存（直近 20 枚、解析用）

        p = self.get_parameter
        self.lower1 = np.array(p('hsv_lower1').value, dtype=np.uint8)
        self.upper1 = np.array(p('hsv_upper1').value, dtype=np.uint8)
        self.lower2 = np.array(p('hsv_lower2').value, dtype=np.uint8)
        self.upper2 = np.array(p('hsv_upper2').value, dtype=np.uint8)
        self.min_area = float(p('min_area_px').value)
        k = int(p('morph_kernel_px').value)
        self.kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (k, k)) if k > 1 else None
        self.scale = float(p('process_scale').value)
        self.publish_debug = bool(p('publish_debug').value)
        self.border_margin = int(p('border_margin_px').value)
        self.r_min = float(p('disk_radius_px_min').value); self.r_max = float(p('disk_radius_px_max').value)
        self.rim_iters = int(p('rim_ransac_iters').value); self.rim_tol = float(p('rim_inlier_px').value)
        self.edge_canny = [int(v) for v in p('rim_edge_canny').value]
        self.edge_hough_votes = int(p('rim_edge_hough_votes').value); self.edge_topk = int(p('rim_edge_topk').value)
        self.edge_tol = float(p('rim_edge_tol_px').value); self.edge_roi = float(p('rim_edge_roi_scale').value)
        self.edge_in_min = float(p('rim_edge_in_min').value); self.edge_contain = float(p('rim_edge_contain_min').value)
        self.tray_bbox = None; self.tray_bbox_time = 0.0; self.tray_bbox_max_age = float(p('tray_bbox_max_age_s').value)
        self.handle_excl = float(p('handle_exclude_scale').value)
        self.handle_frac = float(p('handle_grasp_fraction').value)
        self.handle_min_area = float(p('handle_min_area_px').value)
        self.dbg_dir = str(p('debug_save_dir').value); self._dbg_last = 0.0; self._dbg_n = 0
        if self.dbg_dir:
            self.dbg_dir = os.path.expanduser(self.dbg_dir); os.makedirs(self.dbg_dir, exist_ok=True)
        self._rng = np.random.default_rng(0)

        self.bridge = CvBridge()
        image_topic = p('image_topic').value
        self.sub = self.create_subscription(Image, image_topic, self.on_image, 10)
        self.create_subscription(RegionOfInterest, str(p('tray_bbox_topic').value), self.on_tray_bbox, 10)
        self.pixel_pub = self.create_publisher(PointStamped, '/so101_pick_place/object_pixel', 10)
        self.partial_pub = self.create_publisher(PointStamped, '/so101_pick_place/object_partial_pixel', 10)
        self.handle_pub = self.create_publisher(PointStamped, '/so101_pick_place/handle_pixel', 10)
        self.bbox_pub = self.create_publisher(RegionOfInterest, '/so101_pick_place/object_bbox', 10)
        self.debug_pub = self.create_publisher(Image, p('debug_image_topic').value, 10)
        self.mask_pub = self.create_publisher(Image, '/so101_pick_place/mask', 10)

        self._frames = 0; self._detections = 0
        self._edge_ok = 0; self._edge_fail = 0
        self.create_timer(5.0, self._report)
        self.get_logger().info(
            f'object_detector: {image_topic} -> object_pixel  '
            f'HSV1 {self.lower1.tolist()}-{self.upper1.tolist()} HSV2 {self.lower2.tolist()}-{self.upper2.tolist()} '
            f'min_area={self.min_area:.0f}px scale={self.scale}')

    # ------------------------------------------------------------------ callbacks
    def on_tray_bbox(self, msg: RegionOfInterest):
        self.tray_bbox = (int(msg.x_offset), int(msg.y_offset), int(msg.width), int(msg.height)); self.tray_bbox_time = time.time()

    # ------------------------------------------------------------------ red mask
    def detect(self, bgr: np.ndarray):
        """赤領域の最大輪郭を返す。戻り値: (mask, contour, area) または (mask, None, 0)。"""
        hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, self.lower1, self.upper1) | cv2.inRange(hsv, self.lower2, self.upper2)
        if self.kernel is not None:
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, self.kernel)
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, self.kernel)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            return mask, None, 0.0
        best = max(contours, key=cv2.contourArea)
        return mask, best, float(cv2.contourArea(best))

    # ------------------------------------------------------------------ circle fitting
    def fit_rim_circle(self, contour):
        """輪郭点に RANSAC で円をあてはめる（代用）。戻り値 (cx, cy, r) または None。"""
        pts = contour.reshape(-1, 2).astype(np.float64)
        if len(pts) < 20:
            return None
        best = None
        for _ in range(self.rim_iters):
            i = self._rng.choice(len(pts), 3, replace=False)
            (x1, y1), (x2, y2), (x3, y3) = pts[i]
            d = 2 * (x1 * (y2 - y3) + x2 * (y3 - y1) + x3 * (y1 - y2))
            if abs(d) < 1e-6:
                continue
            ux = ((x1**2 + y1**2) * (y2 - y3) + (x2**2 + y2**2) * (y3 - y1) + (x3**2 + y3**2) * (y1 - y2)) / d
            uy = ((x1**2 + y1**2) * (x3 - x2) + (x2**2 + y2**2) * (x1 - x3) + (x3**2 + y3**2) * (x2 - x1)) / d
            r = math.hypot(x1 - ux, y1 - uy)
            if not (self.r_min <= r <= self.r_max):
                continue
            inl = np.abs(np.hypot(pts[:, 0] - ux, pts[:, 1] - uy) - r) < self.rim_tol
            n = int(inl.sum())
            if best is None or n > best[0]:
                best = (n, ux, uy, r, inl)
        if best is None:
            return None
        _, ux, uy, r, inl = best
        # inlier で最小二乗精緻化（代数的円フィット）
        q = pts[inl]
        A = np.c_[2 * q[:, 0], 2 * q[:, 1], np.ones(len(q))]; b = (q ** 2).sum(1)
        try:
            sol, *_ = np.linalg.lstsq(A, b, rcond=None)
            ux, uy = sol[0], sol[1]; r = math.sqrt(max(1e-9, sol[2] + ux**2 + uy**2))
        except Exception:  # noqa: BLE001
            pass
        return float(ux), float(uy), float(r)

    def fit_rim_edges(self, bgr, contour):
        """トレイ外周の円を Hough 円変換（勾配ベース、決定的）で求め、色の整合で候補を選ぶ。
        候補ごとに: 縁の支持率（円周上で Canny エッジが tol px 以内にある割合）、containment（赤系画素のうち円の内側にある割合）、
        f_in / f_out（円のすぐ内側 / 外側の赤系割合）を計算。f_in が低い候補（パンケーキ等の載せ物の縁）と、赤領域が大きく外に
        はみ出す候補（小さすぎる円）を捨て、支持率 + 0.3*containment - 0.3*f_out が最大の候補を返す。
        つや消しのトレイでは内側の縁（床と壁の境）が最も強いエッジになり、外側に赤が続くため f_out は条件にしない。
        戻り値 (cx, cy, r) または None。座標は bgr（処理解像度）の画素。"""
        H, W = bgr.shape[:2]
        if self.tray_bbox is not None and time.time() - self.tray_bbox_time <= self.tray_bbox_max_age:
            # 学習モデルのトレイ枠を優先（赤領域が取っ手だけ・虫食いでも正しい範囲で円を探せる）
            # 枠は「円盤だけ」を囲む約束（取っ手は含めない）→ 枠の中心 = 円の中心の見当、辺の半分 = 半径の見当
            tx, ty, tw, th = self.tray_bbox; sc = self.scale
            cx0, cy0 = (tx + tw / 2.0) * sc, (ty + th / 2.0) * sc; R = (tw + th) / 4.0 * sc
            m = int(R * 1.3); r_span = (0.75, 1.3)
        else:
            x, y, w, h = cv2.boundingRect(contour); cx0, cy0 = x + w / 2.0, y + h / 2.0; R = max(w, h) / 2.0
            m = int(R * self.edge_roi); r_span = (0.6, 1.7)
        x0, y0, x1, y1 = int(max(0, cx0 - m)), int(max(0, cy0 - m)), int(min(W, cx0 + m)), int(min(H, cy0 + m))
        roi = bgr[y0:y1, x0:x1]
        if roi.shape[0] < 32 or roi.shape[1] < 32:
            return None
        gray = cv2.GaussianBlur(cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY), (5, 5), 1.2)
        r_lo, r_hi = int(max(self.r_min, R * r_span[0])), int(min(self.r_max, R * r_span[1]))
        if r_hi <= r_lo:
            return None
        circles = cv2.HoughCircles(gray, cv2.HOUGH_GRADIENT, dp=1.2, minDist=8, param1=self.edge_canny[1],
                                   param2=self.edge_hough_votes, minRadius=r_lo, maxRadius=r_hi)
        if circles is None:
            return None
        edges = cv2.Canny(gray, self.edge_canny[0], self.edge_canny[1])
        ys, xs = np.nonzero(edges)
        if len(xs) < 30:
            return None
        pts = np.c_[xs, ys].astype(np.float64)
        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
        hh, ss, vv = hsv[..., 0], hsv[..., 1], hsv[..., 2]
        red = (((hh <= 12) | (hh >= 150)) & (ss >= 45) & (vv >= 40))
        red_cnt = max(1, int(red.sum()))
        yy, xx = np.mgrid[0:red.shape[0], 0:red.shape[1]]
        best = None
        for (ux, uy, r) in circles[0][:self.edge_topk]:
            ux, uy, r = float(ux), float(uy), float(r)
            dd = np.hypot(xx - ux, yy - uy)
            inner = red[(dd > r - 9) & (dd < r - 2)]; outer = red[(dd > r + 2) & (dd < r + 9)]
            if inner.size < 30 or outer.size < 30:
                continue
            f_in, f_out = float(inner.mean()), float(outer.mean())
            contain = float((red & (dd <= r + 4.0)).sum()) / red_cnt
            if f_in < self.edge_in_min or contain < self.edge_contain:
                continue
            n_sup = int((np.abs(np.hypot(pts[:, 0] - ux, pts[:, 1] - uy) - r) < self.edge_tol).sum())
            support = n_sup / (2.0 * math.pi * r)
            score = support + 0.3 * contain - 0.3 * f_out
            if best is None or score > best[0]:
                best = (score, ux, uy, r)
        if best is None:
            return None
        _, ux, uy, r = best
        return float(ux + x0), float(uy + y0), float(r)

    def find_disk(self, bgr, contour, contour_mask):
        """円盤の円 (cx, cy, r)。rim_edge → rim_circle → 内接円（距離変換）の順に試す。"""
        fit = self.fit_rim_edges(bgr, contour)
        if fit is not None:
            self._edge_ok += 1
            return fit
        self._edge_fail += 1
        fit = self.fit_rim_circle(contour)
        if fit is not None:
            return fit
        dist = cv2.distanceTransform(contour_mask, cv2.DIST_L2, 5)
        _, radius, _, (cu, cv_) = cv2.minMaxLoc(dist)
        return float(cu), float(cv_), float(radius)

    def find_handle(self, contour_mask, cu, cv_, radius, s):
        """円の外側に残る赤領域から取っ手を求める。戻り値 (grasp_u, grasp_v, angle, centroid_u, centroid_v, tip_dist) または None。"""
        hm = contour_mask.copy()
        cv2.circle(hm, (int(cu), int(cv_)), int(radius * self.handle_excl), 0, -1)
        hys, hxs = np.nonzero(hm)
        if hxs.size * (1.0 / (s * s)) < self.handle_min_area:
            return None
        hcx, hcy = float(hxs.mean()), float(hys.mean())
        ang = math.atan2(hcy - cv_, hcx - cu)
        dcen = math.hypot(hcx - cu, hcy - cv_) / max(1.0, radius)
        if not (1.2 <= dcen <= 2.3):
            # 取っ手なら重心は円の外（縁から先端の中ほど）。縁の残りや別の赤い物なら不合格 → 取っ手なしとして扱う
            self.get_logger().warn(f'handle candidate rejected: centroid at {dcen:.2f} r from center (expect 1.2-2.3)', throttle_duration_sec=2.0)
            return None
        # 取っ手軸上の距離: 円盤縁（radius）から先端（最遠点）まで
        d_all = (hxs - cu) * math.cos(ang) + (hys - cv_) * math.sin(ang)
        tip = float(d_all.max())
        g = radius + (tip - radius) * self.handle_frac
        return (cu + g * math.cos(ang), cv_ + g * math.sin(ang), ang, hcx, hcy, tip)

    # ------------------------------------------------------------------ main
    def on_image(self, msg: Image):
        self._frames += 1
        try:
            bgr_full = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(f'cv_bridge failed: {exc}', throttle_duration_sec=5.0)
            return

        s = self.scale
        bgr = cv2.resize(bgr_full, None, fx=s, fy=s, interpolation=cv2.INTER_AREA) if s != 1.0 else bgr_full
        mask, contour, area = self.detect(bgr)
        area_full = area / (s * s)

        found = contour is not None and area_full >= self.min_area
        touches_border = False
        radius = 0.0; handle = None
        if found:
            x, y, w, h = cv2.boundingRect(contour)
            H, W = bgr.shape[:2]
            mb = max(1, int(round(self.border_margin * s)))
            cm = np.zeros(mask.shape, dtype=np.uint8)
            cv2.drawContours(cm, [contour], -1, 255, -1)
            cu, cv_, radius = self.find_disk(bgr, contour, cm)
            handle = self.find_handle(cm, cu, cv_, radius, s)
            # 円盤が画面端で切れていると半径が過小評価される: 円が画面端に触れていたら除外
            touches_border = (cu - radius <= mb or cv_ - radius <= mb or
                              cu + radius >= W - mb or cv_ + radius >= H - mb)
            if touches_border:
                found = False
                self.get_logger().warn('object touches image border: skipped (partially out of view)',
                                       throttle_duration_sec=2.0)
                # 探索用ヒント: 見切れた赤領域の重心と、画面中心からの水平位置（-1 = 左端, +1 = 右端）
                hint = PointStamped(); hint.header = msg.header
                bx, by = x + w / 2.0, y + h / 2.0
                hint.point.x, hint.point.y, hint.point.z = float(bx / s), float(by / s), float((bx - W / 2.0) / (W / 2.0))
                self.partial_pub.publish(hint)
        if found:
            u, v = cu / s, cv_ / s
            self._detections += 1

            pt = PointStamped()
            pt.header = msg.header
            pt.point.x, pt.point.y, pt.point.z = float(u), float(v), float(area_full)
            self.pixel_pub.publish(pt)
            if handle is not None:
                hu, hv, hang = handle[:3]
                hp = PointStamped(); hp.header = msg.header
                hp.point.x, hp.point.y, hp.point.z = float(hu / s), float(hv / s), float(hang)
                self.handle_pub.publish(hp)

            roi = RegionOfInterest()
            if radius > 0:
                # 円盤の外接正方形（載せ物で赤領域が欠けても、推定ノードの除外範囲が円盤全体を覆うように）
                rx0, ry0 = max(0, cu - radius), max(0, cv_ - radius)
                rx1, ry1 = min(bgr.shape[1], cu + radius), min(bgr.shape[0], cv_ + radius)
                roi.x_offset, roi.y_offset = int(rx0 / s), int(ry0 / s)
                roi.width, roi.height = int((rx1 - rx0) / s), int((ry1 - ry0) / s)
            else:
                roi.x_offset, roi.y_offset = int(x / s), int(y / s)
                roi.width, roi.height = int(w / s), int(h / s)
            roi.do_rectify = False
            self.bbox_pub.publish(roi)

        if self.publish_debug:
            dbg = bgr.copy()
            if contour is not None:
                color = (0, 255, 0) if found else (0, 165, 255)
                cv2.drawContours(dbg, [contour], -1, color, 2)
                x, y, w, h = cv2.boundingRect(contour)
                cv2.rectangle(dbg, (x, y), (x + w, y + h), color, 1)
            if contour is not None and radius > 0:
                cv2.circle(dbg, (int(cu), int(cv_)), int(radius), (255, 255, 0) if found else (0, 165, 255), 1)
            if found and handle is not None:
                hu, hv, hang, _, _, tip = handle
                cv2.line(dbg, (int(cu), int(cv_)), (int(cu + tip * math.cos(hang)), int(cv_ + tip * math.sin(hang))), (0, 255, 255), 1)
                cv2.drawMarker(dbg, (int(hu), int(hv)), (0, 255, 255), cv2.MARKER_TILTED_CROSS, 16, 2)
                cv2.putText(dbg, f'handle {math.degrees(hang):+.0f}deg', (10, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
            if found:
                cv2.drawMarker(dbg, (int(cu), int(cv_)), (255, 255, 0), cv2.MARKER_CROSS, 20, 2)
                cv2.putText(dbg, f'u={u:.0f} v={v:.0f} area={area_full:.0f}', (10, 25),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 0), 2)
            else:
                msg_txt = 'object touches border' if touches_border else 'no red object'
                cv2.putText(dbg, msg_txt, (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
            out = self.bridge.cv2_to_imgmsg(dbg, encoding='bgr8')
            out.header = msg.header
            if self.dbg_dir and found and time.time() - self._dbg_last > 0.5:
                self._dbg_last = time.time(); k = self._dbg_n % 20; self._dbg_n += 1
                cv2.imwrite(f'{self.dbg_dir}/raw_{k:02d}.png', bgr_full); cv2.imwrite(f'{self.dbg_dir}/dbg_{k:02d}.png', dbg)
            self.debug_pub.publish(out)
            mk = self.bridge.cv2_to_imgmsg(mask, encoding='mono8')
            mk.header = msg.header
            self.mask_pub.publish(mk)

    def _report(self):
        if self._frames:
            self.get_logger().info(
                f'last 5 s: frames={self._frames} detections={self._detections} rim_edge ok={self._edge_ok} fallback={self._edge_fail}',
                throttle_duration_sec=4.0)
        self._frames = 0; self._detections = 0
        self._edge_ok = 0; self._edge_fail = 0


def main(args=None):
    rclpy.init(args=args)
    node = ObjectDetector()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except RuntimeError:
        # Humble: SIGINT 直後に spin 内の take_message が RuntimeError を出すことがある
        if rclpy.ok():
            raise
    finally:
        if rclpy.ok():
            node.destroy_node()
            try:
                rclpy.shutdown()
            except Exception:  # noqa: BLE001  (already shut down by signal handler)
                pass


if __name__ == '__main__':
    main()
