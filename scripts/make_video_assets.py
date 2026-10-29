# -*- coding: utf-8 -*-
from PIL import Image, ImageDraw, ImageFont
import os
ROOT = '/home/jetson1/vscode/muran2026_physicalAI_ver'
PHOTO = os.path.join(ROOT, 'docs/photo'); OUT = os.path.join(ROOT, 'docs/video_assets')
W, H = 1920, 1080
BG = (250, 250, 248); INK = (30, 34, 40); SUB = (90, 96, 104); ACC = (190, 40, 50); ACC2 = (230, 120, 60); PANEL = (240, 240, 236); LINE = (200, 200, 196)
REG = '/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc'; BOLD = '/usr/share/fonts/opentype/noto/NotoSansCJK-Bold.ttc'
def F(size, bold=False): return ImageFont.truetype(BOLD if bold else REG, size, index=0)

def new():
    im = Image.new('RGB', (W, H), BG); d = ImageDraw.Draw(im)
    d.rectangle([0, 0, 18, H], fill=ACC)
    return im, d
def header(d, title, num=None):
    if num: d.text((70, 44), num, font=F(30, True), fill=ACC)
    d.text((70, 84), title, font=F(60, True), fill=INK)
    d.line([70, 170, W - 70, 170], fill=LINE, width=3)
def footer(d, txt='Physical AI 応用 1 最終課題  SO-101 × OAK-D Pro  Pick & Place'):
    d.text((70, H - 56), txt, font=F(24), fill=SUB)
def fit(path, box_w, box_h):
    im = Image.open(path).convert('RGB'); r = min(box_w / im.width, box_h / im.height)
    return im.resize((int(im.width * r), int(im.height * r)), Image.LANCZOS)
def paste_fit(im, path, x, y, bw, bh, caption=None, d=None, border=True):
    p = fit(path, bw, bh); px, py = x + (bw - p.width) // 2, y + (bh - p.height) // 2
    im.paste(p, (px, py))
    if border: ImageDraw.Draw(im).rectangle([px - 2, py - 2, px + p.width + 1, py + p.height + 1], outline=LINE, width=2)
    if caption and d:
        tw = d.textlength(caption, font=F(28)); d.text((x + (bw - tw) / 2, py + p.height + 12), caption, font=F(28), fill=INK)
    return (px, py, p.width, p.height)
def bullets(d, items, x, y, size=36, gap=18, width=None, bold_lead=True, color=INK):
    f = F(size); fb = F(size, True); lh = size + gap
    for it in items:
        lead, body = (it if isinstance(it, tuple) else (None, it))
        d.ellipse([x, y + size * 0.38, x + size * 0.32, y + size * 0.70], fill=ACC)
        cx = x + size * 0.9
        if lead:
            d.text((cx, y), lead, font=fb, fill=color); cx += d.textlength(lead, font=fb) + 12
        # wrap body
        lines = wrap(d, body, f, (width or (W - 90 - cx)) - (cx - x))
        for i, ln in enumerate(lines):
            d.text((cx if i == 0 else x + size * 0.9, y), ln, font=f, fill=color); y += lh
        y += gap * 0.6
    return y
import re
NOSTART = set('、。，．）」』】〕〉》］｝,.)]}:;!?・ー〜%')
def wrap(d, text, f, maxw):
    toks = re.findall(r'[A-Za-z0-9_./+\-&%°]+ ?|.', text)
    out, cur = [], ''
    for i, t in enumerate(toks):
        if d.textlength(cur + t, font=f) > maxw and cur:
            if t[0] in NOSTART:
                cur += t; continue
            out.append(cur.rstrip()); cur = t
        else: cur += t
    if cur: out.append(cur.rstrip())
    return out
def chips(d, items, x, y, size=30, h=64, pad=22, gap=14, maxw=W - 140, hl=None):
    f = F(size, True); cx, cy = x, y; arrow = F(size)
    for i, t in enumerate(items):
        tw = d.textlength(t, font=f); bw = tw + pad * 2
        if cx + bw > x + maxw: cx = x; cy += h + 22
        col = ACC if (hl is not None and i in hl) else (60, 64, 72)
        d.rounded_rectangle([cx, cy, cx + bw, cy + h], radius=14, fill=col)
        d.text((cx + pad, cy + (h - size) / 2 - 4), t, font=f, fill=(255, 255, 255))
        cx += bw + gap
        if i < len(items) - 1:
            d.text((cx, cy + (h - size) / 2 - 2), '▶', font=arrow, fill=SUB); cx += d.textlength('▶', font=arrow) + gap
    return cy + h
def table(d, rows, x, y, colw, size=30, rowh=58, head=True):
    f = F(size); fb = F(size, True)
    for r, row in enumerate(rows):
        if r == 0 and head: d.rectangle([x, y, x + sum(colw), y + rowh], fill=(60, 64, 72))
        elif r % 2 == 0: d.rectangle([x, y, x + sum(colw), y + rowh], fill=PANEL)
        cx = x
        for c, cell in enumerate(row):
            d.text((cx + 16, y + (rowh - size) / 2 - 4), cell, font=fb if (r == 0 and head) else f, fill=(255, 255, 255) if (r == 0 and head) else INK)
            cx += colw[c]
        y += rowh
    d.line([x, y, x + sum(colw), y], fill=LINE, width=2)
    return y
def save(im, name):
    p = os.path.join(OUT, 'slides', name); im.save(p, optimize=True); print('saved', p)

# 01 title
im, d = new()
d.rectangle([0, 0, W, H], fill=(28, 32, 40)); d.rectangle([0, 0, 18, H], fill=ACC)
d.text((90, 150), 'Physical AI 応用 1 講座　最終課題', font=F(40), fill=(200, 204, 210))
for i, ln in enumerate(['ROS を用いた SO-101 による', 'パンケーキ皿の自律 Pick & Place']):
    d.text((90, 240 + i * 100), ln, font=F(78, True), fill=(255, 255, 255))
d.text((90, 480), 'パンケーキ焼きロボット 工程④「お客さんに差し出す」', font=F(42), fill=ACC2)
d.text((90, 600), 'SO-101（6 軸・Feetech STS3215）× OAK-D Pro（手首ステレオ深度）', font=F(32), fill=(200, 204, 210))
d.text((90, 650), '× Jetson　ROS 2 Humble / Python', font=F(32), fill=(200, 204, 210))
d.text((90, 860), 'omnicampus アカウント名: ＿＿＿＿＿＿＿＿', font=F(36), fill=(255, 255, 255))
d.text((90, 920), '2026 年 10 月', font=F(32), fill=(200, 204, 210))
paste_fit(im, os.path.join(PHOTO, 'IMG_1791.jpg'), 1330, 560, 520, 440, border=False)
save(im, '01_title.png')

# 02 overview
im, d = new(); header(d, 'パンケーキ焼きロボット（サークル製作）の 4 工程', '背景')
steps = [('① メーカーを開ける', 'SO-101', 'パンケーキメーカーを開ける.png'), ('② 焼印を押す', '焼印装置', '焼印をおす.png'),
         ('③ お皿に乗せる', '別ロボット（仮）', 'パンケーキをお皿に置く(仮).jpg'), ('④ お客さんに差し出す', 'SO-101（本課題）', 'お客さんに差し出す.png')]
bw = 420; gapx = 30; x0 = 70
for i, (t, who, ph) in enumerate(steps):
    x = x0 + i * (bw + gapx); hl = (i == 3)
    d.rounded_rectangle([x, 200, x + bw, 860], radius=18, fill=(255, 240, 240) if hl else PANEL, outline=ACC if hl else LINE, width=4 if hl else 2)
    d.text((x + 24, 220), t, font=F(36, True), fill=ACC if hl else INK)
    d.text((x + 24, 272), '担当: ' + who, font=F(28), fill=SUB)
    if ph: paste_fit(im, os.path.join(PHOTO, ph), x + 20, 330, bw - 40, 500)
    else:
        d.text((x + 24, 520), '（写真なし）', font=F(30), fill=SUB)
for i, ln in enumerate(wrap(d, '本課題はこのうち ④ を担当する。トレイにパンケーキが載ったことを確認し、取っ手を掴んで、お客さんの前に取っ手を手前向きにして置く。', F(32), W - 140)):
    d.text((70, 890 + i * 46), ln, font=F(32), fill=INK)
footer(d); save(im, '02_overview.png')

# 03 task
im, d = new(); header(d, '今回のタスク: 1 コマンドで観測から配置まで自律実行', '課題')
y = chips(d, ['観測（見えなければ探す）', '【AI】パンケーキの確認', '取っ手の 3D 推定', '接近・把持', '把持判定（空振りならやり直し）', '奥へ 2 cm → 持ち上げ', '上空で取っ手を手前向きに回す', '降ろして離す', '観測姿勢へ戻る'], 70, 200, hl=[1])
bullets(d, [('対象:', '赤いトレイ（直径 150 mm、縁 18 mm、取っ手は幅 9 mm の板・高さ 30〜33 mm）。自作 CAD'),
            ('ルール:', '空のトレイは運ばない。パンケーキが載ったら運ぶ（自前学習の物体検出で判定。アメ・ケーキも学習済み）'),
            ('置き方:', '左 5 cm・奥 30 cm の決めた位置に、取っ手を必ず手前向きにそろえる'),
            ('方針:', 'VLA ではなくモデルベース。認識は物体検出に限定し、動作は幾何（平面推定・運動学）で組む')],
        70, y + 50, size=34, width=1050)
paste_fit(im, os.path.join(PHOTO, 'IMG_1791.jpg'), 1200, y + 40, 650, 520)
footer(d); save(im, '03_task.png')

# 04 system
im, d = new(); header(d, 'システム構成: 自作 ROS 2 ノード 6 つ（Python 約 2,300 行）', '構成')
boxes = [('oak_camera', 'RGB / 位置合わせ深度\n(depthai)'), ('object_detector', 'HSV + Hough 円\n取っ手の向き'), ('object_pose_estimator', '台平面 × 視線\n→ base_link (TF)'),
         ('pick_place_controller', 'FK/IK・状態機械\n実測補正・安全'), ('so101_motor_bridge', 'Feetech バス\n(Lerobot_ros2 改変)')]
bx, by, bwid, bh = 70, 260, 330, 190; gx = 32
for i, (n, sub) in enumerate(boxes):
    x = bx + i * (bwid + gx)
    d.rounded_rectangle([x, by, x + bwid, by + bh], radius=16, fill=(60, 64, 72))
    d.text((x + 18, by + 18), n, font=F(26, True), fill=(255, 255, 255))
    d.multiline_text((x + 18, by + 70), sub, font=F(26), fill=(220, 222, 226), spacing=8)
    if i < len(boxes) - 1: d.text((x + bwid + 4, by + bh / 2 - 20), '▶', font=F(30), fill=SUB)
# load_detector below
x = bx + 1 * (bwid + gx); y2 = by + bh + 90
d.rounded_rectangle([x, y2, x + bwid, y2 + bh], radius=16, fill=ACC)
d.text((x + 18, y2 + 18), 'load_detector', font=F(30, True), fill=(255, 255, 255))
d.multiline_text((x + 18, y2 + 70), '自前学習 YOLOv8n\nパンケーキ + tray 枠', font=F(26), fill=(255, 235, 235), spacing=8)
d.line([x + bwid / 2, by + bh, x + bwid / 2, y2], fill=SUB, width=4)
d.text((x + bwid / 2 + 12, by + bh + 20), 'tray 枠 ⇅ 外接円', font=F(26), fill=SUB)
d.line([x + bwid, y2 + bh / 2, bx + 3 * (bwid + gx) + bwid / 2, y2 + bh / 2], fill=SUB, width=4)
d.line([bx + 3 * (bwid + gx) + bwid / 2, y2 + bh / 2, bx + 3 * (bwid + gx) + bwid / 2, by + bh], fill=SUB, width=4)
d.text((x + bwid + 20, y2 + bh / 2 - 40), '「パンケーキが載った」判定 (JSON)', font=F(26), fill=SUB)
bullets(d, [('ハード:', 'SO-101 follower（STS3215 ×6、12 V 5 A）、改造グリッパ、OAK-D Pro を手首 35° ブラケットに取付、Jetson'),
            ('TF:', 'robot_state_publisher に URDF + カメラ取付（CAD から算出）を渡し、実測関節角でカメラ位置を計算'),
            ('安全:', 'シリアルバスの排他ロック、リセット検知で指令停止、controller は既定 dry_run')],
        70, 790, size=32)
footer(d); save(im, '04_system.png')

# 05 point1 depth
im, d = new(); header(d, '工夫①  トレイ表面の深度は読まず、周囲の台面に平面を当てる', '最大の工夫')
r = paste_fit(im, os.path.join(PHOTO, 'depth_tray_plane_ring_2026-10-26.png'), 70, 195, 1780, 440)
d.text((70, r[1] + r[3] + 14), '左: RGB　右: 位置合わせ深度（黒 = 欠損）。水色 = トレイの円、白 = 平面推定に使うリング（半径 1.6〜2.5 倍）', font=F(28), fill=SUB)
bullets(d, [('問題:', 'トレイ内側は深度の有効画素 45%、残りも横縞状の誤マッチ。D455 は 25 cm 未満で全欠損'),
            ('解決:', 'リング内の台面（有効 62%、揺れ 1 mm）に RANSAC で平面を当て、視線との交点を 3D 位置にする'),
            ('高さ:', '中心は縁 18 mm、取っ手は 32 mm の面で切る（台面で切ると約 1 cm ずれて把持を外した）'),
            ('接近中:', '深度は使わず、観測で記憶した座標と関節角の実測だけで動く')],
        70, 720, size=30, gap=14)
footer(d); save(im, '05_point1_plane.png')

# 06 point2 detection + yolo
im, d = new(); header(d, '工夫②  縁の円で中心を取り、パンケーキは自前学習の YOLOv8n で判定', '認識')
r = paste_fit(im, os.path.join(PHOTO, 'IMG_1790.jpg'), 70, 200, 1780, 470)
bullets(d, [('円検出:', 'Hough 円 + 色の整合で「縁」を直接捉える。赤マスクが三日月しか残らなくても中心がぶれない（赤輪郭の円フィットは 3〜5 cm ずれた）'),
            ('把持点:', '計測した取っ手の「向き」だけ使い、距離は CAD 値 108 mm に置き換える'),
            ('パンケーキ:', '115 枚で微調整した YOLOv8n（pancake / candy / cake / tray）。GPU 35〜40 ms。2 秒続けて「パンケーキが載った」で開始、把持直前にも再確認'),
            ('連携:', '学習モデルの tray 枠を円探索の範囲に渡す。ラップの反射やピンクのホットプレートでも見失わない')],
        70, 700, size=30)
footer(d); save(im, '06_point2_detect.png')

# 07 point3 precision & safety
im, d = new(); header(d, '工夫③  数度たわむサーボで数 mm を出す / 実機の安全', '制御')
d.text((70, 200), '精度', font=F(40, True), fill=ACC)
bullets(d, [('移動→実測→補正:', '実測角との差 × 0.8 を指令に上乗せ、最大 3 回。真上 9.9 cm に 2 mm で到達'),
            ('自前 IK:', 'URDF 直読み FK + SciPy least_squares。土台側へ 20〜50° 傾けた放射状接近'),
            ('ロール 2 段階:', '把持時は指の閉じ方向が取っ手と直交する角。配置時は記憶した取っ手ベクトルが手前向きになる角を符号込みで解き、上空で鉛直軸まわりに回す')],
        70, 260, size=30, width=1000)
d.text((1120, 200), '把持判定（閉じ角・負荷）', font=F(40, True), fill=ACC)
table(d, [['状態', '停止角', '負荷'], ['板を挟んだ', '約 8°', '約 200'], ['つまみを挟んだ', '約 18°', '約 500'], ['空振り → やり直し', '1°', '約 30']], 1120, 260, [360, 180, 180], size=28, rowh=54)
d.text((1120, 560), '安全', font=F(40, True), fill=ACC)
bullets(d, [('排他ロック:', 'バスに 2 つ目のプロセスは起動拒否（暴走事故の再発防止）'),
            ('リセット検知:', '読み取り失敗・リセットで指令停止。電源 12 V 5 A 化'),
            ('経路検査:', '41 分割で最低高さを確認、台面近くからの大移動前に 6 cm 上げる'),
            ('dry_run 既定:', '実機で動かすときだけ明示的に解除')],
        1120, 620, size=28, width=720)
footer(d); save(im, '07_point3_control.png')

# 08 results
im, d = new(); header(d, '結果', '結果')
table(d, [['項目', '結果'],
          ['トレイ中心の真上 9.9 cm への到達誤差', '2 mm（補正 3 回後）'],
          ['2 サイクル間の到達位置の差', '3〜5 mm'],
          ['トレイ中心の base 座標の再現性（5 姿勢）', '1〜3 cm（深度と平面フィットの揺れ）'],
          ['取っ手の把持 → 持ち上げ → 配置 → 復帰', '自律実行に成功（閉じ角 18°・負荷 500 で HELD）'],
          ['観測から配置完了までの時間', '約 1 分（関節速度 15°/s に制限）'],
          ['パンケーキ検出（微調整 YOLOv8n、GPU）', 'mAP50 pancake 0.995 / tray 0.995 / candy 0.37、35〜40 ms'],
          ['パンケーキ検出（YOLO-World ゼロショット、旧）', '確信度 0.1〜0.2、約 1 s。アメの名前は区別できず']],
      70, 200, [900, 880], size=30, rowh=64)
d.text((70, 760), '未達: 10 回規模の試行による成功率の統計。空振り時にトレイが 2〜3 cm 動く問題。', font=F(32), fill=SUB)
footer(d); save(im, '08_results.png')

# 09 AI usage
im, d = new(); header(d, 'AI の活用', 'AI')
d.text((70, 200), 'ロボット側の推論', font=F(40, True), fill=ACC)
bullets(d, [('ゼロショット → 微調整:', 'YOLO-World で仕組みを作り、弱点（アメの区別）を確認してから 115 枚で YOLOv8n を学習（Jetson GPU 6 分）'),
            ('使い分け:', '位置は決定論的な画像処理と IK、「パンケーキが載ったか」だけ学習モデル。限定環境では精度と原因究明のしやすさで有利')],
        70, 260, size=30)
d.text((70, 560), '開発プロセス', font=F(40, True), fill=ACC)
bullets(d, [('コーディングエージェント:', 'Jetson 上で Claude Code を動かし、ノード・URDF・launch の実装、ログからの原因究明、CAD 寸法の読み取り、安全ルールの提案を対話で進めた。約 2 週間で実機の自律動作'),
            ('勉強用資料:', '自分のプログラムを教材にした ROS 2 入門資料（題材 5 つ + 演習）を AI に書かせ、概念とコードを結びつけた')],
        70, 620, size=30)
footer(d); save(im, '09_ai.png')

# 10 reflection & future
im, d = new(); header(d, '反省と今後', 'まとめ')
d.text((70, 200), 'うまくいかなかったこと', font=F(40, True), fill=ACC)
bullets(d, [('近距離深度の限界:', '「深度カメラがあれば 3D が取れる」が崩れた。台が無地の平面であることに依存'),
            ('把持の左右非対称:', '固定爪の先端に板が当たると失敗。狙いを可動爪側へ、開き 70° に'),
            ('電源・USB・運用:', '2 A 電源でリセット、OAK が USB 2 に落ちて 640×360、プロセス二重起動'),
            ('成功率が未計測:', '10 回規模の統計を取るところまで至らず')],
        70, 260, size=29, width=880)
d.text((1000, 200), '今後', font=F(40, True), fill=ACC)
bullets(d, [('トレイの傾き:', '片持ち把持で傾く。把持点・爪形状・手首補正で抑える'),
            ('近接視覚フィードバック:', '把持直前に指と板の相対位置を画像で補正'),
            ('他工程との連携:', '工程①（メーカーを開ける）を同じ controller に、工程③の合図をトピックで受ける'),
            ('成功率の計測と向上:', '連続試行と失敗内訳ごとの対策、launch 一括起動'),
            ('一般化:', 'セグメンテーションで色・形の違う食器へ')],
        1000, 260, size=29, width=850)
footer(d); save(im, '10_future.png')

# 11 end
im, d = new()
d.rectangle([0, 0, W, H], fill=(28, 32, 40)); d.rectangle([0, 0, 18, H], fill=ACC)
d.text((90, 300), 'ご視聴ありがとうございました', font=F(70, True), fill=(255, 255, 255))
d.text((90, 440), 'コード・資料', font=F(36), fill=ACC2)
d.text((90, 500), 'https://github.com/droplet-of-star/pancake_2026_physicalAI_ver', font=F(40), fill=(255, 255, 255))
d.text((90, 600), 'README / docs/説明資料_PAI最終課題.md / docs/システム解説 / docs/ROS2学習資料', font=F(30), fill=(200, 204, 210))
d.text((90, 760), '使用 OSS: SO-ARM100 (Apache-2.0), Lerobot_ros2 (MIT), LeRobot (Apache-2.0), ROS 2 Humble, OpenCV, depthai (MIT),', font=F(26), fill=(170, 174, 180))
d.text((90, 800), 'PyTorch (BSD-3), ultralytics YOLOv8 (AGPL-3.0)', font=F(26), fill=(170, 174, 180))
save(im, '11_end.png')

# ---- telops (transparent lower thirds) ----
def telop(name, main, sub=None, badge=None):
    im = Image.new('RGBA', (W, H), (0, 0, 0, 0)); d = ImageDraw.Draw(im)
    bh = 150 if sub else 110; y0 = H - bh - 40
    d.rectangle([0, y0, W, y0 + bh], fill=(20, 24, 30, 200)); d.rectangle([0, y0, 18, y0 + bh], fill=ACC + (255,))
    d.text((60, y0 + 18), main, font=F(52, True), fill=(255, 255, 255, 255))
    if sub: d.text((60, y0 + 90), sub, font=F(30), fill=(220, 222, 226, 255))
    if badge:
        f = F(34, True); tw = d.textlength(badge, font=f)
        d.rounded_rectangle([W - 60 - tw - 40, 40, W - 60, 40 + 64], radius=12, fill=ACC + (235,))
        d.text((W - 60 - tw - 20, 40 + 10), badge, font=f, fill=(255, 255, 255, 255))
    p = os.path.join(OUT, 'telops', name); im.save(p, optimize=True); print('saved', p)
telop('t01_observe.png', '① 観測', '手首カメラでトレイを探す。見えなければ肩を振って探索')
telop('t02_load_check.png', '② パンケーキの確認【AI】', '自前学習の YOLOv8n が「パンケーキが載った」と 2 秒続けて判定したら開始')
telop('t03_estimate.png', '③ 取っ手の 3D 推定', '周囲の台面に平面を当て、視線との交点で位置を出す（トレイ表面の深度は使わない）')
telop('t04_approach_grasp.png', '④ 接近・把持', '土台側へ 20〜50° 傾けて放射状に接近。移動 → 実測 → 補正を最大 3 回')
telop('t05_grasp_check.png', '⑤ 把持判定', '閉じ角と負荷で判定。空振りなら開いて観測からやり直す')
telop('t06_lift.png', '⑥ 奥へ 2 cm → 持ち上げ', '取っ手を押し込んでから 6 cm 持ち上げる')
telop('t07_place.png', '⑦ 配置', '上空で工具軸を鉛直にしてから回し、取っ手を手前向きにして置く')
telop('t08_release_home.png', '⑧ 開放・観測姿勢へ復帰', None)
telop('t09_retry.png', '空振り → やり直し', '把持判定で失敗を検知し、観測からやり直す')
# speed badges
for n, b in (('badge_2x.png', '2 倍速'), ('badge_4x.png', '4 倍速'), ('badge_realtime.png', '等速')):
    im = Image.new('RGBA', (W, H), (0, 0, 0, 0)); d = ImageDraw.Draw(im); f = F(34, True); tw = d.textlength(b, font=f)
    d.rounded_rectangle([W - 60 - tw - 40, 40, W - 60, 40 + 64], radius=12, fill=ACC + (235,)); d.text((W - 60 - tw - 20, 50), b, font=f, fill=(255, 255, 255, 255))
    im.save(os.path.join(OUT, 'telops', n)); print('saved', n)
