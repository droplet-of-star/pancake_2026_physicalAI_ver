# 2026 physicalAI 応用編1 最終課題 — SO-101 × OAK-D Pro 　パンケーキロボット　

手首カメラ付きの SO-101 アーム（Feetech STS3215 ×6）で、作業台上の赤いトレイ（直径 150 mm、取っ手付き）を見つけ、
取っ手を掴んで持ち上げ、指定位置に取っ手を手前向きにして置く ROS 2 Humble システム。Jetson で動作。

## デモ動画

[![デモ動画（YouTube）](https://img.youtube.com/vi/79h9IVhPGWI/maxresdefault.jpg)](https://youtu.be/79h9IVhPGWI)

クリックで YouTube が開く。パンケーキが載ったことを検出してから、取っ手を掴んで持ち上げ、取っ手を手前向きにして置くまでの一連の動作。

- 1 コマンド `pick_place` で、観測 → 把持（空振りなら観測からやり直し）→ 奥へ 2 cm → 持ち上げ → 配置 → 開放 → 観測姿勢 まで自動実行（2026-10-24 実機成功）。
- `auto` コマンドは、**学習済み物体検出（自前データで微調整した YOLOv8n）でトレイにパンケーキが載ったことを確認してから** pick & place を始める（2026-10-25 追加、2026-10-26 にモデルを自前学習に更新）。
- 動画（最終課題の提出用）: https://youtu.be/79h9IVhPGWI （上のサムネイルからも開ける）
- 最終課題の説明資料: [docs/説明資料_PAI最終課題.md](docs/説明資料_PAI最終課題.md)
- 技術解説: [docs/システム解説_2026-10-25.md](docs/システム解説_2026-10-25.md)
- ROS 2 学習資料（本プログラムを題材にした入門）: [docs/ROS2学習資料_SO101_PickPlace.md](docs/ROS2学習資料_SO101_PickPlace.md)
- 開発経緯・計測記録: [docs/環境確認結果_2026-10-13.md](docs/環境確認結果_2026-10-13.md)
- 当初の計画: [SO101_RealSense_Pick_and_Place_実装計画.md](SO101_RealSense_Pick_and_Place_実装計画.md)
- ライセンス: [LICENSE](LICENSE)（All rights reserved。無断での複製・改変・再配布・組み込みを許可しない）、第三者表示 [THIRD_PARTY_NOTICES.md](THIRD_PARTY_NOTICES.md)

## パンケーキ焼きロボットの中での役割

本プログラムは、複数のロボットで分担する**パンケーキ焼きロボット**の最終工程「お客さんに差し出す」を担当する。
全体の流れは次の 4 工程で、SO-101 は 1 と 4 を受け持つ。

| 工程 | 担当 | 内容 |
|---|---|---|
| 1. パンケーキメーカーを開ける | SO-101 | 焼き上がったパンケーキメーカーのふたを開ける |
| 2. 焼印を押す | 焼印装置 | 鉄板上のパンケーキに焼印（ネコの顔）を押す |
| 3. お皿に乗せる | 別ロボット | 焼き上がったパンケーキを取っ手付きトレイに移す |
| 4. お客さんに差し出す | **SO-101（本プログラム）** | トレイにパンケーキが載ったことを物体検出で確認し、取っ手を掴んで持ち上げ、お客さんの前に取っ手を手前向きにして置く |

<table>
<tr>
<td align="center"><img src="docs/photo/パンケーキメーカーを開ける.png" width="300"><br>1. パンケーキメーカーを開ける</td>
<td align="center"><img src="docs/photo/焼印をおす.png" width="300"><br>2. 焼印を押す</td>
<td align="center"><img src="docs/photo/パンケーキをお皿に置く(仮).jpg" width="300"><br>3. お皿に乗せる（別ロボット、仮）</td>
<td align="center"><img src="docs/photo/お客さんに差し出す.png" width="300"><br>4. お客さんに差し出す</td>
</tr>
</table>

本リポジトリが扱うのは工程 4（`auto` コマンド: パンケーキ検出 → pick & place）で、工程 1〜3 の装置・ロボットの制御は含まない。

### 動作中の画面

<img src="docs/photo/IMG_1790.jpg" width="800">

2026-10-26 の実機画面。左は色ベースの検出ノード（緑 = 赤マスクの輪郭、水色の円 = Hough 円で捉えたトレイの縁、黄色の × = 取っ手の把持画素）、
右は学習モデルのパンケーキ検出（水色 = `tray 0.86`、緑 = `pancake 0.92`、橙の円の内側にパンケーキがあるので LOADED）、
下は controller のログ（取っ手の計測距離 10.4 cm を CAD 値 10.8 cm に置き換えて把持点を決め、TCP 高さと開き角 70° を設定）。
パンケーキとラップの反射で赤マスクが大きく欠けても、縁の円と学習モデルの枠でトレイを捉えている。

<img src="docs/photo/depth_tray_plane_ring_2026-10-26.png" width="800">

同じ観測姿勢の RGB と位置合わせ深度。トレイの内側は深度の有効画素が 45% で縞状に乱れる（IR ドットの誤マッチ）が、白いリング（1.6r〜2.5r）の台面は 62% が有効で揺れ 1 mm。
そのため 3D 推定はトレイ表面の深度を使わず、リング内の点に平面を当てて視線との交点を取る。

## 構成

```
OAK-D Pro ──(RGB / 位置合わせ深度 / CameraInfo)──▶ object_detector ──▶ object_pose_estimator ──▶ pick_place_controller ──▶ so101_motor_bridge ──▶ サーボ
                                                     (HSV + 円フィット)   (台平面 × 視線 → base_link)   (FK/IK・状態機械・安全)   (Feetech バス)
                                                                                 ▲ /tf                          ▲ /robot_description
                                                                            robot_state_publisher（URDF + カメラ取付）
```

| ディレクトリ | 内容 |
|---|---|
| `so101_pick_place/` | ROS 2 パッケージ（ament_python）。`~/ros2_ws/src/so101_pick_place` へシンボリックリンクして使う |
| `so101_pick_place/so101_pick_place/` | `oak_camera_node.py`, `object_detector.py`, `object_pose_estimator.py`, `load_detector.py`（YOLO パンケーキ検出）, `kinematics.py`, `pick_place_controller.py`, `calibrate_zero.py` |
| `so101_pick_place/config/` | `params.yaml`（全ノードのパラメータ）、`camera_mount.yaml`（カメラ取付） |
| `so101_pick_place/launch/` | `robot_tf.launch.py`（TF）、`pick_place.launch.py`（一式） |
| `so101_pick_place/urdf/` | SO-101 URDF にカメラを合成する xacro |
| `scripts/` | 環境読込 `setup_env.sh`、環境確認 `check_env.sh`、sudo 作業 `install_sudo_deps.sh`、再ゼロ化・保持テスト等 |
| `udev/` | USB シリアル（SO-101）と OAK の udev ルール |
| `docs/` | 解説、記録、ライセンス、CAD（`cad/`）、画像、ログ |

## ハードウェア

<img src="docs/photo/IMG_1791.jpg" width="600">

- SO-101 follower（CH343 USB シリアル、シリアル番号 5AB9068796）。交換グリッパ v10（固定爪先端 64 mm、可動爪 39 mm）。
- OAK-D Pro を手首の 35° ブラケットに取付（取付値は `config/camera_mount.yaml`）。
- サーボ電源 12 V 5 A 以上（2 A では負荷姿勢で電圧低下 → リセット）。
- Jetson、JetPack 6（L4T R36.5.2）、Ubuntu 22.04、ROS 2 Humble、Python 3.10。

## セットアップ

前提: `~/ros2_ws/src/Lerobot_ros2`（SO-101 の既存パッケージ。`so101_hw_interface/motor_bridge.py` は改変済み。詳細は THIRD_PARTY_NOTICES.md）。

```bash
# 1. sudo が必要な作業（udev ルール、apt 依存）
bash scripts/install_sudo_deps.sh

# 2. pip 依存（ユーザー領域）
pip install --user depthai==2.33.0.0

# 3. パッケージのリンクとビルド
ln -s "$(pwd)/so101_pick_place" ~/ros2_ws/src/so101_pick_place
cd ~/ros2_ws && colcon build --symlink-install --packages-select so101_pick_place

# 4. 環境の確認
source scripts/setup_env.sh
bash scripts/check_env.sh
```

キャリブレーション（URDF ゼロ姿勢基準）: `ros2 run so101_pick_place so101_calibrate_zero` → `~/.so101_follower_calibration.yaml`。

### パンケーキ検出（YOLO）の環境

Jetson の apt numpy 1.21 と pip の PyTorch は BLAS シンボルで衝突するため、PyTorch と ultralytics は専用 venv に隔離する。
`scripts/run_load_detector.sh` は GPU 版 venv `~/.venvs/yolo-gpu` があればそれを、無ければ CPU 版 `~/.venvs/yolo` を使う。

CPU 版（GPU 版が作れないときの予備。推論は約 0.6 s/枚）:

```bash
python3 -m venv --system-site-packages ~/.venvs/yolo
~/.venvs/yolo/bin/pip install torch torchvision --index-url https://download.pytorch.org/whl/cpu
~/.venvs/yolo/bin/pip install -U setuptools "sympy>=1.13.3" typing_extensions filelock fsspec networkx ultralytics py-cpuinfo ultralytics-thop
```

**GPU 版（推奨、2026-10-25 に動作確認）**: Jetson 用の torch は PyPI ではなく Jetson AI Lab の配布先から、**バージョンを固定し PyPI を併用せずに**入れる（併用すると CUDA 13 用の PyPI 版が選ばれて動かない）。cuDSS は PyPI の aarch64 wheel で補う。

```bash
python3 -m venv --system-site-packages --without-pip ~/.venvs/yolo-gpu      # python3-venv 未導入のため --without-pip
curl -sS https://bootstrap.pypa.io/get-pip.py | ~/.venvs/yolo-gpu/bin/python -
~/.venvs/yolo-gpu/bin/pip install -U setuptools
~/.venvs/yolo-gpu/bin/pip install --no-deps --index-url https://pypi.jetson-ai-lab.io/jp6/cu126 "torch==2.11.0" "torchvision==0.26.0"
~/.venvs/yolo-gpu/bin/pip install --ignore-installed "numpy>=2,<3" "matplotlib>=3.8" "pandas>=2.0" "scipy>=1.11" pillow typing_extensions filelock fsspec "sympy>=1.13.3" networkx jinja2
~/.venvs/yolo-gpu/bin/pip install --no-deps "nvidia-cudss-cu12==0.5.0.16"
~/.venvs/yolo-gpu/bin/pip install ultralytics py-cpuinfo ultralytics-thop
LD_LIBRARY_PATH=~/.venvs/yolo-gpu/lib/python3.10/site-packages/nvidia/cu12/lib ~/.venvs/yolo-gpu/bin/python -c "import torch; print(torch.cuda.is_available(), torch.cuda.get_device_name(0))"
```

`scripts/run_load_detector.sh` と `scripts/yolo_env.sh` は GPU 版 venv があれば自動でそれを使い、`LD_LIBRARY_PATH` も通す。CPU 版だけ使うときは `YOLO_VENV=~/.venvs/yolo` を付ける。

モデルは `~/.so101_models/sweets_v2.pt`（YOLOv8n を自前データ 115 枚で微調整。クラス pancake / candy / cake / tray）。
ultralytics と YOLO の重みは **AGPL-3.0** なので、公開リポジトリでの利用は問題ないが、非公開の製品に組み込む場合はライセンス条件を確認すること（THIRD_PARTY_NOTICES.md）。

### パンケーキ検出モデルの学習（やり直すとき）

```bash
python3 scripts/capture_training_images.py --tag pancake            # OAK の画像を ~/.so101_models/dataset_raw/session_YYYYMMDD/images に保存
python3 scripts/downscale_images.py <dir>                            # スマホ写真を混ぜるときは長辺 640 に縮小
python3 scripts/annotate_boxes.py <dir> --classes pancake candy cake tray   # 矩形アノテーション（YOLO 形式、tray は円盤だけを囲む）
python3 scripts/build_yolo_dataset.py <dir> --out ~/.so101_models/datasets/sweets_v3   # train/val 分割と data.yaml
source scripts/yolo_env.sh
yolo detect train model=yolov8n.pt data=~/.so101_models/datasets/sweets_v3/data.yaml epochs=60 imgsz=640 batch=16 device=0 \
  project=~/.so101_models/runs name=sweets_v3                        # sweets_v2 の学習引数（runs/sweets_v2/args.yaml）と同じ
cp ~/.so101_models/runs/sweets_v3/weights/best.pt ~/.so101_models/sweets_v3.pt   # params.yaml の load_detector.model を差し替える
```

## 実行

端末ごとに `source ~/vscode/muran2026_physicalAI_ver/scripts/setup_env.sh` を先に実行する。

```bash
# 端末1: モーターブリッジ（起動時は動かない hold。アームを支えてから。Ctrl+C でトルクが切れる）
ros2 run so101_hw_interface so101_motor_bridge --ros-args \
  -p port:=/dev/serial/by-id/usb-1a86_USB_Single_Serial_5AB9068796-if00 \
  -p calib_file:=$HOME/.so101_follower_calibration.yaml -p startup_pose:=hold -p p_coefficient:=32

# 端末2: TF
ros2 launch so101_pick_place robot_tf.launch.py

# 端末3: カメラ・検出・推定・制御（USB 3 なら 1280x720 既定。USB 2 のときは下の 640x360 版）
P=~/ros2_ws/install/so101_pick_place/share/so101_pick_place/config/params.yaml
ros2 run so101_pick_place oak_camera --ros-args --params-file $P -p width:=640 -p height:=360 &
ros2 run so101_pick_place object_detector --ros-args --params-file $P -p process_scale:=1.0 -p min_area_px:=800 -p border_margin_px:=4 &
ros2 run so101_pick_place object_pose_estimator --ros-args --params-file $P -p plane_stride:=2 -p plane_min_points:=400 &
ros2 run so101_pick_place pick_place_controller --ros-args --params-file $P -p dry_run:=false &
bash scripts/run_load_detector.sh          # パンケーキ検出（専用 venv で起動）

# 端末4: 状態表示と実行
ros2 topic echo /so101_pick_place/status
ros2 topic pub --once /so101_pick_place/cmd std_msgs/String "{data: auto}"         # 載ったのを確認してから pick & place
ros2 topic pub --once /so101_pick_place/cmd std_msgs/String "{data: pick_place}"   # すぐに pick & place
```

コマンド（`/so101_pick_place/cmd`, std_msgs/String）: `observe` `plan` `hover` `pregrasp` `descend` `grasp` `lift` `place` `pick_place` `wait_load` `auto` `auto_loop` `open` `close` `home` `stop`。

- `wait_load`: 観測姿勢でトレイを見つけ、`/so101_pick_place/tray_load`（YOLO の判定）が 2 秒続けて「載っている」になるまで待つ。
- `auto`: `wait_load` → `pick_place`。`auto_loop`: それを `stop` まで繰り返す。
- 検出結果の画像: `ros2 run rqt_image_view rqt_image_view /so101_pick_place/load_debug_image`

トレイは土台の前方 25〜33 cm に置く。正面で見えなければ肩旋回を振って探す（±60°）。`dry_run` は既定 true で、実機を動かすときだけ false にする。

## 主なパラメータ（`so101_pick_place/config/params.yaml`）

| 項目 | 既定 | 意味 |
|---|---|---|
| `observation_pose_deg` | [0, -8, -49, 92, -88.6] | 観測姿勢（カメラ高さ 0.36 m、前方 36 cm を見下ろす） |
| `observe_pan_search_deg` / `observe_pan_gain` | [0, -30, 30] / 1.5 | 見えないときの旋回探索。見切れの位置から修正量を計算 |
| `grasp_height_m` / `tcp_offset_m` / `tcp_lateral_m` | 0.020 / 0.064 / 0.008 | 指先高さ（台面＋）、指先長さ、閉じ方向の狙いオフセット |
| `gripper_open_deg` / `gripper_close_deg` | 60 / 1 | 開閉角。把持判定は閉じ角 ≥ 3° かつ負荷 ≥ 100 |
| `post_grasp_shift_xy` | [0, -0.02] | 把持後、持ち上げ前に奥へ 2 cm |
| `place_position_xyz` | [0.05, -0.30, —] | 置き位置（把持点の絶対座標）。+x 左、-y 奥。z は台面高さ |
| `place_handle_axis_xy` | [0, 1] | 取っ手を必ず手前（ロボット側）に向けて置く |
| `handle_height_m`（推定） | 0.032 | 取っ手の板の高さ。把持画素の視線をこの高さで切る |
| `model` / `conf` / `imgsz`（パンケーキ検出） | sweets_v2.pt / 0.08 / 416 | 自前データで微調整した YOLOv8n（pancake / candy / cake / tray）。tray 枠はトレイ検出の探索範囲に使う。GPU で約 40 ms/推論 |
| `rim_edge_*`（検出） | in_min 0.4 / contain_min 0.55 | Hough 円 + 色整合でトレイ外周を求める（ラップ・パンケーキに強い）。解けなければ赤輪郭の RANSAC 円 |
| `handle_grasp_radius_m` / `table_z_m`（制御） | 0.108 / 0.017 | 把持点 = 中心 + CAD 距離 × 計測した向き。台面高さは固定値 |
| `load_confirm_s`（制御） | 2.0 | 「載っている」がこの秒数続いたら開始 |
| `base_z_min_m` / `base_z_max_m`（推定） | -0.10 / 0.30 | 床などの誤検出を捨てる高さ範囲 |

## 安全上の注意

- シリアルバスに接続するプログラムは常に 1 つ（ブリッジとキャリブレータを同時に動かさない。`bus_lock.py` が二重起動を拒否）。
- ブリッジを止めるとトルクが切れて脱力する。止める前にアームを支える。
- サーボ内部の位置制限（肩 -11°、旋回 +65°）は安全層として残し、IK 側で範囲を制限している。
- 近距離の深度は信用しない（OAK の IR ドット誤マッチ、D455 の 25 cm 未満欠損）。3D 計測は観測姿勢からのみ。

