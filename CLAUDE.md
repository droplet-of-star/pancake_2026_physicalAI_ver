# muran2026 physicalAI: SO-101 × RealSense Pick & Place

計画書: `SO101_RealSense_Pick_and_Place_実装計画.md`。環境確認の記録: `docs/環境確認結果_2026-10-13.md`。システム解説: `docs/システム解説_2026-10-25.md`。概要と起動手順: `README.md`。ライセンス: 自作部分は All rights reserved（2026-10-26、MIT から変更。`LICENSE` の著作権者名は未記入）。第三者表示は `THIRD_PARTY_NOTICES.md`（Lerobot_ros2 は LICENSE ファイル無し）。ultralytics（AGPL）を使うため public 配布との整合に注意。

## 環境の要点
- Jetson AGX Orin (JetPack 6 / L4T R36.5.2), Ubuntu 22.04, ROS 2 Humble, Python 3.10, OpenCV 4.5.4 (apt)。
- ROS ワークスペースは `~/ros2_ws`。`src/Lerobot_ros2` に既存の SO-101 パッケージ
  (`so101_hw_interface`, `so101_follower_description`, `so101_follower_moveit`) がある。
- 本リポジトリの `so101_pick_place/` は `~/ros2_ws/src/so101_pick_place` へシンボリックリンクされている。
  ビルド: `cd ~/ros2_ws && colcon build --symlink-install --packages-select so101_pick_place`
- 環境の読み込み: `source scripts/setup_env.sh`。環境確認: `bash scripts/check_env.sh`。
- sudo が必要な作業は `scripts/install_sudo_deps.sh` にまとめてある（Claude は sudo を実行できない）。

## ハードウェア
- SO-101 follower: CH343 USB シリアル、シリアル番号 `5AB9068796`（udev 適用後は `/dev/usbserial_lerobot_follower`、未適用時は `/dev/serial/by-id/usb-1a86_USB_Single_Serial_5AB9068796-if00`）。
  `/dev/ttyACM0` の番号は差し順で変わるので、ハードコードしない。
- もう 1 本のシリアル `5AB9068794` は leader と推定（未確認）。
- キャリブレーション: `~/.so101_follower_calibration.yaml`（2026-10-13 生成）。
- **2026-10-21 にカメラを OAK-D Pro に変更。** ドライバは pip depthai 2.33 + 自前ノード `oak_camera`（`ros2 run so101_pick_place oak_camera`）。トピック名は D455 と同じ。udev `udev/80-movidius.rules` が必要。取付値は CAD+穴位置から算出済み（±5 mm）。
- **OAK の IR ドットは近距離で誤マッチを起こす**ので、3D 推定は作業台平面方式（周囲の深度に平面を当て、視線との交点を取る。表面深度は使わない）。800P はファームウェアクラッシュあり、400P 固定。色順は BGR。
- （旧）RealSense は **D455**（シリアル 241122301703、FW 5.13.0.55）。`ros2 launch realsense2_camera rs_launch.py align_depth.enable:=true enable_sync:=true` で `/camera/camera/{color/image_raw, aligned_depth_to_color/image_raw, color/camera_info}` が出る（1280x720、深度は 16UC1 mm）。STEP 1 完了（2026-10-13）。STEP 2（HSV 赤色検出、`object_detector.py`）完了（2026-10-20）。対象は赤いフライパン型のお皿で、既定しきい値で検出できる。STEP 3（深度→カメラ座標 XYZ、`object_pose_estimator.py`）実装済み（2026-10-20）。
- STEP 4 ソフト側（`urdf/so101_with_camera.urdf.xacro`, `launch/robot_tf.launch.py`, `config/camera_mount.yaml`）完了（2026-10-20）。取付値は CAD（`docs/cad/realsense_d455_mount_35deg.usdz`）から算出済み（`gripper_link`→ねじ穴 xyz (-0.001, 0.037, -0.0212) rpy (π, 0.96, -π/2)）。`use_fake_joints:=true` でアーム無しに TF を確認できる。
- **D455 は約 25 cm 未満で深度が全欠損する**（解像度を下げても改善せず）。3D 計測はお皿から 35 cm 以上離れた観測姿勢で行い、接近中は深度に頼らない。

## 安全ルール
- **シリアルバスにつなぐプログラムは常に 1 つだけ**（bridge と calibrator を同時に動かさない）。2026-10-20 に同時起動でアームが暴走した。`bus_lock.py` が二重起動を拒否する。
- bridge を Ctrl+C するとトルクが切れて脱力する。止める前にアームを支えるか寝かせる。
- **2026-10-26 午後**: 載せ物検出モデルを 4 クラス（pancake/candy/cake/**tray**）の `sweets_v2.pt` に更新。`load_detector` は画像全体で推論し `/so101_pick_place/tray_bbox` を出す。`object_detector` はその枠（円盤だけを囲む約束）を円探索 ROI にする。推定はトレイ中心を縁の高さ `plate_height_m 0.018` で切って台面に落とし、平面候補は最も低いものを選ぶ（`plane_prefer_lowest`）、深度バッファ 45 枚。controller は把持点を「中心 + `handle_grasp_radius_m 0.108` × 計測した向き」で決め、台面は固定 `table_z_m 0.017`（`table_z_use_observed false`）。学習データ `~/.so101_models/dataset_raw/session_20261025`（115 枚）、アノテーションは `scripts/annotate_boxes.py`、学習は `~/.venvs/yolo-gpu`。
- **トレイ検出は rim_edge（2026-10-26）**: Hough 円 + 色整合（f_in ≥ 0.4、内包率 ≥ 0.55、支持率で選択）。ラップの反射やパンケーキで赤マスクが欠けても外周を捉える。赤輪郭の円フィット（rim_circle）は載せ物の縁に引きずられ中心が 3〜5 cm ずれて把持を外した。controller は取っ手点とトレイ中心の距離 8.5〜14 cm 外を棄却。
- **載せ物検出（2026-10-25）**: `load_detector`（ultralytics YOLO）は **専用 venv** で `bash scripts/run_load_detector.sh` から起動する（apt numpy と pip torch が衝突するため。ユーザーサイトに torch/setuptools を入れると numpy と colcon が壊れる）。モデルは `sweets_v2.pt`（上記）。初期の YOLO-World（オープン語彙）経路と色フォールバックは 2026-10-26 のリファクタで削除した。controller の `auto` = 載ったのを 2 s 確認 → pick_place。ultralytics は AGPL-3.0。**GPU 版 venv `~/.venvs/yolo-gpu`**（torch 2.11.0+CUDA 12.6、Jetson AI Lab index から `--no-deps` で固定インストール、cuDSS は `nvidia-cudss-cu12==0.5.0.16`、`LD_LIBRARY_PATH` に `nvidia/cu12/lib`）。PyPI を併用すると CUDA 13 版が選ばれて失敗する。ラッパーは GPU venv を自動選択。
- **2026-10-25 の拡張**: 置き位置は絶対座標（`place_position_xyz [0.05,-0.30]` = 左 5 cm・奥 30 cm、把持点の位置）、取っ手は必ず手前向き（`place_handle_mode fixed` + `solve_roll_signed`、上空で鉛直軸まわりに回す）、把持後に奥へ 2 cm（`post_grasp_shift_xy`）。観測で見えなければ肩旋回を見切れヒントから連続探索（±60°、`~/.so101_pick_place_last_pan` に保存）。取っ手の視線は板高さ `handle_height_m 0.032` の面で切る（台面で切ると約 1 cm ずれる）。`tcp_lateral_m 0.008`。床の赤い物対策に高さゲート（estimator `base_z_min/max_m`、controller `table_z_tolerance_m`）。**ノードの再起動は pid で子プロセスまで止める**（`ros2 run` のラッパーだけ kill すると controller が二重になる。`pkill -f` は自分のシェルに当たる）。
- **STEP 6 自律一括 SUCCESS（2026-10-24 17:xx）**: `pick_place` で観測→把持（判定 HELD: 18°/負荷 500）→左奥 3/4 cm へ配置→home。把持判定とリトライ、相対配置を実装済み。
- **STEP 6 一連成功（2026-10-24）**: observe → grasp（取っ手）→ lift → place → home が実機で通った。検出は rim_circle（縁の円フィット）、把持は tcp_offset 0.064 / tcp_lateral -0.004 / 指先高さ 台面+2 cm / 開 60° 閉 1°（新グリッパ v10）。把持成功の指標: 閉じ角 ≈8° で停止・負荷 ≈200。OAK は現在 USB 2 認識のため 640x360 で運用（`-p width:=640 -p height:=360`、検出 `process_scale 1.0`、推定 `plane_stride 2 plane_min_points 400`）。
- **STEP 5 完了（2026-10-22 01:3x）**: 2 サイクルで到達 TCP の差 3〜5 mm。平面選択は最多 inlier + リング深度ゲート（最近傍優先は廃止）。tcp_offset 0.095（ハンド 1 cm 短縮）。
- **STEP 5 上方到達に成功（2026-10-22 01:0x）**: `pick_place_controller` の observe → plan → hover で、お皿中心の真上 9.9 cm に実測 2 mm 精度で到達。観測姿勢 [0,-8,-49,92,-88.6]°（サーボ内部制限 lift ≥ -11° の範囲内）。コマンドは `/so101_pick_place/cmd` に observe/plan/hover/home/stop。運動学は `kinematics.py`（URDF 直読み、scipy IK）。
- **STEP 4 完了（2026-10-22）**: 全 direction 正。5 姿勢で base 座標の差 1〜3 cm（残差は OAK 深度/平面フィットの揺れ）。電源は 5 A 以上に交換済み。bridge は `-p p_coefficient:=32` 推奨（元は 16）。サーボの数度の偏差・たわみは前提として受け入れ、controller 側で移動→実測→補正する。`/so101_pick_place/object_point_base_filtered` を使う。
- **サーボ電源が負荷で落ちる疑い**（2026-10-21 に 2 回、モータリセットの痕跡）。bridge は連続読み取り失敗/リセット検知で指令停止するようにした。電源容量・コネクタの確認が未完。bridge の `acceleration` 既定 40。
- 新しいキャリブレーションは `ros2 run so101_pick_place so101_calibrate_zero`（URDF ゼロ姿勢基準）。bridge は `-p startup_pose:=hold` で起動すると起動時に動かない。
- `so101_motor_bridge` は起動直後に全関節を可動範囲中央へ動かす。アームの周囲を確認してから起動すること。Claude は実機を動かすコマンドを自動実行しない。
- `pick_place_controller` の `dry_run` は既定 true。実機で動かすときだけ明示的に false にする。

## 既知の注意点（既存 Lerobot_ros2）
- URDF の関節名は `shoulder_pan, shoulder_lift, elbow_flex, wrist_flex, wrist_roll, gripper`。リンクは `base_link ... wrist_link, gripper_link, jaw_link`。
- `so101_follower_moveit/config/ros2_controllers.yaml` の関節名 (`wrist_pitch, wrist_yaw, gripper_jaw`) は URDF と不一致。MoveIt 実機接続時に修正が必要。
- `src/topic_based_ros2_control` は空ディレクトリ（サブモジュール未取得）。apt の `ros-humble-topic-based-ros2-control` で代替予定。
