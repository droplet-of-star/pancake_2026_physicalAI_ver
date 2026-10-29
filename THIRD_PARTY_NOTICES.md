# Third-Party Notices / 第三者ソフトウェア・設計データの表示

本リポジトリ（`so101_pick_place` ほか）の自作部分は All rights reserved（`LICENSE`。他者への利用許諾なし）とする。
本プロジェクトが利用・改変・派生している第三者の成果物と、その表示義務を以下にまとめる。
ライセンス全文は `LICENSES/` に置く。詳しい確認経緯は手元の記録（リポジトリには含めない）にある。

## 1. 本リポジトリに含まれる、または本リポジトリが派生している成果物

### SO-ARM100 / SO-101 (TheRobotStudio) — Apache License 2.0
- https://github.com/TheRobotStudio/SO-ARM100 （LICENSE: Apache-2.0。2026-10-25 に GitHub で確認）
- 利用: SO-101 の機体、URDF/メッシュ（`Lerobot_ros2/so101_follower_description` 経由で `include`）、
  交換グリッパ `docs/cad/SO101 Assembly_Wrist_Roll_Follower_SO101 v10.usdz` と `docs/cad/Moving_Jaw_SO101.usdz`
  は SO-101 の設計を改変した派生物。
- 表示: 本ファイルおよび `LICENSES/Apache-2.0.txt`。改変の旨: 「グリッパ本体と可動爪を本プロジェクトで短縮・改変（2026-10）」。

### Lerobot_ros2 (AgRoboticsResearch) — README 記載では MIT License（LICENSE ファイル未同梱）
- https://github.com/AgRoboticsResearch/Lerobot_ros2 （README: "This project is licensed under the MIT License"。
  2026-10-25 時点でリポジトリに LICENSE ファイルは無く、`so101_hw_interface` / `so101_follower_description` の
  `package.xml` は "TODO: License declaration"）
- 利用: `so101_hw_interface`（モーターブリッジ `motor_bridge.py`、キャリブレーション `calibrate_arm.py`）、
  `so101_follower_description`（URDF/メッシュ）、`so101_follower_moveit`（BSD、現在未使用）。
  いずれも `~/ros2_ws/src/Lerobot_ros2` に置いて使用し、本リポジトリにはコピーしていない。
- 本プロジェクトによる改変（`~/ros2_ws/src/Lerobot_ros2/src/so101_hw_interface/so101_hw_interface/`）:
  - `motor_bridge.py`: 2026-10-20〜25 改変。起動姿勢 `startup_pose`（mid/hold/zero）、キャリブ yaml の direction・
    エンコーダまたぎ対応、バス排他ロック、連続読み取り失敗/リセット検知での指令停止、acceleration・P/I/D パラメータ、
    起動時レジスタ表示、`joint_states.effort` に Present_Load。原本は `motor_bridge.py.orig_2026-10-20`。
  - `calibrate_arm.py`: バス排他ロックを追加。
  - `bus_lock.py`: 本プロジェクトで新規作成（MIT）。
- 表示: 原作者の著作権表示を保持する。公開・納品時は作者に LICENSE 追加を依頼するか、該当部分を置き換える
  （手元の確認記録の To-Do 参照）。

### LeRobot (Hugging Face) — Apache License 2.0
- Copyright 2024 The HuggingFace Inc. team. All rights reserved.
- https://github.com/huggingface/lerobot
- 利用: `so101_hw_interface/motors/`（`motors_bus.py`, `feetech/feetech.py`, `feetech/tables.py`）と
  `so101_hw_interface/lerobot/` に Lerobot_ros2 がベンダリングしたモータ通信コード。本プロジェクトは無改変で使用。
- 表示: 上記著作権表示と `LICENSES/Apache-2.0.txt`。NOTICE ファイルは同梱されていない。

### Intel RealSense ROS (`realsense2_description`) — Apache License 2.0
- Copyright Intel Corporation / LibRealSense ROS Team
- 利用: 開発初期（2026-10-13〜20、D455 使用時）に `urdf/so101_with_camera.urdf.xacro` が D455 の xacro マクロを `include` していた。
  2026-10-26 のリファクタで分岐を削除し、現在は参照していない。コピーなし。

## 2. 実行時に使用するライブラリ（本リポジトリには含まれない）

### 載せ物検出（`load_detector`、専用 venv `~/.venvs/yolo-gpu` / `~/.venvs/yolo`）— **AGPL-3.0 を含む**

| コンポーネント | 著作権者 | ライセンス | 備考 |
|---|---|---|---|
| ultralytics（YOLOv8 実装、`yolov8n.pt` 重みから微調整した `sweets_v2.pt`） | Ultralytics Inc. | **AGPL-3.0** | 2026-10-26 に自作部分を All rights reserved へ変更したため、public のまま配布すると AGPL（Ultralytics は利用プロジェクトの AGPL 公開を求める）との整合を満たさない。private 運用（配布しない）か、Ultralytics の商用ライセンス、または Apache 系の代替（YOLOX, OWL-ViT 等）を検討 |
| YOLO-World（元論文・元実装 AILab-CVC/YOLO-World） | Tencent AI Lab | GPL-3.0（README: 商用利用はサポート、商用ライセンスは Tencent に連絡）。本プロジェクトでは**未使用**。2026-10-25 に Ultralytics の再実装（AGPL-3.0）をゼロショットで試したが、2026-10-26 に自前学習の YOLOv8n へ置き換え、YOLO-World 経路のコードとスクリプトは削除した | 2026-10-25 確認 |
| CLIP（テキストエンコーダ、ultralytics/CLIP フォーク） | OpenAI | MIT | YOLO-World の語彙埋め込み作成時のみ使用していた（現在は不要） |
| PyTorch / torchvision（Jetson 用 CUDA 版、予備に CPU 版） | Meta Platforms ほか | BSD-3-Clause | 推論・学習 |
| opencv-python（venv 内） | OpenCV team | Apache-2.0（ホイールの同梱物に MIT/BSD） | 画像処理 |
| NumPy 2.x（venv 内） | NumPy Developers | BSD-3-Clause | |


| コンポーネント | 著作権者 | ライセンス | 用途 |
|---|---|---|---|
| ROS 2 Humble（rclpy, sensor_msgs, geometry_msgs, std_msgs, visualization_msgs, launch_ros, ament_index_python, cv_bridge） | Open Source Robotics Foundation ほか | Apache-2.0 | 基盤 |
| tf2_ros, tf2_geometry_msgs, robot_state_publisher, joint_state_publisher, xacro, rviz2 | Open Source Robotics Foundation, Willow Garage ほか | BSD-3-Clause | TF・URDF・可視化 |
| OpenCV (python3-opencv 4.5.4) | OpenCV team, Intel ほか | Apache-2.0 / BSD-3-Clause | 色検出・円フィット |
| NumPy | NumPy Developers | BSD-3-Clause | 数値計算 |
| SciPy | SciPy Developers | BSD-3-Clause | IK（least_squares）、回転 |
| PyYAML | Kirill Simonov ほか | MIT | 設定読込 |
| depthai 2.33 (Python API / core) | Copyright 2020 Luxonis LLC | MIT | OAK-D Pro 取得ノード。デバイス側ファームウェアの条件は Luxonis の配布物で要確認 |
| feetech-servo-sdk (`scservo_sdk`) | Adam-Software（Feetech SDK のフォーク） | Unlicense | サーボ通信（Lerobot_ros2 経由） |
| librealsense2 / realsense2_camera | Intel Corporation | Apache-2.0 | 開発初期の D455 使用時のみ（現在は未使用） |

## 3. 開発時のみ使用（配布物に含まれない）

| コンポーネント | ライセンス | 用途 |
|---|---|---|
| tinyusdz (`lusdcat`) | Apache-2.0 | USDZ → USDA 変換（CAD 寸法読み取り） |
| matplotlib | PSF ベース | 図の描画 |

## 4. 各ライセンスで守ること

- **Apache-2.0**: 著作権表示・LICENSE 全文の同梱、NOTICE があればその同梱、改変ファイルへの改変表示。本ファイルと `LICENSES/Apache-2.0.txt` で対応。
- **BSD-3-Clause**: 著作権表示・免責の同梱、開発元名の宣伝利用禁止。`LICENSES/BSD-3-Clause.txt`。
- **MIT**: 著作権表示・許諾文の同梱。`LICENSES/MIT.txt`。
- **Unlicense**: 義務なし。
