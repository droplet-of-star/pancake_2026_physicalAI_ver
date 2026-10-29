# SO-101 × RealSense × ROS 2：Pick & Place 実装計画

> **状況（2026-10-25）**: STEP 0〜6 は完了し、実機で自律 pick & place が動作している。カメラは 2026-10-21 に RealSense D455 から OAK-D Pro に変更した。本書は当初計画の記録であり、現在の構成・起動手順は `README.md`、技術解説は `docs/システム解説_2026-10-25.md`、経緯は `docs/環境確認結果_2026-10-13.md` を参照。

## 1. 目的

SO-101の手首付近に取り付けたRealSenseを使い、作業台上の**赤いお皿を認識し、自動でつかんで指定位置に置く**。

最初は物体認識に**OpenCVの赤色検出**を使う。YOLOなどの学習モデルは必要になった段階で追加する。

### 完成時の動作

1. RealSenseでRGB画像と深度画像を取得する。
2. 赤いお皿を検出する。
3. 深度画像を用いてお皿の3D位置を推定する。
4. TF2でカメラ座標系からSO-101のベース座標系へ変換する。
5. 把持位置・接近方向を決め、逆運動学／動作計画を行う。
6. SO-101を移動させ、グリッパでお皿をつかむ。
7. 固定の配置位置に移動し、お皿を置く。

## 2. 用意するもの

### ハードウェア

| 項目 | 用途 | 備考 |
|---|---|---|
| SO-101 Follower | ロボットアーム | 既存機体を使用 |
| Intel RealSense | RGB画像・深度画像取得 | D455を想定。型番は要確認 |
| Ubuntu PC または Jetson | ROS 2実行 | ディストリビューションを確認 |
| USB 3対応ケーブル | カメラ接続 | ロボット可動部に干渉しないよう配線 |
| カメラ取付ブラケット | 手首付近への固定 | 重量、視野、グリッパ干渉に注意 |
| 赤いお皿 | 把持対象 | 軽量、形状と寸法が分かるもの |
| 作業台・配置用容器 | 作業環境 | 最初は配置先を固定 |
| 緊急停止手段 | 安全確保 | 速度・可動域制限も設定 |

> **注意**：D455の重量や近距離での深度測定条件がSO-101の手首取り付けに適するか、実機で事前確認する。ブラケット・ケーブルを含む負荷を考慮する。

### ソフトウェア

| 項目 | 用途 | 必要性 |
|---|---|---|
| ROS 2 | ノード・トピック連携 | 必須 |
| `realsense2_camera` | RealSense画像配信 | 必須 |
| OpenCV | 赤色お皿検出 | 必須 |
| `cv_bridge` | ROS画像とOpenCV画像の変換 | 必須 |
| `tf2_ros` | 座標変換 | 必須 |
| URDF / `robot_state_publisher` | ロボット姿勢とTF配信 | 必須 |
| SO-101モーターブリッジ | 関節制御 | 必須 |
| 逆運動学ソルバー | 手先目標位置から関節角を算出 | 必須 |
| MoveIt 2 | 動作計画・衝突確認 | 推奨 |
| RViz2 | 可視化・デバッグ | 推奨 |
| YOLO | 複数物体の認識等 | 後から追加可能 |

## 3. ROS 2 システム構成

```text
SO-101手首のRealSense
   │
   ▼
realsense2_camera
   ├── RGB画像 ──────────┐
   ├── 深度画像 ──────┐  │
   └── camera_info ─┐ │  │
                    ▼ ▼  ▼
            object_detector
             （赤色検出）
                    │ 画像上の中心 (u, v)
                    ▼
           object_pose_estimator
             （深度から3D位置）
                    │ カメラ座標系のXYZ
                    ▼
                 tf2_ros ◄── URDF / joint_states
                    │ ベース座標系のXYZ
                    ▼
           pick_place_controller
                    │ 把持姿勢・置き場所
                    ▼
          IK / MoveIt 2（推奨）
                    │ 関節指令
                    ▼
          SO-101 motor bridge
                    │
                    ▼
                 SO-101
```

## 4. 実装ロードマップ

| STEP | 内容 | 完了条件 |
|---|---|---|
| 1 | RealSenseをROS 2で起動 | RGB・深度画像を受信できる |
| 2 | 赤色お皿検出 | 画像上の中心座標 `(u, v)` を表示できる |
| 3 | カメラ座標系の3D位置推定 | 深度と内部パラメータからXYZを計算できる |
| 4 | SO-101ベース座標へ変換 | 複数のアーム姿勢で座標が安定する |
| 5 | SO-101を目標位置に移動 | 安全な速度で物体上方へ移動できる |
| 6 | Pick & Placeを統合 | お皿をつかみ、固定位置に置ける |

## 5. STEP 0：既存ROS 2環境の確認

### ROS 2ディストリビューション

```bash
echo $ROS_DISTRO
```

### 関連パッケージ

```bash
ros2 pkg list | grep -E 'so101|moveit|tf2|realsense'
```

### SO-101のシリアルポート

```bash
ls -l /dev/serial/by-id/
ls -l /dev/ttyUSB*
```

利用しているモーターブリッジの例：

```bash
ros2 run so101_hw_interface so101_motor_bridge \
  --ros-args -p port:=/dev/usbserial_lerobot_follower
```

> `/dev/usbserial_lerobot_follower` は存在する場合だけ使用する。以前このパスの不存在によるエラーが出ていたため、実際のデバイス名を確認する。

**完了条件**：関節角度を読み取れ、SO-101を安全に指令できる。

## 6. STEP 1：RealSenseの画像取得

ROS 2ディストリビューションに合った `realsense2_camera` を導入する。

公式リポジトリ：https://github.com/realsenseai/realsense-ros

### パッケージの確認

```bash
ros2 pkg list | grep realsense
```

### カメラ起動

```bash
ros2 launch realsense2_camera rs_launch.py \
  align_depth.enable:=true \
  pointcloud.enable:=true
```

### トピック確認

```bash
ros2 topic list
```

代表的なトピック（名前は起動設定により異なる）：

| トピック例 | 内容 |
|---|---|
| `/camera/camera/color/image_raw` | RGB画像 |
| `/camera/camera/depth/image_rect_raw` | 深度画像 |
| `/camera/camera/aligned_depth_to_color/image_raw` | RGBに位置合わせされた深度画像 |
| `/camera/camera/color/camera_info` | 内部パラメータ |
| `/camera/camera/depth/color/points` | 3D点群（設定により名前が異なる） |

RViz2などで画像・点群を確認する。

**完了条件**：RGB画像、深度画像、`CameraInfo`を取得できる。

## 7. STEP 2：赤いお皿の検出

最初はOpenCVによるHSV色空間での赤色抽出を使う。

1. RGB画像を購読する。
2. HSV色空間に変換し、赤色範囲を二つの区間で抽出する。
3. ノイズ除去・輪郭検出を行う。
4. 最大領域の中心 `(u, v)` と外接矩形を取得する。
5. 検出位置を画像上に描画する。

**完了条件**：赤いお皿を動かしても中心座標を追跡できる。

## 8. STEP 3：深度から3D位置を推定

RGB画像と位置合わせした深度画像の対応する画素を用いる。

カメラ内部パラメータ `fx`, `fy`, `cx`, `cy` と深度 `Z` から、

```text
X = (u - cx) * Z / fx
Y = (v - cy) * Z / fy
Z = depth(u, v)
```

を計算する（単位をメートルに統一する）。

- 欠損深度値や外れ値を除外する。
- 中心画素だけに頼らず、検出領域内の有効深度値の中央値などを利用する。
- 得られる座標は基本的に**見えている表面上の点**。把持位置の決定にはお皿の寸法・向きを別途考慮する。

**完了条件**：カメラを動かしても、観測された点の3D位置が合理的に変化する。

## 9. STEP 4：TF2でSO-101基準へ変換

```text
base_link
  └── ...（SO-101の関節リンク）
       └── wrist_link（実際のリンク名を使用）
            └── camera_link（取付位置・姿勢）
                 └── camera_color_optical_frame
                      └── object_point
```

必要な設定：

1. SO-101のURDFと、実際の関節角度を `joint_states` として配信する。
2. `robot_state_publisher` などでロボットのTFを配信する。
3. 手首とカメラ間の固定変換を計測・登録する。
4. 必要ならAprilTag等を用いたHand-Eye Calibrationで取付誤差を補正する。
5. 物体位置を、**画像の取得時刻に対応するTF**で `base_link` に変換する。

> カメラを手首に固定すると、アームの関節角度でカメラ位置が変わる。固定カメラと違い、手首からカメラまでの外部パラメータが重要。

**完了条件**：アームの姿勢を変えても、静止お皿のベース座標系での位置が大きく変わらない。

## 10. STEP 5：目標位置への移動

最初の試験では、対象物に触れず、お皿上方の安全な位置へ移動する。

1. グリッパの把持方向を決める。
2. 物体表面の座標から、お皿の寸法とグリッパ形状を考慮して把持姿勢を計算する。
3. **接近位置 → 把持位置 → 退避位置**の順に目標を用意する。
4. IKまたはMoveIt 2で到達可能性を確認する。
5. 速度・関節範囲・作業台への干渉をチェックする。
6. 低速で実機を動かす。

**完了条件**：お皿に触れずに、その上方の再現性ある位置に移動できる。

## 11. STEP 6：Pick & Place統合

```text
START
  ↓
RGB・Depth取得
  ↓
赤いお皿検出
  ↓
3D位置算出・TF変換
  ↓
到達可能性と安全性を確認
  ↓
グリッパを開く
  ↓
接近位置へ移動
  ↓
把持位置へ移動
  ↓
グリッパを閉じる
  ↓
持ち上げる
  ↓
固定の配置位置へ移動
  ↓
グリッパを開く
  ↓
初期位置へ戻る
  ↓
END
```

**完了条件**：赤いお皿の初期位置を変えても、一定の成功率で指定場所に置ける。

## 12. VS Codeでの推奨ワークスペース構成

ROS 2 Pythonパッケージを複数に分ける例：

```text
ros2_ws/
├── src/
│   ├── so101_hw_interface/       # 既存のSO-101制御パッケージ
│   ├── so101_description/        # URDF・TF設定（名称は例）
│   └── so101_pick_place/         # 新規作成するPythonパッケージ
│       ├── package.xml
│       ├── setup.py
│       ├── setup.cfg
│       ├── resource/
│       ├── so101_pick_place/
│       │   ├── __init__.py
│       │   ├── object_detector.py
│       │   ├── object_pose_estimator.py
│       │   └── pick_place_controller.py
│       ├── launch/
│       │   └── pick_place.launch.py
│       └── config/
│           └── params.yaml
├── build/
├── install/
└── log/
```

新規Pythonパッケージのひな形作成例：

```bash
cd ~/ros2_ws/src
ros2 pkg create so101_pick_place --build-type ament_python \
  --dependencies rclpy sensor_msgs geometry_msgs cv_bridge tf2_ros
```

> 上の構成は**設計案**であり、これらのPythonファイルが自動的に実装されるわけではない。既存パッケージや依存関係に合わせて調整する。

## 13. 安全・精度面の注意

- 最初は**グリッパをお皿の上方で止めるテスト**から始める。
- SO-101の可動範囲、速度、電流／トルク条件を守る。
- カメラやケーブルが作業台・アームに接触しないようにする。
- RealSenseの最小測定距離、死角、深度欠損に注意する。
- カメラ画像とTFのタイムスタンプを合わせる。
- 物体が見つからない場合や深度が無効な場合は**動かさず停止**する。
- 把持前に到達可能性・衝突を確認する。

## 14. 次に確認する環境情報

以下をターミナルで確認し、実装時のバージョンを固定する。

```bash
# ROS 2
printf 'ROS_DISTRO=%s\n' "$ROS_DISTRO"

# OS
lsb_release -a

# SO-101 / MoveIt / RealSenseパッケージ
ros2 pkg list | grep -E 'so101|moveit|realsense'

# カメラ・SO-101の接続デバイス
lsusb
ls -l /dev/serial/by-id/
```

確認したい項目：

- ROS 2のディストリビューション（Humble / Jazzyなど）
- 実行マシン（Ubuntu PC / Jetson AGX Orin）
- RealSenseの型番（D455 / D435 / D435iなど）
- SO-101制御パッケージの起動可否

## 15. 参考リンク

- [RealSense ROS 2 wrapper](https://github.com/realsenseai/realsense-ros)
- [ROS 2公式ドキュメント](https://docs.ros.org/)
- [MoveIt](https://moveit.ai/)
- [OpenCV](https://opencv.org/)
