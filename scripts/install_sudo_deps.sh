#!/usr/bin/env bash
# sudo が必要な作業。内容を確認してから手動で実行する:  bash scripts/install_sudo_deps.sh
set -euo pipefail
HERE="$(cd "$(dirname "$0")/.." && pwd)"

echo "[1/3] apt: 追加 ROS パッケージ"
sudo apt-get update
sudo apt-get install -y \
  ros-humble-tf-transformations \
  ros-humble-topic-based-ros2-control \
  ros-humble-ros2-controllers \
  ros-humble-joint-trajectory-controller \
  ros-humble-joint-state-broadcaster \
  ros-humble-image-pipeline \
  python3-transforms3d

echo "[2/3] udev: SO-101 用の固定デバイス名"
sudo install -m 644 "$HERE/udev/99-so101-lerobot.rules" /etc/udev/rules.d/99-so101-lerobot.rules
sudo install -m 644 "$HERE/udev/80-movidius.rules" /etc/udev/rules.d/80-movidius.rules   # OAK-D Pro
sudo udevadm control --reload-rules
sudo udevadm trigger --subsystem-match=tty
sudo udevadm trigger --subsystem-match=usb
sleep 1
ls -l /dev/usbserial_lerobot_* || echo "  (USB を抜き差しすると symlink が作成される)"

echo "[3/3] 完了。新しいターミナルで source scripts/setup_env.sh を実行する"
