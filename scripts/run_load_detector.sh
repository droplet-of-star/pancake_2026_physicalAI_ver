#!/usr/bin/env bash
# load_detector（YOLO）を専用 venv の Python で起動する。
# 理由: Jetson の apt numpy 1.21 と pip の PyTorch が BLAS シンボルで衝突するため、
#       numpy 1.24 + torch(CPU) + ultralytics は ~/.venvs/yolo に隔離してある（作成手順は README）。
# 使い方: bash scripts/run_load_detector.sh [--ros-args ...]
set -e
# 既定: GPU 版 venv（~/.venvs/yolo-gpu、JetPack 6 用 torch 2.11 + CUDA 12.6）があればそれ、無ければ CPU 版 venv（~/.venvs/yolo）
if [ -n "$YOLO_VENV" ]; then VENV="$YOLO_VENV"
elif [ -x "$HOME/.venvs/yolo-gpu/bin/python" ]; then VENV="$HOME/.venvs/yolo-gpu"
else VENV="$HOME/.venvs/yolo"; fi
if [ ! -x "$VENV/bin/python" ]; then
  echo "venv が見つかりません: $VENV  (README の『載せ物検出（YOLO）』の手順で作成してください)" >&2
  exit 1
fi
# Jetson 用 torch は cuDSS を pip の nvidia-cudss-cu12 から読むので、そのライブラリパスを通す
CUDSS="$VENV/lib/python3.10/site-packages/nvidia/cu12/lib"
[ -d "$CUDSS" ] && export LD_LIBRARY_PATH="$CUDSS${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
if [ -z "$ROS_DISTRO" ]; then
  source /opt/ros/humble/setup.bash
  [ -f "$HOME/ros2_ws/install/setup.bash" ] && source "$HOME/ros2_ws/install/setup.bash"
fi
PARAMS="$HOME/ros2_ws/install/so101_pick_place/share/so101_pick_place/config/params.yaml"
if [ $# -eq 0 ]; then
  exec "$VENV/bin/python" -m so101_pick_place.load_detector --ros-args --params-file "$PARAMS"
else
  exec "$VENV/bin/python" -m so101_pick_place.load_detector "$@"
fi
