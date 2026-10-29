#!/usr/bin/env bash
# ROS 2 Humble と ~/ros2_ws を読み込む。使い方:  source scripts/setup_env.sh
source /opt/ros/humble/setup.bash
[ -f "$HOME/ros2_ws/install/setup.bash" ] && source "$HOME/ros2_ws/install/setup.bash"
export SO101_FOLLOWER_PORT="${SO101_FOLLOWER_PORT:-/dev/usbserial_lerobot_follower}"
# udev ルール未適用時は by-id から実デバイスを解決する
if [ ! -e "$SO101_FOLLOWER_PORT" ]; then
  _byid=/dev/serial/by-id/usb-1a86_USB_Single_Serial_5AB9068796-if00
  [ -e "$_byid" ] && export SO101_FOLLOWER_PORT="$(readlink -f "$_byid")"
fi
echo "ROS_DISTRO=$ROS_DISTRO  SO101_FOLLOWER_PORT=$SO101_FOLLOWER_PORT"
