#!/usr/bin/env bash
# 実装計画 STEP 0 / 14章 の環境確認を一括で行う（読み取りのみ。ロボットは動かさない）
source /opt/ros/humble/setup.bash 2>/dev/null
source "$HOME/ros2_ws/install/setup.bash" 2>/dev/null
set -u   # ROS の setup.bash は未定義変数を参照するため、source の後に有効化する

hr() { printf '\n== %s ==\n' "$1"; }

hr "OS / Jetson"
lsb_release -ds 2>/dev/null; head -1 /etc/nv_tegra_release 2>/dev/null
printf 'ROS_DISTRO=%s  python=%s\n' "${ROS_DISTRO:-unset}" "$(python3 --version 2>&1)"

hr "ROS packages (so101 / moveit / realsense / tf2 / cv_bridge)"
ros2 pkg list | grep -E '^(so101|moveit_(core|ros_planning|kinematics)|realsense2_camera$|tf2_ros$|cv_bridge|so101_pick_place)' | sed 's/^/  /'

hr "SO-101 serial ports"
ls -l /dev/serial/by-id/ 2>/dev/null | sed 's/^/  /'
for l in /dev/usbserial_lerobot_follower /dev/usbserial_lerobot_leader; do
  [ -e "$l" ] && echo "  $l -> $(readlink -f "$l")" || echo "  $l (not present; see udev/README)"
done
for d in /dev/ttyACM*; do [ -e "$d" ] && { who=$(fuser "$d" 2>/dev/null); echo "  $d in use by: ${who:-none}"; }; done

hr "RealSense"
lsusb | grep -i '8086' || echo "  Intel RealSense not found on USB"
timeout 10 rs-enumerate-devices -s 2>&1 | sed 's/^/  /' | head -5

hr "Other cameras (v4l2)"
ls /dev/video* 2>/dev/null | sed 's/^/  /'

hr "Calibration"
ls -l "$HOME/.so101_follower_calibration.yaml" 2>/dev/null || echo "  ~/.so101_follower_calibration.yaml not found"

hr "Python modules"
python3 - <<'PY'
import importlib
for m in ["cv2","numpy","scipy","cv_bridge","tf2_ros","tf2_geometry_msgs","image_geometry","message_filters","scservo_sdk"]:
    try:
        importlib.import_module(m); print(f"  OK   {m}")
    except Exception as e:
        print(f"  FAIL {m}: {e}")
PY

hr "ROS graph (running nodes)"
timeout 5 ros2 node list 2>/dev/null | sed 's/^/  /' || echo "  (none)"
