"""Pick & Place 一式の起動ファイル。

  ros2 launch so101_pick_place pick_place.launch.py            # OAK カメラ + 検出 + 推定 + 制御(dry_run) + 載せ物検出
  ros2 launch so101_pick_place pick_place.launch.py use_camera:=false          # カメラノードを起動しない（別途起動済みのとき）
  ros2 launch so101_pick_place pick_place.launch.py use_controller:=false
  ros2 launch so101_pick_place pick_place.launch.py use_load_detector:=false   # YOLO 載せ物検出を起動しない

モーターブリッジ (so101_hw_interface so101_motor_bridge) と TF (robot_tf.launch.py) はここからは起動しない。
別ターミナルで手動起動する（bridge は起動時にアームを動かし得るため）。
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _load_detector_script():
    """scripts/run_load_detector.sh の場所。--symlink-install ならこのファイルはリポジトリ内を指すので、そこから辿る。"""
    repo = os.path.dirname(os.path.dirname(os.path.dirname(os.path.realpath(__file__))))
    cand = os.path.join(repo, 'scripts', 'run_load_detector.sh')
    if os.path.exists(cand):
        return cand
    return os.path.expanduser('~/vscode/muran2026_physicalAI_ver/scripts/run_load_detector.sh')


def generate_launch_description():
    pkg_share = get_package_share_directory('so101_pick_place')
    params = os.path.join(pkg_share, 'config', 'params.yaml')

    use_camera = LaunchConfiguration('use_camera')
    use_controller = LaunchConfiguration('use_controller')
    use_load_detector = LaunchConfiguration('use_load_detector')

    oak = Node(
        package='so101_pick_place', executable='oak_camera', name='oak_camera', output='screen',
        parameters=[params], condition=IfCondition(use_camera),
    )
    detector = Node(
        package='so101_pick_place', executable='object_detector',
        name='object_detector', output='screen', parameters=[params],
    )
    estimator = Node(
        package='so101_pick_place', executable='object_pose_estimator',
        name='object_pose_estimator', output='screen', parameters=[params],
    )
    controller = Node(
        package='so101_pick_place', executable='pick_place_controller',
        name='pick_place_controller', output='screen', parameters=[params],
        condition=IfCondition(use_controller),
    )
    # 載せ物検出（YOLO）。PyTorch は専用 venv に隔離してあるため、ラッパースクリプト経由で起動する
    load_detector = ExecuteProcess(
        cmd=[_load_detector_script(), '--ros-args', '--params-file', params],
        output='screen', condition=IfCondition(use_load_detector),
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_camera', default_value='true',
                              description='OAK-D Pro のカメラノード（oak_camera）を同時に起動する'),
        DeclareLaunchArgument('use_controller', default_value='true',
                              description='pick_place_controller を起動する'),
        DeclareLaunchArgument('use_load_detector', default_value='true',
                              description='載せ物検出（YOLO, ~/.venvs/yolo-gpu）を起動する'),
        oak, detector, estimator, controller, load_detector,
    ])
