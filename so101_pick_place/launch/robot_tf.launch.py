"""SO-101 + 手首カメラ（OAK-D Pro）の TF を配信する。

  ros2 launch so101_pick_place robot_tf.launch.py                     # 実機の joint_states を使う
  ros2 launch so101_pick_place robot_tf.launch.py use_fake_joints:=true  # 関節角ゼロで TF だけ確認（アーム不要）
  ros2 launch so101_pick_place robot_tf.launch.py camera_xyz:="0.02 0 0.05" camera_rpy:="0 1.57 0"

取付値の既定は config/camera_mount.yaml から読む。launch 引数で上書きできる。
関節角は so101_motor_bridge が出す /so101_follower/joint_states を使う（motor bridge はここでは起動しない）。
"""
import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def _load_mount():
    path = os.path.join(get_package_share_directory('so101_pick_place'), 'config', 'camera_mount.yaml')
    with open(path, 'r', encoding='utf-8') as fp:
        m = yaml.safe_load(fp) or {}
    xyz = ' '.join(str(float(v)) for v in m.get('camera_xyz', [0, 0, 0]))
    rpy = ' '.join(str(float(v)) for v in m.get('camera_rpy', [0, 0, 0]))
    return m.get('camera_parent', 'gripper_link'), xyz, rpy


def generate_launch_description():
    parent_default, xyz_default, rpy_default = _load_mount()

    camera_parent = LaunchConfiguration('camera_parent')
    camera_xyz = LaunchConfiguration('camera_xyz')
    camera_rpy = LaunchConfiguration('camera_rpy')
    joint_states_topic = LaunchConfiguration('joint_states_topic')
    use_fake_joints = LaunchConfiguration('use_fake_joints')
    use_rviz = LaunchConfiguration('rviz')

    robot_description = ParameterValue(
        Command([
            PathJoinSubstitution([FindExecutable(name='xacro')]), ' ',
            PathJoinSubstitution([FindPackageShare('so101_pick_place'), 'urdf', 'so101_with_camera.urdf.xacro']),
            ' camera_parent:=', camera_parent,
            " camera_xyz:='", camera_xyz, "'",
            " camera_rpy:='", camera_rpy, "'",
        ]),
        value_type=str,
    )

    rsp = Node(
        package='robot_state_publisher', executable='robot_state_publisher', output='screen',
        parameters=[{'robot_description': robot_description}],
        remappings=[('/joint_states', joint_states_topic)],
    )
    fake_joints = Node(
        package='joint_state_publisher', executable='joint_state_publisher', name='fake_joint_state_publisher',
        parameters=[{'rate': 20}],
        remappings=[('/joint_states', joint_states_topic)],
        condition=IfCondition(use_fake_joints),
    )
    rviz = Node(
        package='rviz2', executable='rviz2', output='log',
        arguments=['-d', PathJoinSubstitution([FindPackageShare('so101_follower_description'), 'rviz', 'display.rviz'])],
        condition=IfCondition(use_rviz),
    )

    return LaunchDescription([
        DeclareLaunchArgument('camera_parent', default_value=parent_default,
                              description='カメラを取り付けるリンク'),
        DeclareLaunchArgument('camera_xyz', default_value=xyz_default,
                              description='camera_parent から三脚ねじ穴までの xyz [m]'),
        DeclareLaunchArgument('camera_rpy', default_value=rpy_default,
                              description='camera_parent から見たカメラ姿勢 rpy [rad]'),
        DeclareLaunchArgument('joint_states_topic', default_value='/so101_follower/joint_states',
                              description='関節角トピック（motor bridge の出力）'),
        DeclareLaunchArgument('use_fake_joints', default_value='false',
                              description='true: 関節角ゼロを配信して TF だけ確認する'),
        DeclareLaunchArgument('rviz', default_value='false'),
        rsp, fake_joints, rviz,
    ])
