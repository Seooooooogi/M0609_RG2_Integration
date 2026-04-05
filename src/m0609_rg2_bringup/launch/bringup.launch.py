import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    # xacro 파일 경로
    xacro_file = os.path.join(
        get_package_share_directory('m0609_rg2_bringup'),
        'urdf',
        'm0609_with_rg2.urdf.xacro'
    )

    # rviz 설정 파일 경로
    rviz_config_file = os.path.join(
        get_package_share_directory('m0609_rg2_bringup'),
        'rviz',
        'default.rviz'
    )

    # Static TF (world → base_link)
    static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_transform_publisher',
        output='log',
        arguments=['0.0', '0.0', '0.0', '0.0', '0.0', '0.0', 'world', 'base_link']
    )

    # Robot State Publisher (URDF → TF 트리 퍼블리시)
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='both',
        parameters=[{
            'robot_description': Command(['xacro ', xacro_file])
        }]
    )

    # Joint State Publisher GUI (관절 슬라이더로 움직임 확인)
    joint_state_publisher_gui = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui'
    )

    # RViz
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='log',
        arguments=['-d', rviz_config_file]
    )

    return LaunchDescription([
        static_tf,
        robot_state_publisher,
        joint_state_publisher_gui,
        rviz_node,
    ])
