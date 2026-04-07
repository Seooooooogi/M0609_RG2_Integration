import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    # ── Launch Arguments ──────────────────────────────────────────────
    # (virtual) ros2 launch m0609_rg2_bringup bringup.launch.py
    # (real)    ros2 launch m0609_rg2_bringup bringup.launch.py mode:=real host:=192.168.1.100
    args = [
        DeclareLaunchArgument('mode',       default_value='virtual',     description='Operation mode: real | virtual'),
        DeclareLaunchArgument('host',       default_value='127.0.0.1',   description='Robot IP (real mode)'),
        DeclareLaunchArgument('port',       default_value='12345',        description='Robot port'),
        DeclareLaunchArgument('gripper_ip', default_value='192.168.1.1', description='OnRobot compute box IP (real mode)'),
    ]

    is_real    = PythonExpression(["'", LaunchConfiguration('mode'), "' == 'real'"])
    is_virtual = PythonExpression(["'", LaunchConfiguration('mode'), "' == 'virtual'"])

    # ── 커스텀 URDF (M0609 + RG2) ────────────────────────────────────
    xacro_file = os.path.join(
        get_package_share_directory('m0609_rg2_bringup'),
        'urdf', 'm0609_with_rg2.urdf.xacro'
    )
    rviz_config_file = os.path.join(
        get_package_share_directory('m0609_rg2_bringup'),
        'rviz', 'default.rviz'
    )

    # ── [real] Doosan URDF (ros2_control 하드웨어 인터페이스용) ───────
    doosan_xacro = PathJoinSubstitution([
        FindPackageShare('dsr_description2'), 'xacro', 'm0609.urdf.xacro'
    ])
    doosan_robot_description = Command([
        FindExecutable(name='xacro'), ' ', doosan_xacro,
        ' name:=dsr01',
        ' host:=', LaunchConfiguration('host'),
        ' port:=', LaunchConfiguration('port'),
        ' mode:=', LaunchConfiguration('mode'),
        ' model:=m0609',
        ' update_rate:=100',
    ])

    # ── [real] ros2_control_node ──────────────────────────────────────
    control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        namespace='dsr01',
        parameters=[
            {'robot_description': ParameterValue(doosan_robot_description, value_type=str)},
            {'update_rate': 100},
            PathJoinSubstitution([FindPackageShare('dsr_controller2'), 'config', 'dsr_controller2.yaml']),
        ],
        condition=IfCondition(is_real),
        output='both',
    )

    # ── [real] joint_state_broadcaster (/dsr01/joint_states 퍼블리시) ─
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        namespace='dsr01',
        arguments=['joint_state_broadcaster', '-c', 'controller_manager'],
        condition=IfCondition(is_real),
    )

    # ── [real] dsr_controller2 ────────────────────────────────────────
    robot_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        namespace='dsr01',
        arguments=['dsr_controller2', '-c', 'controller_manager'],
    )
    delay_controller = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[robot_controller_spawner],
        ),
        condition=IfCondition(is_real),
    )

    # ── [real] OnRobot RG2 드라이버 ──────────────────────────────────
    # /joint_states → /onrobot_joint_states 로 remap (joint_state_publisher와 충돌 방지)
    onrobot_driver = Node(
        package='onrobot_rg_control',
        executable='OnRobotRGControllerServer',
        name='OnRobotRGControllerServer',
        output='screen',
        parameters=[{
            '/onrobot/control':      'modbus',
            '/onrobot/ip':           LaunchConfiguration('gripper_ip'),
            '/onrobot/port':         502,
            '/onrobot/changer_addr': 65,
            '/onrobot/gripper':      'rg2',
            '/onrobot/offset':       5,
        }],
        remappings=[('/joint_states', '/onrobot_joint_states')],
        condition=IfCondition(is_real),
    )

    # ── [real] 그리퍼 너비 → rg2_finger_joint 변환 노드 ──────────────
    # OnRobotRGInput.ggwd → /gripper_joint_states (rg2_finger_joint)
    gripper_joint_state_publisher = Node(
        package='m0609_rg2_bringup',
        executable='gripper_joint_state_publisher.py',
        name='gripper_joint_state_publisher',
        condition=IfCondition(is_real),
        output='screen',
    )

    # ── [real] joint_state_publisher ─────────────────────────────────
    # arm 6축(/dsr01/joint_states) + 그리퍼(/gripper_joint_states) 병합
    joint_state_publisher_real = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        parameters=[{'source_list': ['/dsr01/joint_states', '/gripper_joint_states']}],
        condition=IfCondition(is_real),
    )

    # ── robot_state_publisher ─────────────────────────────────────────
    # real:    joint_state_publisher가 병합한 /joint_states 수신
    # virtual: joint_state_publisher_gui가 /joint_states 퍼블리시
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='both',
        parameters=[{
            'robot_description': ParameterValue(
                Command(['xacro ', xacro_file]),
                value_type=str
            )
        }],
    )

    # ── [virtual] Joint State Publisher GUI ───────────────────────────
    joint_state_publisher_gui = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        condition=IfCondition(is_virtual),
    )

    # ── Static TF (world → base_link) ────────────────────────────────
    static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_transform_publisher',
        output='log',
        arguments=['0.0', '0.0', '0.0', '0.0', '0.0', '0.0', 'world', 'base_link'],
    )

    # ── RViz ──────────────────────────────────────────────────────────
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='log',
        arguments=['-d', rviz_config_file],
    )

    return LaunchDescription(args + [
        control_node,
        joint_state_broadcaster_spawner,
        delay_controller,
        onrobot_driver,
        gripper_joint_state_publisher,
        joint_state_publisher_real,
        robot_state_publisher,
        joint_state_publisher_gui,
        static_tf,
        rviz_node,
    ])
