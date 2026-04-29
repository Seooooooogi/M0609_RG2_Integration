"""YOLO bbox visualization launch.

Visualizer node depends on ultralytics + torch (GPU). Host has neither — run the
visualizer inside the zium-detection:humble-cu118 docker image. RealSense node
runs on host. Both sides require RMW_IMPLEMENTATION=rmw_cyclonedds_cpp.
See docs/yolo_detection_pipeline.md for full launch commands.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    confidence_arg = DeclareLaunchArgument(
        'confidence_threshold',
        default_value='0.5',
        description='YOLO confidence threshold (0.0 - 1.0)',
    )

    realsense_node = Node(
        package='realsense2_camera',
        executable='realsense2_camera_node',
        namespace='camera_gripper',
        name='camera_gripper',
        parameters=[{
            'enable_color': True,
            'enable_depth': True,
            'align_depth.enable': True,
            'enable_gyro': False,
            'enable_accel': False,
        }],
        output='screen',
    )

    visualizer_node = Node(
        package='object_detection',
        executable='yolo_visualizer',
        name='yolo_visualizer',
        parameters=[{
            'confidence_threshold': ParameterValue(
                LaunchConfiguration('confidence_threshold'),
                value_type=float,
            ),
        }],
        output='screen',
    )

    return LaunchDescription([
        confidence_arg,
        realsense_node,
        visualizer_node,
    ])
