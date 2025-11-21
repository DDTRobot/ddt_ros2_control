import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "debug",
            default_value="false",
            description="Debug mode",
        )
    )
    config = os.path.join(
        get_package_share_directory('teleop_command'),
        'config',
        'param.yaml'
    )

    return LaunchDescription(declared_arguments + [
        Node(
            package='teleop_command',
            executable='teleop_command_node',
            # name='teleop_command_node',
            namespace=os.environ.get('ROBOT_NS', ''),
            output='screen',
            parameters=[config, {"debug": LaunchConfiguration("debug")}]
        )
    ])

