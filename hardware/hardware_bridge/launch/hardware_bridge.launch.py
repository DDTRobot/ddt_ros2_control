import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import LaunchConfiguration
from launch.actions import OpaqueFunction

import xacro

def launch_setup(context, *args, **kwargs):
    robot_name = LaunchConfiguration("robot").perform(context)
    nn=LaunchConfiguration("namespace")

    prefix_string = nn.perform(context)
    robot_xacro_path = os.path.join(
        get_package_share_directory(robot_name + "_description"),
        "xacro",
        "robot.xacro",
    )

    robot_description = xacro.process_file(
        robot_xacro_path, mappings={"hw_env": "hw"}
    ).toxml()
    
    robot_controllers = os.path.join(
        get_package_share_directory("hardware_bridge"),
        "config",
        "controllers.yaml",
    )
    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        output="both",
        parameters=[
            {"robot_description": robot_description},
            robot_controllers
        ],
        # remappings=[("~/robot_description", "/tita/robot_description")],
        namespace=nn, 
    )
    
    robot_state_pub_node = Node(
        package="robot_state_publisher",
        # output="both",
        executable="robot_state_publisher",
        parameters=[
            {"robot_description": robot_description},
            {"frame_prefix": prefix_string+"/"},
        ],
        namespace=nn, 
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            prefix_string + "/controller_manager",
        ],
    )

    imu_sensor_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "imu_sensor_broadcaster",
            "--controller-manager",
            prefix_string + "/controller_manager",
        ],
    )



    nodes = [
        control_node,
        robot_state_pub_node,
        joint_state_broadcaster_spawner,
        imu_sensor_broadcaster_spawner,
    ]

    return nodes

def generate_launch_description():
 # Declare arguments
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "robot",
            default_value="tita",
            description="Path to the robot description file",
        )
    )
    
    declared_arguments.append(
        DeclareLaunchArgument(
            "namespace",
            default_value=os.environ.get('ROBOT_NS', ''),
            description="namespace of robot",
        )
    )
    return LaunchDescription(
        declared_arguments  + 
        [OpaqueFunction(function=launch_setup)]
    )
