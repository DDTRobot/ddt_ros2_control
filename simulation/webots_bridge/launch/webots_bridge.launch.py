#!/usr/bin/env python
import os
import launch
from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
from webots_ros2_driver.webots_launcher import WebotsLauncher
from webots_ros2_driver.webots_controller import WebotsController

from webots_ros2_driver.urdf_spawner import URDFSpawner, get_webots_driver_node
from launch.actions import OpaqueFunction
import xacro


def launch_setup(context, *args, **kwargs):
    robot_name = LaunchConfiguration("robot").perform(context)
    ns = LaunchConfiguration("ns").perform(context)

    robot_xacro_path = os.path.join(
        get_package_share_directory(robot_name + "_description"),
        "xacro",
        "robot.xacro",
    )

    robot_description = xacro.process_file(
        robot_xacro_path, mappings={"hw_env": "webots"}
    ).toxml()
    spawn_robot = URDFSpawner(
        name=robot_name,
        robot_description=robot_description,
        # relative_path_prefix=os.path.join(robot_name + "_description", 'resource'),
        translation="0 0 0.4",
        rotation="0 0 0 0",
    )

    webots = WebotsLauncher(
        world=PathJoinSubstitution(
            [FindPackageShare("webots_bridge"), "worlds", "empty_world.wbt"]
        ),
        ros2_supervisor=True,
    )

    robot_controllers = os.path.join(
        get_package_share_directory("webots_bridge"),
        "config",
        "controllers.yaml",
    )

    tita_driver = WebotsController(
        robot_name=robot_name,
        parameters=[
            {"robot_description": robot_description},
            {"use_sim_time": True},
            {"set_robot_state_publisher": False},
            robot_controllers,
        ],
        respawn=True,
        namespace=ns,
    )

    robot_state_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[
            {"robot_description": robot_description},
            {"frame_prefix": ns + "/"},
        ],
        namespace=ns,
    )
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            ns + "/controller_manager",
        ],
    )

    imu_sensor_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "imu_sensor_broadcaster",
            "--controller-manager",
            ns + "/controller_manager",
        ],
    )

    def get_ros2_nodes(*args):
        return [
            spawn_robot,
            launch.actions.RegisterEventHandler(
                event_handler=launch.event_handlers.OnProcessIO(
                    target_action=spawn_robot,
                    on_stdout=lambda event: get_webots_driver_node(
                        event,
                        [
                            robot_state_pub_node,
                            tita_driver,
                            joint_state_broadcaster_spawner,
                            imu_sensor_broadcaster_spawner,
                        ],
                    ),
                )
            ),
        ]

    webots_event_handler = launch.actions.RegisterEventHandler(
        event_handler=launch.event_handlers.OnProcessExit(
            target_action=webots,
            on_exit=[launch.actions.EmitEvent(event=launch.events.Shutdown())],
        )
    )

    ros2_reset_handler = launch.actions.RegisterEventHandler(
        event_handler=launch.event_handlers.OnProcessExit(
            target_action=webots._supervisor,
            on_exit=get_ros2_nodes,
        )
    )

    return [
        webots,
        webots._supervisor,
        webots_event_handler,
        ros2_reset_handler,
    ] + get_ros2_nodes()


def generate_launch_description():
    declared_arguments = []
    declared_arguments.append(
        launch.actions.DeclareLaunchArgument(
            "robot",
            default_value="tita",
            description="Path to the robot description file",
        )
    )
    declared_arguments.append(
        launch.actions.DeclareLaunchArgument(
            "ns",
            default_value="",
            description="Namespace of launch",
        )
    )
    return LaunchDescription(
        declared_arguments + [OpaqueFunction(function=launch_setup)]
    )
