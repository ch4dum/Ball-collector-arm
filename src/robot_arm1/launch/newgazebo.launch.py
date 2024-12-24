#!/usr/bin/env python3

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, RegisterEventHandler
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.event_handlers import OnProcessExit
import os
import xacro

def generate_launch_description():
    package_name = "robot_arm1"
    pkg = get_package_share_directory(package_name)

    # Convert Xacro to URDF
    path_description = os.path.join(pkg, 'urdf', 'robot_arm1.urdf.xacro')
    robot_desc_xml = xacro.process_file(path_description).toxml()
    parameters = [{'robot_description': robot_desc_xml}]

    # Robot State Publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=parameters
    )

    # Gazebo Launch
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("gazebo_ros"),
                "launch",
                "gazebo.launch.py"
            )
        ),
        launch_arguments={"use_sim_time": "true"}.items()
    )

    # Spawn Entity in Gazebo
    spawn_entity = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=[
            "-topic", "robot_description",
            "-entity", "robot_arm1"
        ],
        output="screen",
    )

    # Joint State Broadcaster Spawner
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
        output="screen"
    )

    # Velocity Controller Spawner
    velocity_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["velocity_controllers", "--controller-manager", "/controller_manager"],
        output="screen"
    )

    effort_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["effort_controllers", "--controller-manager", "/controller_manager"],
        output="screen",
    )

    # Event: Spawn velocity controller after joint state broadcaster
    velocity_controller_event = RegisterEventHandler(
        OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[velocity_controller_spawner]
        )
    )

    # Event: Spawn effort controller after velocity controller is ready
    effort_controller_event = RegisterEventHandler(
        OnProcessExit(
            target_action=velocity_controller_spawner,
            on_exit=[effort_controller_spawner]
        )
    )

    # Launch the components
    return LaunchDescription([
        gazebo,
        robot_state_publisher,
        spawn_entity,
        joint_state_broadcaster_spawner,
        velocity_controller_event,
        effort_controller_event
    ])
