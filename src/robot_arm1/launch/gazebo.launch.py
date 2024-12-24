#!/usr/bin/env python3

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import os
import xacro

def generate_launch_description():
    # Paths
    pkg_path = get_package_share_directory('robot_arm1')
    gazebo_ros_path = get_package_share_directory('gazebo_ros')
    rviz_path = os.path.join(pkg_path, 'config', 'robot_arm1.rviz')
    xacro_path = os.path.join(pkg_path, 'urdf', 'robot_rviz.urdf.xacro')

    # Process Xacro to URDF
    robot_desc_xml = xacro.process_file(xacro_path).toxml()
    parameters = [{'robot_description': robot_desc_xml}]

    # Gazebo launch
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_ros_path, 'launch', 'gazebo.launch.py')
        )
    )

    # Spawn Robot in Gazebo
    spawn_robot = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-entity', 'robotarm', '-topic', '/robot_description'],
        output='screen'
    )

    # Robot State Publisher (sends /joint_states to RViz)
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=parameters,
        remappings=[('/joint_states', '/joint_states')]
    )

    # Load Joint State Broadcaster
    load_joint_state_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
        output="screen"
    )

    # Load Arm Controller (optional, if you have trajectory control)
    load_arm_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["arm_controller", "--controller-manager", "/controller_manager"],
        output="screen"
    )

    # RViz2 - Visualize Robot State
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_path],
        output='screen'
    )

    # Launch Description
    return LaunchDescription([
        gazebo,
        spawn_robot,
        robot_state_publisher,
        load_joint_state_controller,
        load_arm_controller,  # Optional
        rviz
    ])