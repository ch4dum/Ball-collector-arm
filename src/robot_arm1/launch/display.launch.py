#!/usr/bin/env python3

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
import os

def generate_launch_description():
    # Package paths
    pkg = get_package_share_directory('robot_arm1')
    rviz_path = os.path.join(pkg, 'config', 'robot_arm1.rviz')
    urdf_path = os.path.join(pkg, 'urdf', 'robot_rviz.urdf')

    # Load the URDF file
    with open(urdf_path, 'r') as urdf_file:
        robot_desc_xml = urdf_file.read()

    parameters = [{'robot_description': robot_desc_xml}]

    # RViz Node
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz',
        arguments=['-d', rviz_path],
        output='screen'
    )

    # Robot State Publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=parameters,
        remappings=[('/joint_states', '/joint_states')]
    )

    # Joint State Publisher GUI (optional for manual joint manipulation)
    joint_state_publisher_gui = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        output='screen'
    )

    # Launch Description
    ld = LaunchDescription()
    ld.add_action(rviz)
    ld.add_action(robot_state_publisher)
    ld.add_action(joint_state_publisher_gui)

    return ld
