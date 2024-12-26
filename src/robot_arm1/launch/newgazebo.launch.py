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

    # Spawn Entity in Gazebo (ตัวหุ่น)
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

    # Effort Controller Spawner
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

    # -----------------------
    #  Spawn Balls Section
    # -----------------------
    spawn_blue_ball = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=[
            "-file", os.path.join(pkg, "worlds", "blue_ball.sdf"),
            "-entity", "blue_ball"
        ],
        output="screen",
    )

    spawn_green_ball = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=[
            "-file", os.path.join(pkg, "worlds", "green_ball.sdf"),
            "-entity", "green_ball"
        ],
        output="screen",
    )

    spawn_pink_ball = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=[
            "-file", os.path.join(pkg, "worlds", "pink_ball.sdf"),
            "-entity", "pink_ball"
        ],
        output="screen",
    )

    spawn_red_ball_0 = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=[
            "-file", os.path.join(pkg, "worlds", "red_ball_0.sdf"),
            "-entity", "red_ball_0"
        ],
        output="screen",
    )

    spawn_red_ball_1 = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=[
            "-file", os.path.join(pkg, "worlds", "red_ball_1.sdf"),
            "-entity", "red_ball_1"
        ],
        output="screen",
    )

    spawn_yellow_ball = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=[
            "-file", os.path.join(pkg, "worlds", "yellow_ball.sdf"),
            "-entity", "yellow_ball"
        ],
        output="screen",
    )

    # Event: Spawn ลูกบอลทั้งหมดหลังจากตัวหุ่น spawn เสร็จ
    spawn_balls_event = RegisterEventHandler(
        OnProcessExit(
            target_action=spawn_entity,
            on_exit=[
                spawn_blue_ball,
                spawn_green_ball,
                spawn_pink_ball,
                spawn_red_ball_0,
                spawn_red_ball_1,
                spawn_yellow_ball
            ]
        )
    )

    # Add Image Saver and 3D Detector Script
    image_saver_and_3d_detector = Node(
        package="robot_arm1",
        executable="image_saver_and_3d_detector.py",
        output="screen",
        name="image_saver_and_3d_detector",
        parameters=[]  # Add any ROS 2 parameters if needed
    )

    # Final LaunchDescription
    return LaunchDescription([
        gazebo,
        robot_state_publisher,
        spawn_entity,
        joint_state_broadcaster_spawner,
        velocity_controller_event,
        effort_controller_event,
        # ตัวนี้คือ Event Handler ที่จะ spawn ลูกบอลเมื่อตัวหุ่น spawn เสร็จ
        spawn_balls_event,
        image_saver_and_3d_detector
    ])
