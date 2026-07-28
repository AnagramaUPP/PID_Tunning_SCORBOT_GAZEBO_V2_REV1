#!/usr/bin/env python3

import os

from launch import LaunchDescription
from launch.actions import ExecuteProcess, SetEnvironmentVariable, TimerAction
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    package_path = get_package_share_directory('scorbot_gazebo')

    world_file = os.path.join(
        package_path,
        'worlds',
        'empty.world'
    )

    urdf_file = os.path.join(
        package_path,
        'urdf',
        'scorbot_gazebo.urdf'
    )

    with open(urdf_file, 'r') as file:
        robot_description = file.read()

    robot_description = robot_description.replace(
        'package://scorbot_gazebo',
        'file://' + package_path
    )

    gz_plugin_path = SetEnvironmentVariable(
        name='GZ_SIM_SYSTEM_PLUGIN_PATH',
        value='/opt/ros/jazzy/lib'
    )

    gazebo = ExecuteProcess(
        cmd=[
            'gz',
            'sim',
            '-r',
            world_file
        ],
        output='screen'
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[
            {
                'robot_description': robot_description,
                'use_sim_time': True
            }
        ],
        output='screen'
    )

    spawn_scorbot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', 'scorbot',
            '-topic', 'robot_description',
            '-x', '0.0',
            '-y', '0.0',
            '-z', '0.00'
        ],
        output='screen'
    )

    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'joint_state_broadcaster',
            '--controller-manager',
            '/controller_manager'
        ],
        output='screen'
    )

    scorbot_arm_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'scorbot_arm_controller',
            '--controller-manager',
            '/controller_manager'
        ],
        output='screen'
    )

    delayed_controllers = TimerAction(
        period=6.0,
        actions=[
            joint_state_broadcaster_spawner,
            scorbot_arm_controller_spawner
        ]
    )

    clock_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'
        ],
        output='screen'
    )


    world_control_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='world_control_bridge',
        arguments=[
            '/world/empty/control@ros_gz_interfaces/srv/ControlWorld'
        ],
        output='screen'
    )

    return LaunchDescription([
        gz_plugin_path,
        gazebo,
        robot_state_publisher,
        spawn_scorbot,
        clock_bridge,
        world_control_bridge,
        delayed_controllers
    ])