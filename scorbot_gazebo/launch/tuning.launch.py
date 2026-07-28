#!/usr/bin/env python3

"""
Lanzamiento específico para ajuste PID con trayectoria de lemniscata.

Gazebo inicia pausado y scorbot_arm_controller se carga
en estado inactivo. El nodo de ajuste debe cargar las
ganancias del primer individuo antes de activar el controlador.
"""


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

    controllers_file = os.path.join(
        package_path,
        'config',
        'scorbot_controllers.yaml'
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

    robot_description = robot_description.replace(
        '__SCORBOT_CONTROLLERS_YAML__',
        controllers_file
    )

    # El plugin propio se instala en <prefix>/lib.
    package_prefix = os.path.dirname(
        os.path.dirname(package_path)
    )

    scorbot_plugin_path = os.path.join(
        package_prefix,
        'lib'
    )

    plugin_paths = [
        scorbot_plugin_path,
        '/opt/ros/jazzy/lib'
    ]

    current_plugin_path = os.environ.get(
        'GZ_SIM_SYSTEM_PLUGIN_PATH',
        ''
    )

    if current_plugin_path:
        plugin_paths.append(current_plugin_path)

    gz_plugin_path = SetEnvironmentVariable(
        name='GZ_SIM_SYSTEM_PLUGIN_PATH',
        value=os.pathsep.join(plugin_paths)
    )

    gazebo = ExecuteProcess(
        cmd=[
            'gz',
            'sim',
            '-v',
            '4',
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
            '/controller_manager',
            '--inactive'
        ],
        output='screen'
    )

    scorbot_arm_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'scorbot_arm_controller',
            '--controller-manager',
            '/controller_manager',
            '--inactive'
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


    home_reset_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='home_reset_bridge',
        arguments=[
            '/scorbot/home_reset'
            '@std_msgs/msg/Int32'
            '[gz.msgs.Int32'
        ],
        output='screen'
    )


    pid_reset_done_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='pid_reset_done_bridge',
        arguments=[
            '/scorbot/pid_reset_done'
            '@std_msgs/msg/Int32'
            ']gz.msgs.Int32'
        ],
        output='screen'
    )


    tracking_plotter = Node(
        package='scorbot_gazebo',
        executable='cartesian_tracking_plotter.py',
        name='cartesian_tracking_plotter',
        parameters=[
            {
                'use_sim_time': True
            }
        ],
        output='screen'
    )

    # Usa tiempo de pared para poder trabajar aunque
    # la simulación se encuentre pausada.
    pid_reset_supervisor = Node(
        package='scorbot_gazebo',
        executable='scorbot_pid_reset_supervisor.py',
        name='scorbot_pid_reset_supervisor',
        output='screen'
    )

    return LaunchDescription([
        gz_plugin_path,
        gazebo,
        robot_state_publisher,
        spawn_scorbot,
        tracking_plotter,
        clock_bridge,
        world_control_bridge,
        home_reset_bridge,
        pid_reset_done_bridge,
        pid_reset_supervisor,
        delayed_controllers
    ])