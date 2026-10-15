"""Launch Webots and the TurtleBot3 ROS 2 driver and controllers."""

import os

import launch
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from webots_ros2_driver.webots_controller import WebotsController
from webots_ros2_driver.webots_launcher import WebotsLauncher
from webots_ros2_driver.wait_for_controller_connection import WaitForControllerConnection


def generate_launch_description():
    package_dir = get_package_share_directory('practice02_webots')
    turtlebot_dir = get_package_share_directory('webots_ros2_turtlebot')
    world = os.path.join(package_dir, 'worlds', 'number_world.wbt')
    webots = WebotsLauncher(
        world=world, mode=LaunchConfiguration('mode'), ros2_supervisor=True,
    )
    description = os.path.join(turtlebot_dir, 'resource', 'turtlebot_webots.urdf')
    control = os.path.join(turtlebot_dir, 'resource', 'ros2control.yml')
    # Jazzy's diffdrive_controller subscribes to TwistStamped.
    mappings = [
        ('/diffdrive_controller/cmd_vel', '/cmd_vel'),
        ('/diffdrive_controller/odom', '/odom'),
    ]
    driver = WebotsController(
        robot_name='TurtleBot3Burger',
        parameters=[
            {'robot_description': description, 'use_sim_time': True,
             'set_robot_state_publisher': True},
            control,
        ],
        remappings=mappings,
        respawn=True,
    )
    timeout = ['--controller-manager-timeout', '50']
    spawners = [
        Node(package='controller_manager', executable='spawner',
             arguments=['diffdrive_controller'] + timeout, output='screen'),
        Node(package='controller_manager', executable='spawner',
             arguments=['joint_state_broadcaster'] + timeout, output='screen'),
    ]
    return LaunchDescription([
        DeclareLaunchArgument('mode', default_value='realtime'),
        webots,
        webots._supervisor,
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             parameters=[{'robot_description': '<robot name=""><link name=""/></robot>'}],
             output='screen'),
        Node(package='tf2_ros', executable='static_transform_publisher',
             arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'base_footprint'],
             output='screen'),
        driver,
        WaitForControllerConnection(target_driver=driver, nodes_to_start=spawners),
        launch.actions.RegisterEventHandler(
            launch.event_handlers.OnProcessExit(
                target_action=webots,
                on_exit=[launch.actions.EmitEvent(event=launch.events.Shutdown())],
            )
        ),
    ])
