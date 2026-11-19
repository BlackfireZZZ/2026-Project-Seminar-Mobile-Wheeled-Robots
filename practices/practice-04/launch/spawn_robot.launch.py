import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, OpaqueFunction, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from webots_ros2_driver.webots_controller import WebotsController
from webots_ros2_driver.webots_launcher import WebotsLauncher
from webots_ros2_driver.wait_for_controller_connection import WaitForControllerConnection


def start(context):
    def arg(name):
        return LaunchConfiguration(name).perform(context)

    folder = get_package_share_directory('practice_localization')
    turtlebot = get_package_share_directory('webots_ros2_turtlebot')
    port = arg('port')
    name = arg('experiment')
    mode = 'global' if name.startswith('global') else 'initial'
    low = name.endswith('low')
    big_error = 'large' in name
    min_particles, max_particles = (300, 1000) if low else (500, 10000)
    error_x, error_y, error_yaw = (0.5, 0.5, 1.0) if big_error else (0.15, -0.15, 0.2)
    if mode == 'global':
        error_x, error_y, error_yaw = 0.0, 0.0, 0.0

    webots = WebotsLauncher(
        world=arg('world_path'), mode=arg('mode'),
        gui=arg('gui').lower() == 'true', ros2_supervisor=False, port=port,
    )
    rviz = Node(
        package='rviz2', executable='rviz2', name='rviz2', output='screen',
        arguments=['-d', os.path.join(folder, 'rviz', 'config.rviz')],
        parameters=[{'use_sim_time': True}],
    )
    robot_state_publisher = Node(
        package='robot_state_publisher', executable='robot_state_publisher', output='screen',
        parameters=[{'robot_description': Path(folder, 'urdf', 'turtlebot.urdf').read_text(),
                     'use_sim_time': True}],
    )
    footprint_publisher = Node(
        package='tf2_ros', executable='static_transform_publisher', output='screen',
        arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'base_footprint'],
    )
    map_server = Node(
        package='nav2_map_server', executable='map_server', name='map_server', output='screen',
        parameters=[{'use_sim_time': True, 'yaml_filename': os.path.join(folder, 'maps', 'numbers.yaml')}],
    )
    amcl = Node(
        package='nav2_amcl', executable='amcl', name='amcl', output='screen',
        parameters=[os.path.join(folder, 'config', 'amcl.yaml'), {
            'use_sim_time': True, 'set_initial_pose': False,
            'min_particles': min_particles, 'max_particles': max_particles,
        }],
    )
    lifecycle = Node(
        package='nav2_lifecycle_manager', executable='lifecycle_manager',
        name='lifecycle_manager_localization', output='screen',
        parameters=[{'use_sim_time': True, 'autostart': True, 'node_names': ['map_server', 'amcl']}],
    )
    supervisor = WebotsController(
        robot_name='ground_truth_supervisor',
        port=port,
        parameters=[{'robot_description': os.path.join(folder, 'urdf', 'ground_truth_supervisor.urdf'),
                     'use_sim_time': True}], respawn=True,
    )

    # ROS control spawners
    spawners = [
        Node(
            package='controller_manager', executable='spawner', output='screen',
            arguments=['diffdrive_controller', '--controller-manager-timeout', '50',
                       '--controller-ros-args=-p diffdrive_controller:wheel_separation_multiplier:=1.1'],
        ),
        Node(
            package='controller_manager', executable='spawner', output='screen',
            arguments=['joint_state_broadcaster', '--controller-manager-timeout', '50'],
        ),
    ]
    driver = WebotsController(
        robot_name='TurtleBot3Burger',
        port=port,
        parameters=[{
            'robot_description': os.path.join(folder, 'urdf', 'turtlebot.urdf'),
            'use_sim_time': True, 'set_robot_state_publisher': True,
        }, os.path.join(turtlebot, 'resource', 'ros2control.yml')],
        remappings=[('/diffdrive_controller/cmd_vel', '/cmd_vel'), ('/diffdrive_controller/odom', '/odom')],
        respawn=True,
    )
    experiment = Node(
        package='practice_localization', executable='experiment', name='experiment', output='screen',
        parameters=[{
            'use_sim_time': True, 'duration': float(arg('duration')),
            'position_threshold': float(arg('position_threshold')),
            'angle_threshold': float(arg('angle_threshold')),
            'confirmation_time': float(arg('confirmation_time')),
            'localization_mode': mode,
            'error_x': error_x, 'error_y': error_y, 'error_yaw': error_yaw,
            'min_particles': min_particles, 'max_particles': max_particles,
            'experiment_name': name, 'output_dir': arg('output_dir'),
        }],
    )

    # Wait for the simulation to be ready to start control nodes
    waiting = WaitForControllerConnection(target_driver=driver, nodes_to_start=spawners)
    actions = [webots, robot_state_publisher, footprint_publisher, map_server, amcl,
               lifecycle, supervisor, driver, experiment, waiting]
    if arg('use_rviz').lower() == 'true':
        actions.append(rviz)
    actions.append(RegisterEventHandler(OnProcessExit(
        target_action=experiment, on_exit=[EmitEvent(event=Shutdown(reason='Experiment finished'))],
    )))
    # This action will kill all nodes once the Webots simulation has exited
    actions.append(RegisterEventHandler(OnProcessExit(
        target_action=webots, on_exit=[EmitEvent(event=Shutdown(reason='Webots exited'))],
    )))
    return actions


def generate_launch_description():
    folder = get_package_share_directory('practice_localization')
    cases = ['initial_small_low', 'initial_small_high', 'initial_large_low',
             'initial_large_high', 'global_low', 'global_high']
    return LaunchDescription([
        DeclareLaunchArgument('experiment', default_value=cases[0], choices=cases),
        DeclareLaunchArgument('duration', default_value='180.0'),
        DeclareLaunchArgument('position_threshold', default_value='0.2'),
        DeclareLaunchArgument('angle_threshold', default_value='0.2'),
        DeclareLaunchArgument('confirmation_time', default_value='10.0'),
        DeclareLaunchArgument('output_dir', default_value=os.path.join(os.getcwd(), 'results')),
        DeclareLaunchArgument('mode', default_value='realtime', choices=['realtime', 'fast', 'pause']),
        DeclareLaunchArgument('gui', default_value='true', choices=['true', 'false']),
        DeclareLaunchArgument('use_rviz', default_value='true', choices=['true', 'false']),
        DeclareLaunchArgument('port', default_value='1234'),
        DeclareLaunchArgument('world_path', default_value=os.path.join(folder, 'worlds', 'numbers.wbt')),
        OpaqueFunction(function=start),
    ])
