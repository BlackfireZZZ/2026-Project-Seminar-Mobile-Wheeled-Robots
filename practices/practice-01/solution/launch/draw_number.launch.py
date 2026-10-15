"""Start turtlesim, create the two turtles, then draw the number."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    simulator = Node(package='turtlesim', executable='turtlesim_node')
    prepare = Node(package='practice01_digits', executable='prepare', output='screen')
    left = Node(
        package='practice01_digits', executable='draw_digit', name='draw_left',
        parameters=[{'turtle': 'digit_left', 'digit': LaunchConfiguration('left_digit'),
                     'origin_x': 1.6, 'origin_y': 3.1}], output='screen',
    )
    right = Node(
        package='practice01_digits', executable='draw_digit', name='draw_right',
        parameters=[{'turtle': 'digit_right', 'digit': LaunchConfiguration('right_digit'),
                     'origin_x': 6.5, 'origin_y': 3.1}], output='screen',
    )
    return LaunchDescription([
        DeclareLaunchArgument('left_digit', default_value='0'),
        DeclareLaunchArgument('right_digit', default_value='2'),
        simulator,
        prepare,
        RegisterEventHandler(OnProcessExit(target_action=prepare, on_exit=[left, right])),
    ])
