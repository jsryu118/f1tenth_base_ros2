import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    package_dir = get_package_share_directory('vesc_ackermann')
    config_file = os.path.join(package_dir, 'config', 'config.yaml')

    debug_arg = DeclareLaunchArgument('debug', default_value='false', description='Launch in debug mode')
    launch_prefix = ['xterm -e gdb --args'] if LaunchConfiguration('debug') == 'true' else []

    # VESC to Odom Node
    vesc_to_odom_node = Node(
        package='vesc_ackermann',
        executable='vesc_to_odom_node',
        name='vesc_to_odom_node',
        output='screen',
        parameters=[config_file, {'global_parameters': '/global_parameters'}],
        prefix=launch_prefix
    )

    return LaunchDescription([
        debug_arg,
        vesc_to_odom_node
    ])
