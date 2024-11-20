import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, EnvironmentVariable
from launch_ros.actions import Node

def generate_launch_description():
    # 공통 설정 파일
    package_dir = get_package_share_directory('vesc_ackermann')
    config_file = os.path.join(package_dir, 'config', 'config.yaml')

    # VESC Driver 설정 파일
    vesc_config = os.path.join(
        get_package_share_directory('vesc_driver'),
        'params',
        'vesc_config.yaml'
    )

    # Launch Arguments
    config_arg = DeclareLaunchArgument(
        name='config',
        default_value=vesc_config,
        description='VESC yaml configuration file.'
    )
    debug_arg = DeclareLaunchArgument(
        name='debug',
        default_value='false',
        description='Launch in debug mode'
    )
    # Launch Prefix 설정 (디버그 모드)
    launch_prefix = ['xterm -e gdb --args'] if LaunchConfiguration('debug') == 'true' else []

    # VESC Driver Node
    vesc_driver_node = Node(
        package='vesc_driver',
        executable='vesc_driver_node',
        name='vesc_driver_node',
        parameters=[LaunchConfiguration('config')],
        output='screen'
    )

    # Ackermann to VESC Node
    ackermann_to_vesc_node = Node(
        package='vesc_ackermann',
        executable='ackermann_to_vesc_node',
        name='ackermann_to_vesc_node',
        output='screen',
        parameters=[config_file],
        prefix=launch_prefix
    )

    # VESC to Odom Node
    vesc_to_odom_node = Node(
        package='vesc_ackermann',
        executable='vesc_to_odom_node',
        name='vesc_to_odom_node',
        output='screen',
        parameters=[config_file],
        prefix=launch_prefix
    )

    # Ackermann Manager Node
    ackermann_manager_node = Node(
        package='vesc_ackermann',
        executable='ackermann_manager_node',
        name='ackermann_manager_node',
        output='screen',
        parameters=[config_file]
    )

    # Launch Description
    return LaunchDescription([
        config_arg,
        debug_arg,
        vesc_driver_node,
        ackermann_to_vesc_node,
        vesc_to_odom_node,
        ackermann_manager_node
    ])
