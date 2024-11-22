import os

import ament_index_python

from launch import LaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch_ros.actions import Node, SetRemap

# Path to the launch files and directories that we will use
_MICROSTRAIN_LAUNCH_FILE = os.path.join(ament_index_python.packages.get_package_share_directory('microstrain_inertial_driver'), 'launch', 'microstrain_launch.py')
_GX5_25_PARAMS_FILE = os.path.join(ament_index_python.packages.get_package_share_directory('microstrain_inertial_examples'), 'config', 'gx5_25', 'gx5_25.yml')
_RVIZ_DISPLAY_FILE = os.path.join(ament_index_python.packages.get_package_share_directory('microstrain_inertial_examples'), 'config', 'gx5_25', 'display.rviz')

def generate_launch_description():
  car_name = os.getenv('F1TENTH_CAR_NAME', 'f1tenth')  # 기본값은 'f1tenth'

  return LaunchDescription([
    # Microstrain node
    IncludeLaunchDescription(
      PythonLaunchDescriptionSource(_MICROSTRAIN_LAUNCH_FILE),
      launch_arguments={
        'configure': 'true',
        'activate': 'true',
        'params_file': _GX5_25_PARAMS_FILE,
        'namespace': '/gx5',
        # 'node_name': 'microstrain_inertial_driver_node'
      }.items()
    ),

    # In this example we have no way to publish an actual map transform, so just publish a static one so we can display data on rviz
    # If integrating into an existing system, this should be replaced with a navigation solution
    # Node(
    #   package='tf2_ros',
    #   executable='static_transform_publisher',
    #   output='screen',
    #   arguments=[
    #       "0", "0", "0", "0", "0", "0", "map", "base_link"
    #   ]
    # ),

    # Publish a static transform for where the GX5-25 is mounted on base_link.
    # Unless the GX5-25 is mounted exactly at base_link, you should change this to be accurate to your setup
    # Node(
    #   package='tf2_ros',
    #   executable='static_transform_publisher',
    #   output='screen',
    #   arguments=[
    #       "0", "0", "0", "0", "0", "0", "base_link", "gx5_25_link"
    #   ]
    # ),

    # # Run rviz to view the state of the application
    # Node(
    #   package='rviz2',
    #   executable='rviz2',
    #   output='screen',
    #   arguments=[
    #     '-d', _RVIZ_DISPLAY_FILE
    #   ]
    # ),
  ])