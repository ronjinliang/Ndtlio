import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, RegisterEventHandler, ExecuteProcess, TimerAction
from launch_ros.actions import Node
from launch.substitutions import TextSubstitution, LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python import get_package_share_directory, get_package_prefix
from launch.event_handlers import OnProcessStart

def generate_launch_description():

    # 1. 掩码地图 Server
    map_server_node = Node(
            package='NdtLIO',
            executable='test_ros2_lio',
            output='screen',
            remappings=[("/scan_point_cloud", "/ndt_lio/scan_point_cloud"), ("/ndt_odom", "/ndt_lio/ndt_odom")]
        )
    
    return LaunchDescription([
        map_server_node,
    ])