import os
import sys
from glob import glob
import launch
import launch_ros.actions
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

def generate_launch_description():
    share_dir = get_package_share_directory('wit_node')
    rviz_config_file = os.path.join(share_dir, 'rviz2','wit.rviz')
    params = [
        launch.actions.DeclareLaunchArgument('port',
                                            default_value="/dev/ttyUSB0"),
        launch.actions.DeclareLaunchArgument('baud_rate',
                                            default_value="115200"),
        launch.actions.DeclareLaunchArgument('frame_id',
                                            default_value="/imu_link"),
        launch.actions.DeclareLaunchArgument('publish_hz',
                                            default_value="10.0"),
    ]
    wit_node = launch_ros.actions.Node(
        package="wit_node", executable="wit_node",
        parameters=[{
            "port": launch.substitutions.LaunchConfiguration('port'),
            "baud_rate": launch.substitutions.LaunchConfiguration('baud_rate'),
            "frame_id": launch.substitutions.LaunchConfiguration('frame_id'),
            "publish_hz": launch.substitutions.LaunchConfiguration('publish_hz')
        }],
    )
    rviz2_node = launch_ros.actions.Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config_file],
    )
    return launch.LaunchDescription(
        params + 
        [wit_node, rviz2_node]
    )