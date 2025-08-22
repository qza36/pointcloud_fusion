import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    pkg_dir = get_package_share_directory('pointcloud_fusion')
    config_file = os.path.join(pkg_dir, 'config', 'fusion_params.yaml')

    cloud_fusion_node = Node(
        package='pointcloud_fusion',
        executable='fusion_node',
        name='fusion_node',
        output='screen',
        parameters=[config_file]
    )

    return LaunchDescription([
        cloud_fusion_node
    ])
