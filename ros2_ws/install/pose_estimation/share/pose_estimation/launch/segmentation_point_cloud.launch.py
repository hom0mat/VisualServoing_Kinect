import os

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_vision = get_package_share_directory('pose_estimation')

    config_file = os.path.join(pkg_vision, 'config', 'segmentation_point_cloud_config.yaml')
    intrinsics_file = os.path.join(pkg_vision, 'config', 'kinect_calibration.yaml')

    image_segmentation_node = Node(
        package='pose_estimation',
        executable='image_segmentation',
        name='image_segmentation',
        parameters=[config_file],
        output='screen'
    )

    point_cloud_generator_node = Node(
        package='pose_estimation',
        executable='point_cloud_generator',
        name='point_cloud_generator',
        parameters=[
            config_file,
            {'camera_intrinsics_file': intrinsics_file}
        ],
        output='screen'
    )

    pose_estimator_node = Node(
        package='pose_estimation',
        executable='pose_estimator',
        name='pose_estimator',
        output='screen'
    )

    return LaunchDescription([
        image_segmentation_node,
        point_cloud_generator_node,
        pose_estimator_node
    ])
