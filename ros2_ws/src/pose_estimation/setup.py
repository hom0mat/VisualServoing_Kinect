from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'pose_estimation'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'),
            glob(os.path.join('config', '*'))),
        (os.path.join('share', package_name, 'launch'),
            glob(os.path.join('launch', '*.launch.py'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Mateo Sanchez',
    maintainer_email='neronf123@gmail.com',
    description='Visual servoing package for X-ARM robot with Azure Kinect.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'image_segmentation = pose_estimation.image_segmentation:main',
            'point_cloud_generator = pose_estimation.point_cloud_generator:main',
            'point_cloud_visualizer = pose_estimation.point_cloud_visualizer:main',
            'pose_estimator = pose_estimation.pose_estimator:main',
        ],
    },
)
