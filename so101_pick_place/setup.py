import os
from glob import glob

from setuptools import setup

package_name = 'so101_pick_place'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*.xacro')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='so101_pick_place maintainers',
    maintainer_email='maintainers@example.com',
    description='SO-101 + OAK-D Pro red-tray pick and place',
    license='Proprietary',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'object_detector = so101_pick_place.object_detector:main',
            'object_pose_estimator = so101_pick_place.object_pose_estimator:main',
            'pick_place_controller = so101_pick_place.pick_place_controller:main',
            'so101_calibrate_zero = so101_pick_place.calibrate_zero:main',
            'oak_camera = so101_pick_place.oak_camera_node:main',
            'load_detector = so101_pick_place.load_detector:main',
        ],
    },
)
