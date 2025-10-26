from setuptools import find_packages, setup
from glob import glob
import os


package_name = 'go2_robot'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name),[os.path.join(package_name, 'untitled.ui')]),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
        (os.path.join('share', package_name, 'urdf'), glob(os.path.join('urdf', '*'))),
        (os.path.join('share', package_name, 'dae'), glob(os.path.join('dae', '*'))),
        (os.path.join('share', package_name, 'meshes'), glob(os.path.join('meshes', '*'))),
        (os.path.join('share', package_name, 'config'), glob(os.path.join('config', '*'))),
        (os.path.join('share', package_name, 'calibration'), glob(os.path.join('calibration', '*'))),
        (os.path.join('share', package_name, 'yolo_models'), glob(os.path.join('yolo_models', '*'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='root@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'go2_driver_node = go2_robot.go2_driver_node:main',
            'go2_sport_ctrl_node = go2_robot.go2_sport_ctrl_node:main',
            'go2_realsense_transformer_node = go2_robot.go2_realsense_transformer_node:main',
            'go2_realsense_node = go2_robot.go2_realsense:main',
            'go2_image_lidar_overlay_node = go2_robot.image_lidar_overlay:main',
            'cupy_pointcloud_to_laserscan = go2_robot.cupy_pointcloud_to_laserscan:main',
            'qt_test = go2_robot.qt_test:main',
            'realsense_node = go2_robot.realsense_node:main',
        ],
    },
)
