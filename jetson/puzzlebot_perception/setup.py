from setuptools import setup
import os
from glob import glob

package_name = 'puzzlebot_perception'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='student',
    maintainer_email='student@university.edu',
    description='HSV vision node for Puzzlebot visual servoing',
    license='MIT',
    entry_points={
        'console_scripts': [
            'vision_node = puzzlebot_perception.vision_node:main',
            'hsv_calibrator = puzzlebot_perception.hsv_calibrator:main',
        ],
    },
)
