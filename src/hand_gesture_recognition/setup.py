from setuptools import setup
import os
from glob import glob

package_name = 'hand_gesture_recognition'

setup(
    name=package_name,
    version='2.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Launch files
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='nguyendinh',
    maintainer_email='nguyenpazo123@gmail.com',
    description='Hand gesture recognition using MediaPipe and ROS 2',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'gesture_detector = hand_gesture_recognition.gesture_detector:main',
            'gesture_controller = hand_gesture_recognition.gesture_controller:main',
        ],
    },
)