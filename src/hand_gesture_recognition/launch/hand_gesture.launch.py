#!/usr/bin/env python3
"""
Launch file cho Hand Gesture Recognition System - ROS 2
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch_ros.actions import Node
from launch.conditions import IfCondition

def generate_launch_description():
    """Generate launch description"""
    
    # Declare launch arguments
    use_webcam_arg = DeclareLaunchArgument(
        'use_webcam',
        default_value='true',
        description='Use webcam instead of ROS camera topic'
    )
    
    camera_id_arg = DeclareLaunchArgument(
        'camera_id',
        default_value='0',
        description='Camera ID for webcam (0, 1, 2, ...)'
    )
    
    linear_speed_arg = DeclareLaunchArgument(
        'linear_speed',
        default_value='0.5',
        description='Default linear speed (m/s)'
    )
    
    angular_speed_arg = DeclareLaunchArgument(
        'angular_speed',
        default_value='1.0',
        description='Default angular speed (rad/s)'
    )
    
    enable_controller_arg = DeclareLaunchArgument(
        'enable_controller',
        default_value='true',
        description='Enable robot controller node'
    )
    
    cmd_vel_topic_arg = DeclareLaunchArgument(
        'cmd_vel_topic',
        default_value='/cmd_vel',
        description='Topic name for velocity commands'
    )
    
    enable_timeout_arg = DeclareLaunchArgument(
        'enable_timeout',
        default_value='true',
        description='Enable auto-stop timeout'
    )
    
    timeout_seconds_arg = DeclareLaunchArgument(
        'timeout_seconds',
        default_value='1.0',
        description='Timeout in seconds before auto-stop'
    )
    
    # Gesture Detector Node
    gesture_detector_node = Node(
        package='hand_gesture_recognition',
        executable='gesture_detector',
        name='gesture_detector',
        output='screen',
        parameters=[{
            'use_webcam': LaunchConfiguration('use_webcam'),
            'camera_id': LaunchConfiguration('camera_id'),
            'min_detection_confidence': 0.7,
            'min_tracking_confidence': 0.5,
            'max_num_hands': 2,
        }]
    )
    
    # Gesture Controller Node
    gesture_controller_node = Node(
        package='hand_gesture_recognition',
        executable='gesture_controller',
        name='gesture_controller',
        output='screen',
        parameters=[{
            'linear_speed': LaunchConfiguration('linear_speed'),
            'angular_speed': LaunchConfiguration('angular_speed'),
            'cmd_vel_topic': LaunchConfiguration('cmd_vel_topic'),
            'enable_timeout': LaunchConfiguration('enable_timeout'),
            'timeout_seconds': LaunchConfiguration('timeout_seconds'),
        }],
        condition=IfCondition(LaunchConfiguration('enable_controller'))

        )
    
    
    # Log info
    log_info = LogInfo(
        msg=[
            '=== Hand Gesture Recognition System ===\n',
            'Detector: ', LaunchConfiguration('use_webcam'), '\n',
            'Controller: ', LaunchConfiguration('enable_controller'), '\n',
            'Speed: ', LaunchConfiguration('linear_speed'), ' m/s\n',
            '========================================'
        ]
    )
    
    return LaunchDescription([
        # Arguments
        use_webcam_arg,
        camera_id_arg,
        linear_speed_arg,
        angular_speed_arg,
        enable_controller_arg,
        cmd_vel_topic_arg,
        enable_timeout_arg,
        timeout_seconds_arg,
        
        # Nodes
        gesture_detector_node,
        gesture_controller_node,
        
        # Info
        log_info,
    ])