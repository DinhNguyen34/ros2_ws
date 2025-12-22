#!/usr/bin/env python3
"""
Gesture Controller Node - ROS 2 Version
Điều khiển robot dựa trên gesture được nhận dạng

Author: ROS Hand Gesture Recognition Team
License: MIT
ROS Version: ROS 2 (Humble/Iron/Jazzy)
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from std_msgs.msg import String, Int32
from geometry_msgs.msg import Twist, Point
import time


class GestureController(Node):
    """
    Class điều khiển robot dựa trên hand gestures - ROS 2
    """
    
    def __init__(self):
        """Khởi tạo controller node"""
        super().__init__('gesture_controller')
        
        # Declare parameters
        self.declare_parameter('linear_speed', 0.5)
        self.declare_parameter('angular_speed', 1.0)
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('enable_timeout', True)
        self.declare_parameter('timeout_seconds', 1.0)
        
        # Get parameters
        self.linear_speed = self.get_parameter('linear_speed').value
        self.angular_speed = self.get_parameter('angular_speed').value
        cmd_vel_topic = self.get_parameter('cmd_vel_topic').value
        self.enable_timeout = self.get_parameter('enable_timeout').value
        self.timeout_seconds = self.get_parameter('timeout_seconds').value
        
        # QoS Profile
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # Publishers
        self.cmd_vel_pub = self.create_publisher(
            Twist,
            cmd_vel_topic,
            10
        )
        
        # Subscribers
        self.gesture_sub = self.create_subscription(
            String,
            '/hand_gesture',
            self.gesture_callback,
            qos_profile
        )
        
        self.finger_count_sub = self.create_subscription(
            Int32,
            '/finger_count',
            self.finger_count_callback,
            qos_profile
        )
        
        self.hand_position_sub = self.create_subscription(
            Point,
            '/hand_position',
            self.position_callback,
            qos_profile
        )
        
        # State variables
        self.current_gesture = "UNKNOWN"
        self.finger_count = 0
        self.hand_position = Point()
        self.last_gesture_time = time.time()
        
        # Gesture to command mapping
        self.gesture_commands = {
            'OPEN_PALM': 'stop',
            'FIST': 'stop',
            'POINT': 'forward',
            'PEACE': 'backward',
            'THUMBS_UP': 'rotate_left',
            'THREE': 'rotate_right',
            'FOUR': 'increase_speed',
            'ONE': 'decrease_speed'
        }
        
        # Speed multiplier
        self.speed_multiplier = 1.0
        self.min_speed = 0.2
        self.max_speed = 2.0
        
        # Timer để check timeout
        if self.enable_timeout:
            self.timeout_timer = self.create_timer(
                0.1,  # Check every 100ms
                self.timeout_callback
            )
        
        self.get_logger().info('Gesture Controller initialized')
        self.get_logger().info(f'Linear speed: {self.linear_speed} m/s')
        self.get_logger().info(f'Angular speed: {self.angular_speed} rad/s')
        self.get_logger().info(f'Publishing to: {cmd_vel_topic}')
        self.get_logger().info('Gesture mappings:')
        for gesture, command in self.gesture_commands.items():
            self.get_logger().info(f'  {gesture} -> {command}')
    
    def gesture_callback(self, msg):
        """
        Callback nhận gesture từ detector
        
        Args:
            msg: String message chứa gesture name
        """
        # Update last gesture time
        self.last_gesture_time = time.time()
        
        # Parse gesture
        gesture_parts = msg.data.split(': ')
        if len(gesture_parts) == 2:
            hand = gesture_parts[0]
            gesture = gesture_parts[1]
            
            self.current_gesture = gesture
            self.get_logger().info(f'Received gesture: {hand} - {gesture}')
            
            # Execute command
            self.execute_gesture_command(gesture)
    
    def finger_count_callback(self, msg):
        """
        Callback nhận số lượng ngón tay
        
        Args:
            msg: Int32 message
        """
        self.finger_count = msg.data
    
    def position_callback(self, msg):
        """
        Callback nhận vị trí bàn tay
        
        Args:
            msg: Point message
        """
        self.hand_position = msg
    
    def timeout_callback(self):
        """
        Timer callback để check timeout và auto-stop robot
        """
        if self.enable_timeout:
            time_since_last_gesture = time.time() - self.last_gesture_time
            
            if time_since_last_gesture > self.timeout_seconds:
                # No gesture for timeout_seconds → stop robot
                twist = Twist()
                self.cmd_vel_pub.publish(twist)
    
    def execute_gesture_command(self, gesture):
        """
        Thực thi lệnh điều khiển dựa trên gesture
        
        Args:
            gesture: Tên của gesture
        """
        # Lấy command từ mapping
        command = self.gesture_commands.get(gesture, 'unknown')
        
        # Tạo Twist message
        twist = Twist()
        
        if command == 'stop':
            # Dừng robot
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            self.get_logger().info('Command: STOP')
            
        elif command == 'forward':
            # Di chuyển về phía trước
            twist.linear.x = self.linear_speed * self.speed_multiplier
            twist.angular.z = 0.0
            self.get_logger().info(f'Command: FORWARD (speed: {twist.linear.x:.2f})')
            
        elif command == 'backward':
            # Di chuyển lùi
            twist.linear.x = -self.linear_speed * self.speed_multiplier
            twist.angular.z = 0.0
            self.get_logger().info(f'Command: BACKWARD (speed: {twist.linear.x:.2f})')
            
        elif command == 'rotate_left':
            # Quay trái
            twist.linear.x = 0.0
            twist.angular.z = self.angular_speed * self.speed_multiplier
            self.get_logger().info(f'Command: ROTATE LEFT (speed: {twist.angular.z:.2f})')
            
        elif command == 'rotate_right':
            # Quay phải
            twist.linear.x = 0.0
            twist.angular.z = -self.angular_speed * self.speed_multiplier
            self.get_logger().info(f'Command: ROTATE RIGHT (speed: {twist.angular.z:.2f})')
            
        elif command == 'increase_speed':
            # Tăng tốc độ
            self.speed_multiplier = min(self.speed_multiplier + 0.1, self.max_speed)
            self.get_logger().info(f'Speed increased: {self.speed_multiplier:.1f}x')
            return  # Không publish twist
            
        elif command == 'decrease_speed':
            # Giảm tốc độ
            self.speed_multiplier = max(self.speed_multiplier - 0.1, self.min_speed)
            self.get_logger().info(f'Speed decreased: {self.speed_multiplier:.1f}x')
            return  # Không publish twist
        
        else:
            self.get_logger().warn(f'Unknown command: {command}')
            return
        
        # Publish velocity command
        self.cmd_vel_pub.publish(twist)


def main(args=None):
    """Main entry point"""
    rclpy.init(args=args)
    
    try:
        controller = GestureController()
        rclpy.spin(controller)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f'Exception in gesture controller: {e}')
    finally:
        # Shutdown
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()