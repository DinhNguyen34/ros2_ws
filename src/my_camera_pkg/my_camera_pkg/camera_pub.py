"""import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np


class CameraPublisher(Node):
    def __init__(self):
        super().__init__('camera_publisher')
        self.publisher = self.create_publisher(Image, '/camera/image_raw', 10)
        self.timer = self.create_timer(0.033,self.timer_callback)
        self.bridge = CvBridge()
        self.frame_count = 0
        self.get_logger().info('Camera publisher started!')

    def timer_callback(self):
        img = np.zeros((480,640,3), dtype=np.uint8)
        cv2.putText(img, f'Frame: {self.frame_count}', (50, 240), cv2.FONT_HERSHEY_SIMPLEX, 2, (255, 255, 255),3)

        msg = self.bridge.cv2_to_imgmsg(img,encoding='bgr8')
        self.publisher.publish(msg)
        self.frame_count += 1

def main(args = None):
    rclpy.init(args=args)
    node = CameraPublisher()
    try: 
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
    """


import rclpy  # ROS 2 Python client library
from rclpy.node import Node    #Base class for ROS 2 nodes
from sensor_msgs.msg import Image      #Message type for images
from cv_bridge import CvBridge  #Bridge between ROS and OpenCV
import cv2  #OpenCV library for image processing

class CameraPublisher(Node):
    def __init__(self):
        super().__init__('camera_publisher')    #name of the node
        # open the default camera
        self.cap = cv2.VideoCapture(0)
        # Check if the camera opened successfully
        if not self.cap.isOpened():
            self.get_logger().error('Cannot open camera')
            raise Exception('Failed to open camera')
        
        # set resolution
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH,640)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT,480)
        
        self.bridge = CvBridge()    #translation between ROS and OpenCV
        self.publisher = self.create_publisher(Image, '/camera/image_raw', 10)   #create publisher to publish images to /camera/image_raw topic

        # timer 30FPS
        self.timer = self.create_timer(1.0/30.0, self.timer_callback)
        self.get_logger().info('Camera publisher staerted!')

    def timer_callback(self):
        # read frame from camera
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().error('Failed to capture image')
            return
        else:
            # convert OpenCV image to ROS Image message
            msg = self.bridge.cv2_to_imgmsg(frame, encoding = 'bgr8')
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = 'camera_frame'

            self.publisher.publish(msg)
            self.get_logger().info('Captured image frame')

    def __del__(self):
        # release the camera when the node is destroyed
        if hasattr(self, 'cap') and self.cap.isOpened():
            self.cap.release()

def main(args = None):
    rclpy.init(args=args)
    node = CameraPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
if __name__ == '__main__':
    main()