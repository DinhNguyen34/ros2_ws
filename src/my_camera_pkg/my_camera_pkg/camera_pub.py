import rclpy
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