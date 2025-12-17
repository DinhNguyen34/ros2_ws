import rclpy  # ROS 2 Python client library
from rclpy.node import Node    #Base class for ROS 2 nodes
from sensor_msgs.msg import Image      #Message type for images
from cv_bridge import CvBridge  #Bridge between ROS and OpenCV
import cv2  #OpenCV library for image processing

class CameraSubscriber(Node):
    def __init__(self):
        super().__init__('camera_subscriber')   #name of the node
        self.bridge = CvBridge()    #translation between ROS and OpenCV
        # Create a subscription to the /camera/image_raw topic
        self.subscription = self.create_subscription(
            Image,    #message type
            '/camera/image_raw',    #topic name
            self.image_callback,   #callback function, called when a message is received
            10  #queue size
            
        )
        self.get_logger().info('Camera subscriber started!')

    def image_callback(self, msg):
        """
        Callback function to handle incoming image messages.
        args:
            msg (Image): The incoming image message.
        """
        try:
            self.get_logger().info('Received image frame')
            # Convert ROS Image message to OpenCV image
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            # get info about the image
            height, width, channels = cv_image.shape
            # Overlay image info on the image
            text = f'Size: {width}x{height} |Frame: {msg.header.stamp.sec}.{msg.header.stamp.nanosec}'
            cv2.putText(cv_image, text, (10,30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,255, 0),2 )
            cv2.imshow('Camera', cv_image)
            cv2.waitKey(1)
        except Exception as e:
            self.get_logger.error('Error: '+ str(e))


def main(args = None):
    rclpy.init(args=args)
    node = CameraSubscriber()
    try:
        # Keep the node running to listen for incoming messages
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()


if __name__ == '__main__':
    main()