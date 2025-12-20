"""
Docstring for hand_gesture_recognition.gesture_node
"""
import rclpy
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy     # Import QoS classes from rclpy
from rclpy.node import Node         # Import Node class from rclpy
from std_msgs.msg import String, Int32     # Import String message type
from sensor_msgs.msg import Image    # Import Image message type
from cv_bridge import CvBridge, CvBridgeError     # Import CvBridge to convert ROS Image messages to OpenCV images
import cv2
import numpy as np                  # Import NumPy for numerical operations
import mediapipe as mp      # Import MediaPipe for hand tracking
from geometry_msgs.msg import Point     # Import Point message type


class HandGesture():
    """
    Docstring for HandGesture
    """
    def __init__(self):
        super().__init__('hand_gesture_node')

        #parameters
        self.declare_parameter('use_camera', True )
        self.declare_parameter('width',640)
        self.declare_parameter('height',480)
        self.declare_parameter('camera_id',0)
        self.declare_parameter('min_detection_confidence', 0.5)
        self.declare_parameter('min_tracking_confidence', 0.5)
        self.declare_parameter('max_num_hands', 2)

        # get parameters
        use_camera = self.get_parameter('use_camera').value
        width = self.get_parameter('width').value
        height = self.get_parameter('height').value
        camera_id = self.get_parameter('camera_id').value
        min_detection = self.get_parameter('min_detection_confidence').value
        min_tracking = self.get_parameter('min_tracking_confidence').value
        max_hands = self.get_parameter('max_num_hands').value


        # intialize MediaPipe Hands
        self.mp_hands = mp.solution.hands
        self.mp_drawing = mp.solutions.drawing_utils
        self.mp_drawing_styles = mp.solutions.drawing_styles
        # Create a MediaPipe Hands object
        self.hands = self.mp_hands.Hands(
            static_image_node = False,
            max_num_hands = max_hands,
            min_detection_confidence = min_detection,
            min_tracking_confidence = min_tracking

        )
        # Initialize CvBridge
        self.bridge = CvBridge()

        #Qos profile for real-time video processing
        qos_profile = QoSProfile(
            reliability = ReliabilityPolicy.BEST_EFFORT,
            history = HistoryPolicy.KEEP_LAST,
            depth = 1
        )
        #publisher for gesture recognition results
        self.gesture_pub = self.create_publisher(String, '/hand_gesture', 10)
        self.count_pub = self.create_publisher(Int32, '/hand_count', 10)
        self.hand_pos_pub = self.create_publisher(Point, '/hand_position',10)
        self.annotated_image_pub = self.create_publisher(Image,'/annotated_image', qos_profile)

        #subscriber for image data
        if not self.use_camera:
            self.image_sub = self.create_subscription(
                Image,
                '/camera/image_raw',
                self.image_callback,
                qos_profile
            )
        #webcam setup
        if self.use_camera:
            self.cap = cv2.VideoCapture(camera_id)
            if not self.cap.isOpened():
                self.get_logger().error(f'cannot open camera {camera_id}')
                raise RuntimeError('camera inititalization failed')
            
            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
            self.timer =self.create_timer(1.0/30.0, self.timer_callback)  # 30 Hz

        else:
            self.cap = None
        
        self.get_logger().info('Hand Gesture Node has been started.')
        self.get_logger().info(f'using_camera: {self.use_camera}')
        self.get_logger().info(f'Max hands: {max_hands}')

    def count_fingers(self, hand_landmarks, handedness):
        """
        Docstring for count_fingers
        
        args:
            hand_landmarks: landmarks of the detected hand
            handedness: 'Left' or 'Right' hand
        returns:
            int: number of fingers extended
        """
        landmarks = hand_landmarks.landmark

        finger_tips = [4, 8, 12, 16, 20]
        finger_pips = [3, 6, 10, 14, 18]
        fingers = []

        # check thumb
        
        if handedness == "Right":
            if landmarks[finger_tips[0]].x < landmarks[finger_pips[0]].x:
                fingers.append(1)
            else:
                fingers.append(0)
        else: 
            if landmarks[finger_tips[0]].x > landmarks[finger_pips[0]].x:
                fingers.append(1)
            else:
                fingers.append(0)

        # check other four fingers
        for tip, pip in zip(finger_tips[1:], finger_pips[1:]):
            if landmarks[tip].y < landmarks[pip].y:
                fingers.append(1)
            else:
                fingers.append(0)
        return sum(fingers)
    
    #
    def recognize_gesture(self, hand_landmarks, handedness, hand_count ):
        """
        Recognize hand gesture based on landmarks
        args:
            hand_landmarks: landmarks of the detected hand
            handedness: 'Left' or 'Right' hand
            hand_count: total number of hands detected
        Returns:
            str: recognized gesture
        """

        landmarks = hand_landmarks.landmark
        if hand_count == 0:
            gesture = "Fist"
        elif hand_count == 1:
            if landmarks[8].y < landmarks[6].y:
                gesture = "Pointing Up"
            elif landmarks[4].y < landmarks[3].y:
                gesture = "Thumbs Up"
            else :
                gesture = "One"
        elif hand_count == 2:
            index_up = landmarks[8].y < landmarks[6].y
            middle_up = landmarks[12].y < landmarks[10].y
            if index_up and middle_up:
                gesture = "PEACE"
            else:
                gesture = "TWO"
        elif hand_count == 3:
            gesture = "THREE"
        elif hand_count == 4:
            gesture = "FOUR"
        elif hand_count == 5:
            gesture = "OPEN_PALM"
        else:
            gesture = "UNKNOWN"

        return gesture
    
    # caculator position land
    def get_hand_center(self, hand_landmarks, image_shape):
        """
        Caculator position center hand

        args: 
            hand_landmarks: landmarks of hand
            image_shape: shape of image ( height, width, channels)
        returns:
            point: center coordinates
        """

        # uses wrist as the center
        wrist = hand_landmarks.landmark[0]

        # convert from normalized coordinates to pixel coordinates
        h, w, _ = image_shape
        point = Point()
        point.x = float(wrist.x * w)
        point.y = float(wrist.y * h)
        point.z = float(wrist.z)   # depth (relative)

        return point
    def image_callback(self, msg):
        """
        Callback handle image from ROS topic 
        args:
            msg: ROS image message
        """

        try:
            #Convert ROS image to OpenCV image
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            self.process_frame(cv_image)
        except CvBridgeError as e:
            self.get_logger().error(f"CvBridge Error: {e}")

    def timer_callback(self):
        """Timer callback to read webcam frames"""
        if self.cap is not None:
            ret, frame = self.cap.read()
            if ret:
                self.process_frame(frame)
            else: 
                self.get_logger().warn('Failed to read frame from camera')
    
    def process_frame(self, frame):
        """
        handle one frame image to dectect gestures

        args: 
            frame: Opencv Image (BRG format)
        """
        # convert BGR to RGB
        rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

        # process with MediaPipe

        results = self.hand.process(rgb_frame)

        #create annotated image
        annotated_image = frame.copy()

        if results.multi_hand_landmarks:
            for hand_idx, hand_landmarks in enumerate(results.multi_hand_landmarks):
                # get handedness
                handedness = results.multi_handedness[hand_idx].classification[0].label

                # draw landmarks
                self.mp_drawing.draw_landmarks(
                    annotated_image,
                    hand_landmarks,
                    self.mp_hands.HAND_CONNECTIONS,
                    self.mp_drawing_styles.get_default_hand_landmarks_style(),
                    self.mp_drawing_styles.get_default_hand_connections_style()
                )

                # counting finger
                hand_count = self.count_fingers(hand_landmarks, handedness)
                #recognition gesture
                gesture = self.recognize_gesture(hand_landmarks, handedness, hand_count)
                # get location 
                hand_position = self.get_hand_center(hand_landmarks, frame.shape)

                #publish
                gesture_msg = String()
                gesture_msg.data = f"{handedness}: {gesture}"
                self.gesture_pub.publish(gesture_msg)

                finger_msg = Int32()
                finger_msg.data = hand_count
                self.count_pub.publish(finger_msg)

                self.hand_pos_pub.publish(hand_position)

                # draw text 
                cv2.putText(
                    annotated_image,
                    f"{handedness}: {gesture} ({hand_count} fingers)",
                    (10, 30 + hand_idx *30),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.7,
                    (0, 255, 0),
                    2
                )

                self.get_logger().info(
                    f'Detected: {handedness} - {gesture} - {hand_count} fingers'
                )
        # publish annotated image
        try:
            annotated_msg = self.bridge.cv2_to_imgmsg(annotated_image, "bgr8")
            self.annotated_image_pub.publish(annotated_msg)

        except CvBridgeError as e:
            self.get_logger().error(f'CvBridge Error: {e}')

        # display if use webcam
        if self.use_webcam:
            cv2.imshow('Hand Gesture Recognition', annotated_image)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                self.get_logger().info('User request shutdown')
                rclpy.shutdown()

    def cleanup(self):
        """
        Cleanup resources
        """
        if self.cap is not None:
            self.cap.release()
        cv2.destroyAllWindows()
        self.hands.close()

def main (args = None):
    """Main entry point"""

    rclpy.init(args = args)
    try:
        detector = HandGesture()
        rclpy.spin(detector)

    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f'Exception in gesture detector: {e}')
    finally: 
        if 'detection' in locals():
            detector.cleanup()

        #shutdown
        if rclpy.ok():
            rclpy.shutdown()



if __name__ == '__main__':
    main()