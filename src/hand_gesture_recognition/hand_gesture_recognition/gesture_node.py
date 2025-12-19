"""
Docstring for hand_gesture_recognition.gesture_node
"""
import rclpy
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy     # Import QoS classes from rclpy
from rclpy.node import Node         # Import Node class from rclpy
from std_msgs.msg import String, Int32     # Import String message type
from sensor_msgs.msg import Image    # Import Image message type
from cv_bridge import CvBridge      # Import CvBridge to convert ROS Image messages to OpenCV images
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
            if landmarks[finger_tips[0]].x > landmarks[finger_pis[0]].x:
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
    

