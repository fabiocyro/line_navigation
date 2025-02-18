import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from geometry_msgs.msg import Twist
import cv2
from cv_bridge import CvBridge
import numpy as np
import time

# Initial values for linear speed and KP
INITIAL_LINEAR_SPEED = 0.2
INITIAL_KP = 1.5 / 100

# Offset from the left edge of the image where the robot should keep the line
LEFT_OFFSET = 160  # Adjust this value based on your robot's camera resolution

class ImageSubscriber(Node):
    def __init__(self):
        super().__init__('image_subscriber')
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.subscription = self.create_subscription(
            CompressedImage,
            '/image_raw/compressed',
            self.listener_callback,
            10)
        self.subscription
        self.bridge = CvBridge()

        # Sliders for tuning
        cv2.namedWindow("Tuning")
        cv2.createTrackbar("Low Yellow H", "Tuning", 20, 255, lambda x: None)
        cv2.createTrackbar("High Yellow H", "Tuning", 30, 255, lambda x: None)
        cv2.createTrackbar("Linear Speed", "Tuning", int(INITIAL_LINEAR_SPEED * 100), 100, lambda x: None)
        cv2.createTrackbar("KP", "Tuning", int(INITIAL_KP * 1000), 1000, lambda x: None)
        cv2.createTrackbar("ROI Height", "Tuning", 0, 480, lambda x: None)  # Trackbar for ROI height

        # Variables for robustness
        self.last_valid_cmd = Twist()
        self.last_valid_time = time.time()
        self.timeout_duration = 5.0  # 5 seconds timeout

    def listener_callback(self, data):
        # Convert ROS Image message to OpenCV image
        error = 0
        kp = INITIAL_KP
        linear_speed = INITIAL_LINEAR_SPEED
        current_frame = self.bridge.compressed_imgmsg_to_cv2(data, desired_encoding='bgr8')

        # Get the ROI height from the trackbar
        roi_height = cv2.getTrackbarPos("ROI Height", "Tuning")

        # Crop the image to the ROI (from roi_height to the bottom)
        if roi_height < current_frame.shape[0]:  # Ensure ROI height is within the image height
            cropped_frame = current_frame[roi_height:, :]
        else:
            cropped_frame = current_frame  # Use the full frame if ROI height is invalid

        # Convert BGR to HSV
        hsv_image = cv2.cvtColor(cropped_frame, cv2.COLOR_BGR2HSV)

        # Define range of yellow color in HSV
        lower_yellow = np.array([cv2.getTrackbarPos("Low Yellow H", "Tuning"), 100, 100])  # Lower bound of yellow color
        upper_yellow = np.array([cv2.getTrackbarPos("High Yellow H", "Tuning"), 255, 255])  # Upper bound of yellow color

        # Create a binary mask
        yellow_mask = cv2.inRange(hsv_image, lower_yellow, upper_yellow)

        # Apply the mask to the cropped image
        yellow_segmented_image = cv2.bitwise_and(cropped_frame, cropped_frame, mask=yellow_mask)

        # Detect line and get its centroid
        line = self.get_contour_data(yellow_mask)

        # Move depending on detection
        cmd = Twist()
        _, width, _ = cropped_frame.shape
        if line:
            x = line['x']

            # Calculate error such that the robot keeps the line to the left
            error = x - LEFT_OFFSET

            # Get the current values of linear speed and KP from the trackbars
            linear_speed = cv2.getTrackbarPos("Linear Speed", "Tuning") / 100.0
            kp = cv2.getTrackbarPos("KP", "Tuning") / 1000.0

            cmd.linear.x = linear_speed
            # Draw the centroid on the cropped image
            cv2.circle(yellow_segmented_image, (line['x'], line['y']), 5, (0, 0, 255), 7)

            # Update the last valid command and time
            self.last_valid_cmd = cmd
            self.last_valid_time = time.time()

        else:
            # If no line is detected, check if the timeout has elapsed
            if time.time() - self.last_valid_time < self.timeout_duration:
                # Use the last valid command
                cmd = self.last_valid_cmd
            else:
                # Stop the robot if the timeout has elapsed
                cmd = Twist()

        # Determine the speed to turn and get the line in the desired position
        cmd.angular.z = float(error) * -kp
        print("Error: {} | Angular Z: {}, Linear Speed: {}, KP: {}".format(error, cmd.angular.z, linear_speed, kp))

        # Send the command to execute
        self.publisher.publish(cmd)

        # Display the segmented image
        cv2.imshow("Yellow Segmented Image", yellow_segmented_image)
        cv2.waitKey(1)

    def get_contour_data(self, mask):
        """
        Return the centroid of the largest contour in the binary image 'mask' (the line)
        """
        # Constants
        MIN_AREA_TRACK = 1100  # Minimum area for track marks

        # Get a list of contours
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)

        line = {}

        for contour in contours:
            M = cv2.moments(contour)

            if M['m00'] > MIN_AREA_TRACK:
                # Contour is part of the track
                line['x'] = int(M["m10"] / M["m00"])
                line['y'] = int(M["m01"] / M["m00"])
        return line

def main(args=None):
    rclpy.init(args=args)
    image_subscriber = ImageSubscriber()
    rclpy.spin(image_subscriber)
    image_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
