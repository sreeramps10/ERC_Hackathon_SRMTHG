#!/usr/bin/env python3
#You need to name this node "color_detector"

import rospy
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from std_msgs.msg import String

class ColorDetectionNode:
    def __init__(self):
        rospy.init_node('color_detector')
        self.bridge = CvBridge()
        self.task_status_pub = rospy.Publisher('/task_status', String, queue_size=10)

    def run(self):
        rospy.Subscriber('/camera/rgb/image_raw', Image, self.image_callback)
        rospy.spin()

    def image_callback(self, msg):
        # Convert ROS image to OpenCV format
        cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")

        # Detect colors and publish task status
        color_detected = self.detect_color(cv_image)
        self.publish_task_status(color_detected)

    def detect_color(self, image):
        # Convert the image to the HSV color space
        hsv_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        # Define color ranges for different colored cones (You may need to adjust these values)
        blue_lower = (90, 100, 100)
        blue_upper = (120, 255, 255)
        red_lower = (0, 100, 100)
        red_upper = (10, 255, 255)

        # Threshold the image to extract specific colors
        mask_blue = cv2.inRange(hsv_image, blue_lower, blue_upper)
        mask_red = cv2.inRange(hsv_image, red_lower, red_upper)

        # Count the number of blue and red pixels
        blue_pixel_count = cv2.countNonZero(mask_blue)
        red_pixel_count = cv2.countNonZero(mask_red)

        # Determine the detected color based on the pixel count threshold (You may need to adjust these values)
        if blue_pixel_count > 500:  # Adjust the threshold as per your requirement
            color_detected = "Blue"
        elif red_pixel_count > 500:  # Adjust the threshold as per your requirement
            color_detected = "Red"
        else:
            color_detected = "Unknown"

        return color_detected

    def publish_task_status(self, color_detected):
        task_status_msg = "Unknown"

        if color_detected == "Blue":
            task_status_msg = "Iron extraction ongoing"
        elif color_detected == "Red":
            task_status_msg = "Zinc extraction ongoing"
        elif color_detected == "Unknown":
            task_status_msg = "Unknown cone color detected"

        self.task_status_pub.publish(task_status_msg)

if __name__ == '__main__':
    try:
        node = ColorDetectionNode()
        node.run()
    except rospy.ROSInterruptException:
        pass
