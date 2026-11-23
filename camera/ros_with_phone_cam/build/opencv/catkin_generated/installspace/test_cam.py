#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2
import numpy as np


class PhoneObjectDetector:

    def __init__(self):
        # Initialize the ROS node
        rospy.init_node('phone_object_detector', anonymous=True)

        # Bridge to convert ROS image messages to OpenCV format
        self.bridge = CvBridge()

        # Subscribe to the phone's camera feed topic
        self.image_sub = rospy.Subscriber('/usb_cam/image_raw', Image, self.image_callback)

        # Publisher for processed data or object detection results
        self.object_pub = rospy.Publisher('/detected_objects', String, queue_size=10)

        rospy.loginfo("Phone Object Detector Node Initialized. Listening to /usb_cam/image_raw...")

    def image_callback(self, data):
        try:
            # Convert ROS Image message to an OpenCV BGR frame
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except Exception as e:
            rospy.logerr(f"CvBridge Error: {e}")
            return

        # --- YOUR OBJECT DETECTION / PROCESSING LOGIC HERE ---
        new_w = 1920
        new_h = 1080

     #   cv_image = cv2.resize(cv_image,(new_w,new_h))

        height, width, _ = cv_image.shape
        detected_info = f"Frame received: {width}x{height} - Status: Processing..."
        #let's try using contours

        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)

        gray = cv2.GaussianBlur(gray, (11, 11), 0)
        #        gray = cv2.medianBlur(gray, 5)

        _, thresh = cv2.threshold(gray, 0, 50, cv2.THRESH_BINARY_INV)
#        inv_cont = cv2.bitwise_not(thresh)

#inverted

        #draw contour

        contours, hierarchy = cv2.findContours(thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        mask = np.zeros(cv_image.shape, dtype=np.uint8)
        cv2.drawContours(mask, contours, -1, (255, 255, 255), thickness=-1)

        #mask is the contour
        for i in contours:
            area = cv2.contourArea(i)

            epsilon = 0.2 * cv2.arcLength(i, True)
            perimeter = cv2.arcLength(i, True)

            if area > 500:
                #create the borders
                x, y, w, h = cv2.boundingRect(i)

                #calc centers
                center_x = x + w // 2
                center_y = y + h // 2

                # adding rectangle + circle for location, and added a safety measurement (20 pixels)
                cv2.rectangle(cv_image, (x - 20, y - 20), (x + w + 20, y + h + 20), (0, 0, 255), 2)
                cv2.circle(cv_image, (center_x, center_y), 5, (255, 0, 0), -1)



        # Publish the results to your custom topic
        self.object_pub.publish(detected_info)

#        _3gray= cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)
#        disp = np.hstack(_3gray,cv_image)

        # Optional: Display the feed locally on your desktop for verification
        cv2.imshow("Phone contourd", thresh)
        cv2.imshow("Phone cam (modified)", cv_image)
        cv2.imshow("gray", gray)
        cv2.waitKey(30)


if __name__ == '__main__':
    try:
        detector = PhoneObjectDetector()
        rospy.spin()
    except rospy.ROSInterruptException:
        cv2.destroyAllWindows()