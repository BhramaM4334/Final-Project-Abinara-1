import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np
from glfw import get_framebuffer_size


def callback(msg):
    bridge = CvBridge()
    kernel = np.ones((5, 5), np.uint8)

    try:
        frame = bridge.imgmsg_to_cv2(msg, "bgr8")
    except CvBridgeError as e:
        rospy.logerr(f"CV Bridge error: {e}")
        return

    frame = cv2.medianBlur(frame, 15)
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # 2. Define HSV range for the bright orange/red objects
    # Adjust these thresholds based on your specific lighting and object color
    lower_color = np.array([5, 70, 70])
    upper_color = np.array([35, 255, 255])

    # Create a binary mask
    mask = cv2.inRange(hsv, lower_color, upper_color)

    # Clean up noise using morphological operations
    kernel = np.ones((5, 5), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
#    mask = cv2.dilate(mask, kernel, iterations= 3)

    contours, _ = cv2.findContours(
        mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
    )
    for cnt in contours:
        if cv2.contourArea(cnt) > 600:
            cv2.drawContours(frame, [cnt], -1, (0, 255, 0), 2)


    cv2.imshow("mask", frame)
    cv2.imshow("hsv",hsv)
    key = cv2.waitKey(1)
    if key == 27:  # ESC key
        rospy.signal_shutdown("ESC pressed.")


def main():
    #node
    rospy.init_node('image_subscriber', anonymous=True)
    rospy.Subscriber('/usb_cam/image_raw', Image, callback)
    rospy.loginfo("Image subscriber started.")

    global img_pub #img_pub becomes public for callback to use
    img_pub = rospy.Publisher('/detected_objects', Image, queue_size=10)

    #rospy
    rospy.spin()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()