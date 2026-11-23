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


    cv2.imshow("Camera Feed", frame)
    key = cv2.waitKey(30)
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