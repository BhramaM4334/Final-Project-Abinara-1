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

    #do the thing
    #orange, beige, and green

    hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    blurred = cv2.medianBlur(frame, 11)
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    #ORANGE
    o_low = np.array([5, 100, 100])
    o_up = np.array([35, 255, 255])

    # 3. Generate the binary mask
    o_mask = cv2.inRange(blurred, o_low, o_up)

    o_frame = cv2.bitwise_and(blurred, blurred, mask=o_mask)
    o_rgb_frame = cv2.bitwise_and(frame, frame, mask = o_mask)

 #   img_pub.publish(frame)

    #BEIGE

    b_low = np.array([15, 10, 170])
    b_up = np.array([35, 80, 255])

    # masking
    b_mask = cv2.inRange(blurred, b_low,b_up)

    b_frame = cv2.bitwise_and(blurred,blurred, mask=b_mask)
    b_bgr = cv2.cvtColor(b_frame, cv2.COLOR_HSV2BGR)

#testing for transformations
#    eroded = cv2.erode(b_bgr_frame, kernel, iterations=1)
#    dilated = cv2.dilate(b_bgr_frame, kernel, iterations=1)
#    opening = cv2.morphologyEx(b_bgr_frame, cv2.MORPH_OPEN, kernel)
#    closing = cv2.morphologyEx(b_bgr_frame, cv2.MORPH_CLOSE, kernel)
#    group = np.hstack([eroded,dilated,opening,closing])


    #GREEM
    g_low = np.array([50, 100, 100])
    g_up = np.array([70, 255, 255])

    g_mask = cv2.inRange(blurred, g_low, g_up)

    g_frame = cv2.bitwise_and(blurred,blurred, mask= g_mask)
    g_bgr_frame = cv2.bitwise_and(frame, frame, mask = g_mask)


    #ALL
    all_frame = cv2.bitwise_or(g_bgr_frame, o_rgb_frame, b_bgr)
 #   beige_test = np.hstack([blurred, bu_bgr_frame, b_frame])


    cv2.imshow("Camera Feed", all_frame)
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