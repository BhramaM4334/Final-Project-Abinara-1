import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Point
import cv2
import numpy as np
from glfw import get_framebuffer_size

px_per_cm = 5 #1 cm = 6.5 px
trigger = False #for space bar
#y negative goes to the right

def callback(msg):
    global trigger #so that trigger becomes global
    bridge = CvBridge()
    kernel = np.ones((5, 5), np.uint8)

    try:
        frame = bridge.imgmsg_to_cv2(msg, "bgr8")
    except CvBridgeError as e:
        rospy.logerr(f"CV Bridge error: {e}")
        return

    frame = cv2.medianBlur(frame, 5)
    frame = cv2.GaussianBlur(frame, (7,7), 0)

    circled = frame
    contoured = frame
    circled = cv2.rectangle(circled, [0,240],[640,240], [0,0,255], -1)
    circled = cv2.rectangle(circled, [320, 0], [320, 480], [0, 0, 255], -1)


    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    blur = cv2.medianBlur(hsv, 5)

    #black
    bl_low = np.array([90, 20, 20])
    bl_up = np.array([130, 130, 120])

    bl_mask = cv2.inRange(hsv,bl_low,bl_up)
    bl_mask = cv2.morphologyEx(bl_mask, cv2.MORPH_OPEN, kernel)  # remove small specks
    bl_mask = cv2.morphologyEx(bl_mask, cv2.MORPH_CLOSE, kernel)

    #orange
    or_low = np.array([3, 150, 150])
    or_up = np.array([15, 255, 255])

    or_mask = cv2.inRange(hsv, or_low,or_up)
    or_mask = cv2.morphologyEx(or_mask, cv2.MORPH_OPEN, kernel)  # remove small specks
    or_mask = cv2.morphologyEx(or_mask, cv2.MORPH_CLOSE, kernel)

    or_fr = cv2.bitwise_and(frame,frame, mask=or_mask)
    #beige
    be_low = np.array([5, 25, 150])
    be_up = np.array([25, 110, 255])

    be_mask = cv2.inRange(hsv,be_low,be_up)
    be_mask = cv2.morphologyEx(be_mask, cv2.MORPH_OPEN, kernel)  # remove small specks
    be_mask = cv2.morphologyEx(be_mask, cv2.MORPH_CLOSE, kernel)


    be_fr = cv2.bitwise_and(frame, frame, mask=be_mask)

    #contours for the
    be_contours, _ = cv2.findContours(be_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    or_contours, _ = cv2.findContours(or_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    bl_contours, _ = cv2.findContours(bl_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    #    _3_contours = sorted(be_contours, key=cv2.contourArea, reverse=True)[:3]

    for cnt in or_contours:
        area = cv2.contourArea(cnt)
        if area > 250:
            x, y, w, h = cv2.boundingRect(cnt)  # x and y already at the mid of the square
            cx = x + w // 2
            cy = y + h // 2

            x_act = x - 320
            y_act = (y - 240)

            # cv2.drawContours(frame, [cnt], -1, (0, 255, 0), 2)
            cv2.rectangle(contoured, (x, y), (x + w, y + h), (0, 255, 0), 2)
            cv2.circle(contoured, (cx, cy), 4, (0, 0, 255), -1)
            cv2.putText(frame, f"#({x_act},{y_act})", (x, y - 8), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)

            if trigger:
                #send_target(x_act, y_act)
                send_target(x_act / px_per_cm * 10, y_act / px_per_cm * 10)

    for cnt in bl_contours:
        area = cv2.contourArea(cnt)
        if area > 250:
            x, y, w, h = cv2.boundingRect(cnt)  # x and y already at the mid of the square
            cx = x + w // 2
            cy = y + h // 2

            x_act = x - 320
            y_act = (y - 240)

            # cv2.drawContours(frame, [cnt], -1, (0, 255, 0), 2)
            cv2.rectangle(contoured, (x, y), (x + w, y + h), (0, 255, 0), 2)
            cv2.circle(contoured, (cx, cy), 4, (0, 0, 255), -1)
            cv2.putText(frame, f"#({x_act},{y_act})", (x, y - 8), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)

            if trigger:
                #send_target(x_act, y_act)
                send_target(x_act / px_per_cm * 10, y_act / px_per_cm * 10)


    for cnt in be_contours:
        area = cv2.contourArea(cnt)
        if area > 250:
            x, y, w, h = cv2.boundingRect(cnt) # x and y already at the mid of the square
            cx = x + w // 2
            cy = y + h // 2

            x_act = x-320
            y_act = (y-240)

            #cv2.drawContours(frame, [cnt], -1, (0, 255, 0), 2)
            cv2.rectangle(contoured, (x, y), (x + w, y + h), (0, 255, 0), 2)
            cv2.circle(contoured, (cx, cy), 4, (0, 0, 255), -1)
            cv2.putText(frame, f"#({x_act},{y_act})", (x, y - 8), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
            if trigger:
                #send_target(x_act,y_act)
                send_target(x_act / px_per_cm * 10, y_act / px_per_cm * 10)

    # Click to print HSV at that pixel
    def on_click(event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN:
            print("HSV:", hsv[y, x])

    #show

    cv2.imshow("img", contoured)
    cv2.setMouseCallback("img", on_click)


    cv2.imshow("hsv", be_fr)
    cv2.imshow("circle",circled)

    key = cv2.waitKey(1)
    trigger = False  # reset: the frame just processed has used its chance
    if key == 32:  # SPACE
        trigger = True  # the next frame will send its detections
        rospy.loginfo("Sent XYZ!")
    if key == 27:  # ESC
        rospy.signal_shutdown("ESC pressed.")

def main():
    #node
    rospy.init_node('image_viewer')
    rospy.Subscriber('/usb_cam/image_raw', Image, callback) #subscribe from the image cam of usb_cam
    rospy.loginfo("Image subscriber started.")

    #publisher for the x and y coordinates
    global xy_pub
    xy_pub = rospy.Publisher("/target_xy", Point, queue_size=10)


    global img_pub #img_pub becomes public for callback to use
    img_pub = rospy.Publisher('/detected_objects', Image, queue_size=10)

    #rospy
    rospy.spin()
    cv2.destroyAllWindows()

def send_target(x, y):          # x, y in mm
    msg = Point()
    msg.x = x
    msg.y = y
    msg.z = 30.0                 # ignored by the C++ node (it uses Z_FIXED)
    rospy.loginfo(f"sent x = {x}, y = {y}")
    xy_pub.publish(msg)

if __name__ == '__main__':
    main()