#!/usr/bin/env python3

from __future__ import print_function
import cv2
from cv_bridge import CvBridge, CvBridgeError
import rospy
from sensor_msgs.msg import Image
import numpy as np
from utils import *

from std_msgs.msg import Bool

class ParkingSingDetection:
    def __init__(self):
        rospy.init_node('parking_sign', anonymous=True)

        try:
            self.bridge = CvBridge()
            rospy.Subscriber("/tunnel_done", Bool, self.tunnel_done_callback) # 콜백 만들어서 주석 풀어야함!

            rospy.Subscriber("/usb_cam/image_raw", Image, self.cameraCB)
            self.is_blue_pub = rospy.Publisher("/is_blue", Bool, queue_size=1)

            self.tunnel_done_flag = False
            self.cv_image = None

            self.is_blue_msg = Bool()

            # 트랙바 윈도우 생성
            cv2.namedWindow('BLUE Trackbars')
            cv2.createTrackbar('H_min_blue', 'BLUE Trackbars', 0, 179, self.nothing)
            cv2.createTrackbar('H_max_blue', 'BLUE Trackbars', 17, 179, self.nothing)
            cv2.createTrackbar('S_min_blue', 'BLUE Trackbars', 133, 255, self.nothing)
            cv2.createTrackbar('S_max_blue', 'BLUE Trackbars', 205, 255, self.nothing)
            cv2.createTrackbar('V_min_blue', 'BLUE Trackbars', 113, 255, self.nothing)
            cv2.createTrackbar('V_max_blue', 'BLUE Trackbars', 161, 255, self.nothing)
            
            rate = rospy.Rate(30)
            while not rospy.is_shutdown():
                if self.cv_image is not None:
                    self.detect_blue(self.cv_image)
                    cv2.waitKey(1)
                rate.sleep()

        finally:
            cv2.destroyAllWindows()

    def nothing(self, x):
        pass

    def cameraCB(self, msg):
        try:
            self.cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as e:
            rospy.logwarn(e)

    def tunnel_done_callback(self, msg):
        self.tunnel_done_flag = msg.data

    def detect_blue(self, image):
        hsv_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        h_min_blue = 100
        h_max_blue = 130
        s_min_blue = 100
        s_max_blue = 255
        v_min_blue = 50
        v_max_blue = 255

        lower_blue = np.array([h_min_blue, s_min_blue, v_min_blue])
        upper_blue = np.array([h_max_blue, s_max_blue, v_max_blue])

        blue_mask = cv2.inRange(hsv_image, lower_blue, upper_blue)

        # 윤곽선 찾기
        blue_pixel_counts = np.count_nonzero(blue_mask)
        cv2.imshow("blue mask", blue_mask)
        if blue_pixel_counts > 1000 and self.tunnel_done_flag == True:
            self.is_blue_msg.data = True
        else:
            self.is_blue_msg.data = False
        
        self.is_blue_pub.publish(self.is_blue_msg)


if __name__ == '__main__':
    try:
        parking_sign_detection_node = ParkingSingDetection()
    except rospy.ROSInterruptException:
        pass
