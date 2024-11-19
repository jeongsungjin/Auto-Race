#!/usr/bin/env python3

from __future__ import print_function
import cv2
from cv_bridge import CvBridge, CvBridgeError
import rospy
from sensor_msgs.msg import Image
import numpy as np
from utils import *

from std_msgs.msg import Bool, Int32

class ParkingSingDetection:
    def __init__(self):
        rospy.init_node('parking_sign', anonymous=True)

        try:
            self.bridge = CvBridge()
            rospy.Subscriber("/tunnel_done", Bool, self.tunnel_done_callback) # 콜백 만들어서 주석 풀어야함!

            rospy.Subscriber("/usb_cam/image_raw", Image, self.cameraCB)
            self.is_blue_pub = rospy.Publisher("/is_blue", Bool, queue_size=1)
            self.blue_pixel_pub = rospy.Publisher("/blue_pixels", Int32, queue_size=1)

            self.tunnel_done_flag = False
            self.cv_image = None

            self.is_blue_msg = Bool()

            # 트랙바 윈도우 생성
            cv2.namedWindow('BLUE Trackbars')
            cv2.createTrackbar('H_min_blue', 'BLUE Trackbars', 15, 179, self.nothing)
            cv2.createTrackbar('H_max_blue', 'BLUE Trackbars', 150, 179, self.nothing)
            cv2.createTrackbar('S_min_blue', 'BLUE Trackbars', 170, 255, self.nothing)
            cv2.createTrackbar('S_max_blue', 'BLUE Trackbars', 255, 255, self.nothing)
            cv2.createTrackbar('V_min_blue', 'BLUE Trackbars', 200, 255, self.nothing)
            cv2.createTrackbar('V_max_blue', 'BLUE Trackbars', 255, 255, self.nothing)
            
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
    
    def publish_blue_pixel(self, blue_pixel):
        self.blue_pixel_pub.publish(blue_pixel)

    def detect_blue(self, image):
        frame_resized = cv2.resize(self.cv_image, (640, 480))

        self.lower_blue = np.array([
            cv2.getTrackbarPos("H_min_blue", "BLUE Trackbars"),
            cv2.getTrackbarPos("S_min_blue", "BLUE Trackbars"),
            cv2.getTrackbarPos("V_min_blue", "BLUE Trackbars")
        ])
        
        self.upper_blue = np.array([
            cv2.getTrackbarPos("H_max_blue", "BLUE Trackbars"),
            cv2.getTrackbarPos("S_max_blue", "BLUE Trackbars"),
            cv2.getTrackbarPos("V_max_blue", "BLUE Trackbars")
        ])

        img_hsv = cv2.cvtColor(frame_resized, cv2.COLOR_BGR2HSV)

        # 노란색 및 흰색 마스크 생성
        mask_blue = cv2.inRange(img_hsv, self.lower_blue, self.upper_blue)

        # 윤곽선 찾기
        blue_pixel_counts = np.count_nonzero(mask_blue)
        cv2.imshow("blue mask", mask_blue)
        if blue_pixel_counts > 1000 :
            self.is_blue_msg.data = True
        else:
            self.is_blue_msg.data = False
        
        self.is_blue_pub.publish(self.is_blue_msg)
        self.publish_blue_pixel(blue_pixel_counts)

if __name__ == '__main__':
    try:
        parking_sign_detection_node = ParkingSingDetection()
    except rospy.ROSInterruptException:
        pass
