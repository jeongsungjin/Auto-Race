#!/usr/bin/env python3

from __future__ import print_function
import cv2
from cv_bridge import CvBridge, CvBridgeError
import rospy
from sensor_msgs.msg import Image
import numpy as np
from std_msgs.msg import Bool, Int32

class RubberconeOrangeDetection:
    def __init__(self):
        rospy.init_node('rubbercone_orange_detection', anonymous=True)

        try:
            self.bridge = CvBridge()

            # ROS 구독 및 퍼블리셔 설정
            rospy.Subscriber("/usb_cam/image_raw", Image, self.cameraCB)
            self.is_orange_pub = rospy.Publisher("/is_orange", Bool, queue_size=1)
            self.orange_pixels_pub = rospy.Publisher("/orange_pixels", Int32, queue_size=1)

            self.cv_image = None
            self.is_orange_msg = Bool()

            # 트랙바 윈도우 생성
            cv2.namedWindow('ORANGE Trackbars')
            cv2.createTrackbar('H_min_orange', 'ORANGE Trackbars', 0, 179, self.nothing)
            cv2.createTrackbar('H_max_orange', 'ORANGE Trackbars', 66, 179, self.nothing)
            cv2.createTrackbar('S_min_orange', 'ORANGE Trackbars', 41, 255, self.nothing)
            cv2.createTrackbar('S_max_orange', 'ORANGE Trackbars', 255, 255, self.nothing)
            cv2.createTrackbar('V_min_orange', 'ORANGE Trackbars', 223, 255, self.nothing)
            cv2.createTrackbar('V_max_orange', 'ORANGE Trackbars', 255, 255, self.nothing)
            
            rate = rospy.Rate(30)
            while not rospy.is_shutdown():
                if self.cv_image is not None:
                    self.detect_orange(self.cv_image)
                
                # OpenCV 트랙바 업데이트를 위한 waitKey 호출
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

    def publish_orange_pixel(self, orange_pixel):
        self.orange_pixels_pub.publish(orange_pixel)
    
    def detect_orange(self, image):
        # BGR 이미지를 HSV로 변환
        hsv_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        # 트랙바에서 HSV 값 읽기
        h_min_orange = cv2.getTrackbarPos('H_min_orange', 'ORANGE Trackbars')
        h_max_orange = cv2.getTrackbarPos('H_max_orange', 'ORANGE Trackbars')
        s_min_orange = cv2.getTrackbarPos('S_min_orange', 'ORANGE Trackbars')
        s_max_orange = cv2.getTrackbarPos('S_max_orange', 'ORANGE Trackbars')
        v_min_orange = cv2.getTrackbarPos('V_min_orange', 'ORANGE Trackbars')
        v_max_orange = cv2.getTrackbarPos('V_max_orange', 'ORANGE Trackbars')

        # HSV 값 범위 설정
        lower_orange = np.array([h_min_orange, s_min_orange, v_min_orange])
        upper_orange = np.array([h_max_orange, s_max_orange, v_max_orange])

        # 마스크 생성
        orange_mask = cv2.inRange(hsv_image, lower_orange, upper_orange)

        # 마스크에서 오렌지색 픽셀 개수 계산
        orange_pixel_counts = np.count_nonzero(orange_mask)
        # cv2.imshow("Orange Mask", orange_mask)

        # Boolean 메시지 생성
        if orange_pixel_counts > 10000:
            self.is_orange_msg.data = True
        else:
            self.is_orange_msg.data = False

        # 메시지 퍼블리시
        self.is_orange_pub.publish(self.is_orange_msg)


if __name__ == '__main__':
    try:
        rubbercone_orange_detection_node = RubberconeOrangeDetection()
    except rospy.ROSInterruptException:
        pass
