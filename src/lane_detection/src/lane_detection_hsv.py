#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
import sys
import os
from std_msgs.msg import Float32
from lane_detection.msg import Drive_command
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), 'scripts')))
from slidewindow_both_lane import SlideWindow

class LaneDetectionROS:
    def __init__(self):
        # ROS 노드 초기화
        rospy.init_node('lane_detection_ros', anonymous=True)
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber('/usb_cam/image_rect_color', Image, self.image_callback)

        self.ctrl_cmd_pub = rospy.Publisher('/motor_lane', Drive_command, queue_size=1)
        
        self.bridge = CvBridge()  # CV-Bridge 초기화
        
        self.ctrl_cmd_msg = Drive_command()  # 모터 제어 메시지 초기화

        # SlideWindow 객체 초기화
        self.slidewindow = SlideWindow()


        self.steer = 0.0  # 조향각 초기화
        self.motor = 0.0  # 모터 속도 초기화
           

        # 초기 HSV 범위 설정
        self.lower_yellow = np.array([20, 110, 40])
        self.upper_yellow = np.array([53, 240, 255])
        self.lower_white = np.array([10, 0, 214])
        self.upper_white = np.array([82, 140, 255])

        # 트랙바 윈도우 생성
        cv2.namedWindow("Trackbars")

        # 트랙바 생성 (노란색 범위)
        cv2.createTrackbar("Yellow Lower H", "Trackbars", self.lower_yellow[0], 179, self.nothing)
        cv2.createTrackbar("Yellow Lower S", "Trackbars", self.lower_yellow[1], 255, self.nothing)
        cv2.createTrackbar("Yellow Lower V", "Trackbars", self.lower_yellow[2], 255, self.nothing)
        cv2.createTrackbar("Yellow Upper H", "Trackbars", self.upper_yellow[0], 179, self.nothing)
        cv2.createTrackbar("Yellow Upper S", "Trackbars", self.upper_yellow[1], 255, self.nothing)
        cv2.createTrackbar("Yellow Upper V", "Trackbars", self.upper_yellow[2], 255, self.nothing)

        # 트랙바 생성 (흰색 범위)
        cv2.createTrackbar("White Lower H", "Trackbars", self.lower_white[0], 179, self.nothing)
        cv2.createTrackbar("White Lower S", "Trackbars", self.lower_white[1], 255, self.nothing)
        cv2.createTrackbar("White Lower V", "Trackbars", self.lower_white[2], 255, self.nothing)
        cv2.createTrackbar("White Upper H", "Trackbars", self.upper_white[0], 179, self.nothing)
        cv2.createTrackbar("White Upper S", "Trackbars", self.upper_white[1], 255, self.nothing)
        cv2.createTrackbar("White Upper V", "Trackbars", self.upper_white[2], 255, self.nothing)

        self.pub_x_location = rospy.Publisher('/lane_x_location', Float32, queue_size=1)

        self.cv_image = None
        self.rate = rospy.Rate(30)  # 10Hz 루프

    def nothing(self, x):
        pass

    def image_callback(self, msg):
        try:
            # ROS 이미지 메시지를 OpenCV 이미지로 변환
            self.cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            rospy.logerr("Error converting image: %s" % str(e))

    def run(self):
        while not rospy.is_shutdown():
            if self.cv_image is not None:
                # 비디오 프레임 크기 조정
                frame_resized = cv2.resize(self.cv_image, (640, 480))

                # 트랙바로부터 현재 HSV 범위 가져오기 (노란색)
                self.lower_yellow = np.array([
                    cv2.getTrackbarPos("Yellow Lower H", "Trackbars"),
                    cv2.getTrackbarPos("Yellow Lower S", "Trackbars"),
                    cv2.getTrackbarPos("Yellow Lower V", "Trackbars")
                ])
                self.upper_yellow = np.array([
                    cv2.getTrackbarPos("Yellow Upper H", "Trackbars"),
                    cv2.getTrackbarPos("Yellow Upper S", "Trackbars"),
                    cv2.getTrackbarPos("Yellow Upper V", "Trackbars")
                ])

                # 트랙바로부터 현재 HSV 범위 가져오기 (흰색)
                self.lower_white = np.array([
                    cv2.getTrackbarPos("White Lower H", "Trackbars"),
                    cv2.getTrackbarPos("White Lower S", "Trackbars"),
                    cv2.getTrackbarPos("White Lower V", "Trackbars")
                ])
                self.upper_white = np.array([
                    cv2.getTrackbarPos("White Upper H", "Trackbars"),
                    cv2.getTrackbarPos("White Upper S", "Trackbars"),
                    cv2.getTrackbarPos("White Upper V", "Trackbars")
                ])

                # 이미지 처리
                y, x = frame_resized.shape[0:2]

                # HSV 변환 및 마스크 생성
                img_hsv = cv2.cvtColor(frame_resized, cv2.COLOR_BGR2HSV)

                # 노란색 및 흰색 마스크 생성
                mask_yellow = cv2.inRange(img_hsv, self.lower_yellow, self.upper_yellow)
                mask_white = cv2.inRange(img_hsv, self.lower_white, self.upper_white)
                filtered_yellow = cv2.bitwise_and(frame_resized, frame_resized, mask=mask_yellow)
                filtered_white = cv2.bitwise_and(frame_resized, frame_resized, mask=mask_white)
                masks = cv2.bitwise_or(mask_yellow, mask_white)
                filtered_img = cv2.bitwise_and(frame_resized, frame_resized, mask=masks)

                # Perspective Transform
                left_margin = 250
                top_margin = 320
                src_point1 = [100, 460]      # 왼쪽 아래
                src_point2 = [left_margin+20, top_margin]
                src_point3 = [x-left_margin-20, top_margin]
                src_point4 = [x -100, 460]  

                src_points = np.float32([src_point1, src_point2, src_point3, src_point4])

                dst_point1 = [x//4, 460]    # 왼쪽 아래
                dst_point2 = [x//4, 0]      # 왼쪽 위
                dst_point3 = [x//4*3, 0]    # 오른쪽 위
                dst_point4 = [x//4*3, 460]  # 오른쪽 아래

                dst_points = np.float32([dst_point1, dst_point2, dst_point3, dst_point4])

                matrix = cv2.getPerspectiveTransform(src_points, dst_points)
                warped_img = cv2.warpPerspective(filtered_img, matrix, (640, 480))

                # 기존 HSV 방식에서 다시 살리기
                grayed_img = cv2.cvtColor(warped_img, cv2.COLOR_BGR2GRAY)

                # 이미지 이진화
                bin_img = np.zeros_like(grayed_img)
                bin_img[grayed_img > 20] = 1

                # 슬라이딩 윈도우 차선 검출
                out_img, x_location, _ = self.slidewindow.slidewindow(bin_img, False)

                self.steer = (x_location - 320)
                if self.version == 'fast':
                    self.motor = 0.7 
                else:
                    self.motor = 0.35               
                self.publishCtrlCmd(self.motor, self.steer) 
                
                # 결과 표시
                cv2.imshow('Original Image', frame_resized)
                cv2.imshow("Yellow Mask", filtered_yellow)
                cv2.imshow("White Mask", filtered_white)
                cv2.imshow("Filtered Image", filtered_img)
                cv2.imshow("Warped Image", warped_img)
                cv2.imshow("Output Image", out_img)
                print("x_location", x_location)
                # 화면 업데이트 및 이벤트 처리
                cv2.waitKey(1)  # 1ms 동안 대기

            self.rate.sleep()

    def publishCtrlCmd(self, motor_msg, servo_msg):
        self.ctrl_cmd_msg.speed = motor_msg  # 모터 속도 설정
        self.ctrl_cmd_msg.angle = servo_msg  # 조향각 설정
        self.ctrl_cmd_msg.flag = True
        self.ctrl_cmd_pub.publish(self.ctrl_cmd_msg)  # 명령 퍼블리시
        
if __name__ == "__main__":
    lane_detection_ros = LaneDetectionROS()
    lane_detection_ros.run()
