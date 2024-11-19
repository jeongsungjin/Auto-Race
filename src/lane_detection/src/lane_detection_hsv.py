#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
import sys
import os
from std_msgs.msg import Float32, Float64, String, Int32
from lane_detection.msg import Drive_command
from obstacle_detector.msg import Obstacles

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), 'scripts')))
from slidewindow_both_lane import SlideWindow

class Obstacle:
    def __init__(self, x=None, y=None, distance=None):
        self.x = x
        self.y = y
        self.distance = distance

# PID 클래스 정의
class PID():
    def __init__(self, kp, ki, kd):
        self.kp = kp  # 비례 이득 설정
        self.ki = ki  # 적분 이득 설정
        self.kd = kd  # 미분 이s득 설정
        self.p_error = 0.0  # 이전 비례 오차 초기화
        self.i_error = 0.0  # 적분 오차 초기화
        self.d_error = 0.0  # 미분 오차 초기화

    def pid_control(self, cte):
        self.d_error = cte - self.p_error  # 미분 오차 계산
        self.p_error = cte  # 비례 오차 갱신
        self.i_error += cte  # 적분 오차 갱신

        # PID 제어 계산
        return self.kp * self.p_error + self.ki * self.i_error + self.kd * self.d_error


class LaneDetectionROS:
    def __init__(self):
        # ROS 노드 초기화
        rospy.init_node('lane_detection_ros', anonymous=True)
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber('/usb_cam/image_raw', Image, self.image_callback)
        rospy.Subscriber("/heading", Float64, self.headingCB)
        rospy.Subscriber("/raw_obstacles", Obstacles, self.obstacleCB)
        rospy.Subscriber('/lane_topic', String, self.lane_topic_callback)

        self.ctrl_cmd_pub = rospy.Publisher('/motor_lane', Drive_command, queue_size=1)
        self.white_cnt = rospy.Publisher('/white_cnt', Int32, queue_size=1)
        self.yellow_pixel = rospy.Publisher('/yellow_pixel', Int32, queue_size=1)

        self.white_count = 0
        self.ctrl_cmd_msg = Drive_command()  # 모터 제어 메시지 초기화

        self.after_white = False
        self.stop_count = 0
        self.closest_obstacle = None
        # SlideWindow 객체 초기화
        self.slidewindow = SlideWindow()
        
        self.version = rospy.get_param('~version', 'safe')

        # ---------------------------튜닝 해야하는 값 ------------------------------#
        self.steer = 0.0  # 조향각 초기화
        self.motor = 0.5  # 모터 속도 초기화
        self.k_p = 10

        if self.version == 'fast':
            self.pid = PID(0.78, 0.0005, 0.405) 
        else:
            self.pid = PID(0.7, 0.0008, 0.15)
        #-----------------------------------------------------------------------#
        self.real_heading = None
        self.gt_heading = None
        self.local_heading = None
        self.obstacles = []
        self.gt_heading_list = []
        self.lane_state = None
        #초기 HSV 범위 설정
        self.lower_yellow = np.array([29, 66, 105])
        self.upper_yellow = np.array([68, 192, 255])
        self.lower_white = np.array([0, 0, 235])
        self.upper_white = np.array([0, 255, 255])
        self.lower_red = np.array([0, 25, 150])
        self.upper_red = np.array([5, 255, 255])
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

        cv2.createTrackbar("Red Lower H", "Trackbars", self.lower_red[0], 179, self.nothing)
        cv2.createTrackbar("Red Lower S", "Trackbars", self.lower_red[1], 255, self.nothing)
        cv2.createTrackbar("Red Lower V", "Trackbars", self.lower_red[2], 255, self.nothing)
        cv2.createTrackbar("Red Upper H", "Trackbars", self.upper_red[0], 179, self.nothing)
        cv2.createTrackbar("Red Upper S", "Trackbars", self.upper_red[1], 255, self.nothing)
        cv2.createTrackbar("Red Upper V", "Trackbars", self.upper_red[2], 255, self.nothing)

        self.pub_x_location = rospy.Publisher('/lane_x_location', Float32, queue_size=1)

        self.cv_image = None
        self.rate = rospy.Rate(30)  # 30Hz 루프

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
                # 트랙바 빨강
                self.lower_red = np.array([
                    cv2.getTrackbarPos("Red Lower H", "Trackbars"),
                    cv2.getTrackbarPos("Red Lower S", "Trackbars"),
                    cv2.getTrackbarPos("Red Lower V", "Trackbars")
                ])
                self.upper_red = np.array([
                    cv2.getTrackbarPos("Red Upper H", "Trackbars"),
                    cv2.getTrackbarPos("Red Upper S", "Trackbars"),
                    cv2.getTrackbarPos("Red Upper V", "Trackbars")
                ])

                # 이미지 처리
                y, x = frame_resized.shape[0:2]


                # HSV 변환 및 마스크 생성
                img_hsv = cv2.cvtColor(frame_resized, cv2.COLOR_BGR2HSV)

                # 노란색 및 흰색 마스크 생성
                mask_yellow = cv2.inRange(img_hsv, self.lower_yellow, self.upper_yellow)
                mask_white = cv2.inRange(img_hsv, self.lower_white, self.upper_white)

                # 빨간색 감속
                mask_red = cv2.inRange(img_hsv, self.lower_red, self.upper_red) 

                filtered_yellow = cv2.bitwise_and(frame_resized, frame_resized, mask=mask_yellow)
                filtered_white = cv2.bitwise_and(frame_resized, frame_resized, mask=mask_white)
                filtered_red = cv2.bitwise_and(frame_resized, frame_resized, mask=mask_red)

                filtered_img = cv2.bitwise_and(frame_resized, frame_resized, mask=mask_yellow)
                yellow_pixels = cv2.countNonZero(mask_yellow)                       
                self.publish_yellow_pixel(yellow_pixels)

                # Perspective Transform
                left_margin = 200
                top_margin = 340
                # src_point1 = [100, 460]      # 왼쪽 아래
                # src_point2 = [left_margin+20, top_margin]
                # src_point3 = [x-left_margin-20, top_margin]
                # src_point4 = [x -100, 460]  

                src_point1 = [128, 400]      # 왼쪽 아래
                src_point2 = [left_margin, top_margin]
                src_point3 = [x-left_margin, top_margin]
                src_point4 = [520, 400] 

                src_points = np.float32([src_point1, src_point2, src_point3, src_point4])

                dst_point1 = [x//4, 460]    # 왼쪽 아래
                dst_point2 = [x//4, 0]      # 왼쪽 위
                dst_point3 = [x//4*3, 0]    # 오른쪽 위
                dst_point4 = [x//4*3, 460]  # 오른쪽 아래

                dst_points = np.float32([dst_point1, dst_point2, dst_point3, dst_point4])
                

                matrix = cv2.getPerspectiveTransform(src_points, dst_points)
                

                warped_img = cv2.warpPerspective(filtered_img, matrix, (640, 480))
                warped_img_white = cv2.warpPerspective(mask_white, matrix, (640, 480))

                # 기존 HSV 방식에서 다시 살리기
                grayed_img = cv2.cvtColor(warped_img , cv2.COLOR_BGR2GRAY)

                # 이미지 이진화
                bin_img = np.zeros_like(grayed_img)
                bin_img[grayed_img > 20] = 1 

                # 슬라이딩 윈도우 차선 검출
                out_img, x_location, _ = self.slidewindow.slidewindow(bin_img)

                # self.steer = (self.pid.pid_control(x_location - 320))  # PID 제어를 통한 각도 계산
                self.steer = (x_location - 320)
                if self.version == 'fast':
                    self.motor = 0.5 
                else:
                    self.motor = 0.4      

                
                # 미션 2: 빨간색 차로 구간에서 감속
                if np.count_nonzero(mask_red) > 5000:  # 빨간색 픽셀 개수 기준 감속 여부 판단
                    self.motor = 0.21  # 감속
                    print("빨강빨강~")

                # 미션 3: 흰색 횡단보도 구간에서 정지
                elif self.stop_count < 220 and np.count_nonzero(warped_img_white) > 60000: # 흰색 픽셀 개수 기준 정지 여부 판단
                    print("흰색 만나서 정지한 횟수", self.stop_count)
                    self.motor = 0.0
                    self.stop_count += 1
                    # rospy.sleep(8)  # 8초 동안 정지
                    # self.motor = 0.5
                    self.after_white = True
                
                else:
                    self.publishCtrlCmd(self.motor, self.steer) 
                
                if self.after_white == True:
                    self.white_count += 1
                    self.publish_white_cnt(self.white_count)
                
                print(np.count_nonzero(warped_img_white))

                # 결과 표시
                # # cv2.imshow('Original Image', frame_resized)
                # cv2.imshow("Yellow Mask", filtered_yellow)
                # cv2.imshow("White Mask", filtered_white)
                # # cv2.imshow("Red Mask", filtered_red)
                # # cv2.imshow("Filtered Image", filtered_img)
                # # cv2.imshow("Warped Image", warped_img)
                # cv2.imshow("Output Image", out_img)
                # # cv2.imshow("Warped White Stop Line", warped_img_white)

                # print("x_location", x_location)
                # 화면 업데이트 및 이벤트 처리
                cv2.waitKey(1)  # 1ms 동안 대기
                self.publishCtrlCmd(self.motor, self.steer) 

            self.rate.sleep()

    def obstacleCB(self, msg):
        self.obstacles = []
        for circle in msg.circles:
            x = circle.center.x
            y = circle.center.y
            distance = (x**2 + y**2) ** 0.5  # 유클리드 거리 계산
            obstacle = Obstacle(x, y, distance)
            self.obstacles.append(obstacle)
        
        self.obstacles.sort(key=lambda obs: obs.distance)

        if len(self.obstacles) > 0:
            self.closest_obstacle = self.obstacles[0]
        else:
            self.closest_obstacle = Obstacle()

    def headingCB(self, msg):
        self.real_heading = msg.data
        if self.gt_heading != None:
            self.local_heading = self.real_heading - self.gt_heading
            if self.local_heading > 180:
                self.local_heading -= 360
            elif self.local_heading < -180:
                self.local_heading += 360
        else:
            self.local_heading = None

    def publish_white_cnt(self, white_count):
        self.white_cnt.publish(white_count)

    def publish_yellow_pixel(self, yellow_pixel):
        self.yellow_pixel.publish(yellow_pixel)


    def lane_topic_callback(self, msg):
        # /lane_topic의 메시지를 받아서 현재 lane_state 업데이트
        if msg.data == "LEFT":
            self.lane_state = "LEFT"
        elif msg.data == "RIGHT":
            self.lane_state = "RIGHT"
        rospy.loginfo(f"Current lane state: {self.lane_state}")

    def publishCtrlCmd(self, motor_msg, servo_msg):
        self.ctrl_cmd_msg.speed = motor_msg  # 모터 속도 설정
        self.ctrl_cmd_msg.angle = servo_msg  # 조향각 설정
        self.ctrl_cmd_msg.flag = True
        self.ctrl_cmd_pub.publish(self.ctrl_cmd_msg)  # 명령 퍼블리시
     
if __name__ == "__main__":
    lane_detection_ros = LaneDetectionROS()
    lane_detection_ros.run()
