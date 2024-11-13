#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
import sys
import os
from std_msgs.msg import Float32, Float64, String
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
        self.kd = kd  # 미분 이득 설정
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
        self.image_sub = rospy.Subscriber('/usb_cam/image_rect_color', Image, self.image_callback)
        rospy.Subscriber("/heading", Float64, self.headingCB)
        rospy.Subscriber("/raw_obstacles", Obstacles, self.obstacleCB)
        rospy.Subscriber('/lane_topic', String, self.lane_topic_callback)

        self.ctrl_cmd_pub = rospy.Publisher('/motor_lane', Drive_command, queue_size=1)
        
        
        self.ctrl_cmd_msg = Drive_command()  # 모터 제어 메시지 초기화

        # SlideWindow 객체 초기화
        self.slidewindow = SlideWindow()
        
        self.version = rospy.get_param('~version', 'safe')

        # ---------------------------튜닝 해야하는 값 ------------------------------#
        self.steer = 0.0  # 조향각 초기화
        self.motor = 0.5  # 모터 속도 초기화
        self.k_p = 0.5

        if self.version == 'fast':
            self.pid = PID(0.78, 0.0005, 0.405) 
        else:
            self.pid = PID(0.7, 0.0008, 0.15)
        # -----------------------------------------------------------------------#
        self.real_heading = None
        self.gt_heading = None
        self.local_heading = None
        self.obstacles = []
        self.gt_heading_list = []
        self.lane_state = None
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
                yellow_pixels = cv2.countNonZero(mask_yellow)

                if yellow_pixels < 500:  # 노란색 픽셀 수 50은 환경에 따라 조정 가능
                    # 노란색 차선이 없을 때 수행할 로직
                    print("Yellow lane not detected!")
                    if self.gt_heading is None:
                        # 터널 들어 갈 때 값이 아닌, 실험적으로 값 찾아서 하드코딩 하는게 더 좋을거같음(imu값 잘변함 heading을 하드코딩은 위험)
                        # 초기 heading n개의 평균을 내서 안정적인 초기 heading 채택 방법도 괜찮을듯
                        # 라이다로 충돌 방지도 추가 우측, 좌측 근접한 roi에 터널 벽이 잡힌다면, 이와 멀어지는 방향으로 gt_heading 수정
                        if self.gt_heading is None:
                            if len(self.gt_heading_list) < 10:
                                self.gt_heading_list.append(self.real_heading)
                            else:
                                self.gt_heading = np.mean(self.gt_heading_list)

                    else:
                        if(len(self.obstacles) > 0):
                            for obstacle in self.obstacles:
                                # 좌측 터널 벽 감지 (x 좌표가 left_threshold_x 이상이고 장애물의 거리가 임계값 이하일 때)
                                if - 0.3 < obstacle.x < 0.0 and 0.0 < obstacle.y < 0.3: # 왼쪽 벽 근접
                                    self.gt_heading -= 0.05  # 우측으로 살짝 이동하도록 헤딩 조정

                                # 우측 터널 벽 감지 (x 좌표가 right_threshold_x 이하이고 장애물의 거리가 임계값 이하일 때)
                                elif - 0.3 < obstacle.x < 0.0 and -0.3 < obstacle.y < 0.0: # 오른쪽 벽 근접
                                    self.gt_heading += 0.05  # 좌측으로 살짝 이동하도록 헤딩 조정

                        if self.local_heading is not None and self.gt_heading is not None: 
                            self.steer = self.k_p * (self.local_heading - self.gt_heading)
                            self.publishCtrlCmd(self.motor, self.steer)

                else:
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

                    self.steer = round(self.pid.pid_control(x_location - 320))  # PID 제어를 통한 각도 계산

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

    def obstacleCB(self, msg):
        self.obstacles = []
        for circle in msg.circles:
            x = circle.center.x
            y = circle.center.y
            distance = (x**2 + y**2) ** 0.5  # 유클리드 거리 계산
            obstacle = Obstacle(x, y, distance)
            self.obstacles.append(obstacle)
        
        self.obstacles.sort(key=lambda obs: obs.distance)

        # if len(self.obstacles) > 0:
        #     self.closest_obstacle = self.obstacles[0]
        # else:
        #     self.closest_obstacle = Obstacle()

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

    def lane_topic_callback(self, msg):
        # /lane_topic의 메시지를 받아서 현재 lane_state 업데이트
        if msg.data == "LEFT_LANE":
            self.lane_state = "LEFT_LANE"
        elif msg.data == "RIGHT_LANE":
            self.lane_state = "RIGHT_LANE"
        rospy.loginfo(f"Current lane state: {self.lane_state}")

    def publishCtrlCmd(self, motor_msg, servo_msg):
        self.ctrl_cmd_msg.speed = motor_msg  # 모터 속도 설정
        self.ctrl_cmd_msg.angle = servo_msg  # 조향각 설정
        self.ctrl_cmd_msg.flag = True
        self.ctrl_cmd_pub.publish(self.ctrl_cmd_msg)  # 명령 퍼블리시
        
if __name__ == "__main__":
    lane_detection_ros = LaneDetectionROS()
    lane_detection_ros.run()
