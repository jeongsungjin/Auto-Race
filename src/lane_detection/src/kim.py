#!/usr/bin/env python3

# 필요한 라이브러리 및 모듈 임포트
import rospy
from sensor_msgs.msg import Image  # 카메라 이미지 메시지를 수신하기 위한 ROS 메시지 타입
from cv_bridge import CvBridge  # ROS 이미지 메시지와 OpenCV 이미지 간 변환을 위한 브리지
import cv2
import numpy as np  # 배열 및 이미지 처리를 위한 NumPy 라이브러리
import os
import sys
from lane_detection.msg import Drive_command  # 차량 제어 명령 메시지 타입
from slidewindow_both_lane import SlideWindow  # 슬라이딩 윈도우 방식의 차선 탐지를 위한 모듈

# PID 제어기를 정의하는 클래스
class PID:
    def __init__(self, kp, ki, kd):
        # PID 제어를 위한 계수 초기화 (비례, 적분, 미분)
        self.kp = kp
        self.ki = ki
        self.kd = kd
        # 이전 오차, 누적 오차 및 변화율 오차 초기화
        self.p_error = 0.0
        self.i_error = 0.0
        self.d_error = 0.0

    def pid_control(self, cte):
        # PID 제어를 통해 조향 각도를 계산
        self.d_error = cte - self.p_error  # 오차 변화량
        self.p_error = cte  # 현재 오차 갱신
        self.i_error += cte  # 누적 오차 갱신
        # PID 계수를 통해 제어 값 반환
        return self.kp * self.p_error + self.ki * self.i_error + self.kd * self.d_error

# 메인 자율 주행 클래스
class LaneDetectionROS:
    def __init__(self):
        rospy.init_node('lane_detection_ros', anonymous=True)  # ROS 노드 초기화
        self.bridge = CvBridge()  # CvBridge 초기화
        # 카메라 이미지를 구독하고 콜백 함수 지정
        self.image_sub = rospy.Subscriber('/usb_cam/image_rect_color', Image, self.image_callback)
        # 차량의 속도 및 조향 명령을 퍼블리시할 주제와 메시지 타입 설정
        self.ctrl_cmd_pub = rospy.Publisher('/motor_lane', Drive_command, queue_size=1)
        self.ctrl_cmd_msg = Drive_command()  # 제어 명령 메시지 초기화

        # 슬라이딩 윈도우 및 PID 제어 인스턴스 초기화
        self.slidewindow = SlideWindow()
        self.pid = PID(0.7, 0.0008, 0.15)  # PID 제어기를 통해 조향각 계산

        # HSV 범위 초기값 설정 (노란색, 흰색, 빨간색 차선)
        self.lower_yellow = np.array([20, 110, 40])
        self.upper_yellow = np.array([53, 240, 255])
        self.lower_white = np.array([10, 0, 214])
        self.upper_white = np.array([82, 140, 255])
        self.lower_red = np.array([0, 120, 70])
        self.upper_red = np.array([10, 255, 255])

        # 기본 주행 속도 및 횡단보도 감지 상태 변수 설정
        self.motor_speed = 0.5
        self.crosswalk_detected = False  # 횡단보도 감지 여부 플래그
        self.stop_time = None  # 정지 시간 기록 변수
        self.stop_duration = 3  # 횡단보도 구간 정지 지속 시간 (초)

        # 트랙바 생성 (HSV 값 실시간 조정을 위한 UI)
        cv2.namedWindow("Trackbars")
        # 노란색 HSV 범위 트랙바 생성
        cv2.createTrackbar("Yellow Lower H", "Trackbars", self.lower_yellow[0], 179, self.nothing)
        cv2.createTrackbar("Yellow Lower S", "Trackbars", self.lower_yellow[1], 255, self.nothing)
        cv2.createTrackbar("Yellow Lower V", "Trackbars", self.lower_yellow[2], 255, self.nothing)
        cv2.createTrackbar("Yellow Upper H", "Trackbars", self.upper_yellow[0], 179, self.nothing)
        cv2.createTrackbar("Yellow Upper S", "Trackbars", self.upper_yellow[1], 255, self.nothing)
        cv2.createTrackbar("Yellow Upper V", "Trackbars", self.upper_yellow[2], 255, self.nothing)
        
        # 흰색 HSV 범위 트랙바 생성
        cv2.createTrackbar("White Lower H", "Trackbars", self.lower_white[0], 179, self.nothing)
        cv2.createTrackbar("White Lower S", "Trackbars", self.lower_white[1], 255, self.nothing)
        cv2.createTrackbar("White Lower V", "Trackbars", self.lower_white[2], 255, self.nothing)
        cv2.createTrackbar("White Upper H", "Trackbars", self.upper_white[0], 179, self.nothing)
        cv2.createTrackbar("White Upper S", "Trackbars", self.upper_white[1], 255, self.nothing)
        cv2.createTrackbar("White Upper V", "Trackbars", self.upper_white[2], 255, self.nothing)

    # 트랙바에서 사용하는 빈 함수
    def nothing(self, x):
        pass

    # 이미지 콜백 함수, 카메라 이미지 수신 시 실행
    def image_callback(self, msg):
        try:
            # ROS 이미지를 OpenCV 이미지로 변환
            self.cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            self.process_image()  # 이미지 처리 함수 호출
        except Exception as e:
            rospy.logerr(f"Error converting image: {e}")

    # 메인 이미지 처리 함수
    def process_image(self):
        # 이미지 리사이즈 및 HSV 범위 업데이트
        frame_resized = cv2.resize(self.cv_image, (640, 480))
        self.update_hsv_ranges()  # 트랙바 값을 통해 HSV 범위 업데이트

        # HSV 변환 및 색상별 마스크 생성
        hsv_img = cv2.cvtColor(frame_resized, cv2.COLOR_BGR2HSV)
        yellow_mask = cv2.inRange(hsv_img, self.lower_yellow, self.upper_yellow)  # 노란색 차선 마스크
        white_mask = cv2.inRange(hsv_img, self.lower_white, self.upper_white)  # 흰색 차선 마스크
        red_mask = cv2.inRange(hsv_img, self.lower_red, self.upper_red)  # 빨간색 차로 마스크

        # 노란색과 흰색 마스크 결합
        combined_mask = cv2.bitwise_or(yellow_mask, white_mask)
        filtered_img = cv2.bitwise_and(frame_resized, frame_resized, mask=combined_mask)

        # 미션 2: 빨간색 차로 구간에서 감속
        if np.count_nonzero(red_mask) > 5000:  # 빨간색 픽셀 개수 기준 감속 여부 판단
            self.motor_speed = 0.2  # 감속
        else:
            self.motor_speed = 0.5  # 정상 속도

        # 미션 3: 흰색 횡단보도 구간에서 정지
        if np.count_nonzero(white_mask) > 5000:  # 흰색 픽셀 개수 기준 정지 여부 판단
            if not self.crosswalk_detected:
                self.crosswalk_detected = True  # 횡단보도 감지 상태 설정
                self.stop_time = rospy.get_time()  # 정지 시간 기록
            speed = 0  # 정지 속도
        elif self.crosswalk_detected and rospy.get_time() - self.stop_time > self.stop_duration:
            self.crosswalk_detected = False  # 횡단보도 상태 해제
            speed = self.motor_speed  # 정지 후 서서히 출발

        # 퍼스펙티브 변환을 통한 차선 인식 강화
        src_points = np.float32([[100, 460], [250, 320], [390, 320], [540, 460]])
        dst_points = np.float32([[160, 460], [160, 0], [480, 0], [480, 460]])
        matrix = cv2.getPerspectiveTransform(src_points, dst_points)
        warped_img = cv2.warpPerspective(filtered_img, matrix, (640, 480))

        # 변환된 이미지를 그레이스케일로 변환 후 이진화
        gray_img = cv2.cvtColor(warped_img, cv2.COLOR_BGR2GRAY)
        _, binary_img = cv2.threshold(gray_img, 20, 255, cv2.THRESH_BINARY)

        # 슬라이딩 윈도우로 차선 중심 탐지
        out_img, x_location, _ = self.slidewindow.slidewindow(binary_img, False)
        steering = round(self.pid.pid_control(x_location - 320))  # PID로 조향각 계산
        self.publish_drive_cmd(speed, steering)

        # 디버깅용 이미지 출력
        cv2.imshow("Original Image", frame_resized)
        cv2.imshow("Yellow Mask", yellow_mask)
        cv2.imshow("Red Mask", red_mask)
        cv2.imshow("Filtered Image", filtered_img)
        cv2.imshow("Warped Image", warped_img)
        cv2.imshow("Output Image", out_img)
        cv2.waitKey(1)

    def update_hsv_ranges(self):
        # 트랙바에서 HSV 범위를 업데이트
        self.lower_yellow = np.array([cv2.getTrackbarPos("Yellow Lower H", "Trackbars"),
                                      cv2.getTrackbarPos("Yellow Lower S", "Trackbars"),
                                      cv2.getTrackbarPos("Yellow Lower V", "Trackbars")])
        self.upper_yellow = np.array([cv2.getTrackbarPos("Yellow Upper H", "Trackbars"),
                                      cv2.getTrackbarPos("Yellow Upper S", "Trackbars"),
                                      cv2.getTrackbarPos("Yellow Upper V", "Trackbars")])

        self.lower_white = np.array([cv2.getTrackbarPos("White Lower H", "Trackbars"),
                                     cv2.getTrackbarPos("White Lower S", "Trackbars"),
                                     cv2.getTrackbarPos("White Lower V", "Trackbars")])
        self.upper_white = np.array([cv2.getTrackbarPos("White Upper H", "Trackbars"),
                                     cv2.getTrackbarPos("White Upper S", "Trackbars"),
                                     cv2.getTrackbarPos("White Upper V", "Trackbars")])

    def publish_drive_cmd(self, motor, steering):
        # 주행 명령 퍼블리시
        self.ctrl_cmd_msg.speed = motor
        self.ctrl_cmd_msg.angle = steering
        self.ctrl_cmd_pub.publish(self.ctrl_cmd_msg)

if __name__ == "__main__":
    lane_detection_ros = LaneDetectionROS()