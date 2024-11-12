#!/usr/bin/env python3

# 기본 Python3 인터프리터 설정

from __future__ import print_function

from ackermann_msgs.msg import AckermannDriveStamped

from slidewindow_both_lane import SlideWindow  # 슬라이드 윈도우 알고리즘 모듈 임포트
import cv2  # OpenCV 라이브러리 임포트

from cv_bridge import CvBridge, CvBridgeError  # CV-Bridge 라이브러리 임포트

import rospy  # ROS 파이썬 라이브러리 임포트
from sensor_msgs.msg import Image  # 이미지 데이터 메시지 모듈 임포트

from utils import *

from lane_detection.msg import Drive_command

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


class LaneDetection(object):
    def __init__(self):
        rospy.init_node('lane_detection', anonymous=True)  # ROS 노드 초기화
        
        try:
            # 카메라와 IMU 데이터 구독
            rospy.Subscriber("/usb_cam/image_rect_color", Image, self.cameraCB)

            # 모터 제어 명령과 현재 속도 퍼블리셔 설정
            self.ctrl_cmd_pub = rospy.Publisher('/motor_lane', Drive_command, queue_size=1)
            
            self.bridge = CvBridge()  # CV-Bridge 초기화
            self.ctrl_cmd_msg = Drive_command()  # 모터 제어 메시지 초기화
            
            self.slidewindow = SlideWindow()  # 슬라이드 윈도우 알고리즘 초기화

            self.version = rospy.get_param('~version', 'safe')

            rospy.loginfo(f"LANE: {self.version}")

            # ---------------------------튜닝 해야하는 값 ------------------------------#
            self.steer = 0.0  # 조향각 초기화
            self.motor = 0.5  # 모터 속도 초기화
           
            # 원래 잘되던 버전
            if self.version == 'fast':
                self.pid = PID(0.78, 0.0005, 0.405) 
            else:
                self.pid = PID(0.7, 0.0008, 0.15)
            #--------------------------------------------------------------------------#
            self.cv_image = None  # 카메라 이미지 초기화
            
            
            # IMU 기반 속도 계산을 위한 변수 초기화
            rate = rospy.Rate(30)  # 루프 주기 설정
            while not rospy.is_shutdown():  # ROS 노드가 종료될 때까지 반복
                if self.cv_image is not None:  # 카메라 이미지가 있는 경우
                    cropped_image = roi_for_lane(self.cv_image)
                    gray_img, blurred_image, adaptive_gaussian, edged, closed_image = process_image(cropped_image)
                    warped_img = warper(closed_image)

                    out_img, x_location, _ = self.slidewindow.slidewindow(warped_img)  # 슬라이드 윈도우 알고리즘 적용

                    if x_location == None:  # x 위치가 없는 경우
                        x_location = last_x_location  # 이전 x 위치 사용
                    else:
                        last_x_location = x_location  # x 위치 갱신
                    
                    
                    self.steer = round(self.pid.pid_control(x_location - 320))  # PID 제어를 통한 각도 계산

                    if self.version == 'fast':
                        self.motor = 0.7 
                    else:
                        self.motor = 0.35
                    
                    self.publishCtrlCmd(self.motor, self.steer)  # 제어 명령 퍼블리시


                    cv2.imshow('out_img', out_img)
                    

                    cv2.waitKey(1)  # 키 입력 대기



                rate.sleep()  # 주기마다 대기
                
        finally:
            cv2.destroyAllWindows()  # 창 닫기
        
    def publishCtrlCmd(self, motor_msg, servo_msg):
        self.ctrl_cmd_msg.speed = motor_msg  # 모터 속도 설정
        self.ctrl_cmd_msg.angle = servo_msg  # 조향각 설정
        self.ctrl_cmd_msg.flag = True
        self.ctrl_cmd_pub.publish(self.ctrl_cmd_msg)  # 명령 퍼블리시
        
    def cameraCB(self, msg):
        try:
            self.cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")  # ROS 이미지 메시지를 OpenCV 이미지로 변환
        except CvBridgeError as e:
            rospy.logwarn(e)


if __name__ == '__main__':
    try:
        controller = LaneDetection()  # AutopilotControl 객체 생성
    except rospy.ROSInterruptException:
        pass  # 예외 발생 시 무시하고 종료