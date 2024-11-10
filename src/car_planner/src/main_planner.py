#!/usr/bin/env python
#-*- coding: utf-8 -*-
from __future__ import print_function
from this import d
import rospy
from time import sleep, time
import statistics
from collections import deque

# Steering_angle --> 입력 가능한 범위가 -0.34 ~ +0.34 까지 입력 가능
# speed --> 실차량 기준은 -10m/s ~ 10m/s (권장 사항은 -2.5~ 2.5m/s 만 사용하는 것을 권장)


## HSV 50 50 30 그늘 있을 때
## battery 9.47
from car_planner.msgs import Drive_command
from ackermann_msgs.msg import AckermannDriveStamped
from std_msgs.msg import Int32, String, Float32
from sensor_msgs.msg import Image
from obstacle_detector.msg import Obstacles
from cv_bridge import CvBridge
import cv2
import os
import numpy as np

class Obstacle:
    def __init__(self, x=None, y=None, distance=None):
        self.x = x
        self.y = y
        self.distance = distance

def nothing(x):
    pass

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

class Controller():

    def __init__(self):
        self.publishing_data = AckermannDriveStamped()

        self.ctrl_lane = Drive_command()  # 모터 제어 메시지 초기화
        self.ctrl_static = Drive_command()
        self.ctrl_rabacon = Drive_command()
        self.ctrl_sign = Drive_command()
        self.ctrl_dynamic = Drive_command()

        rospy.init_node('main_planner', anonymous=True)  # ROS 노드 초기화

        # 이미지 처리 및 장애물 관련 데이터 구독 걍 미션 별 메시지 타입 하나로 통일 해야겠다!!!!!!!!!!!!!!
        rospy.Subscriber("/motor_lane", Drive_command, self.ctrlLaneCB)  # 카메라 이미지 구독 (차선 인식용)
        rospy.Subscriber("/motor_static", Drive_command, self.ctrlStaticCB)  # LiDAR 기반 객체 탐지 경고 신호 구독 (안전/경고)
        rospy.Subscriber("/motor_dynamic", Drive_command, self.ctrlDynamicCB)  # 장애물 상태 데이터 구독
       
        rospy.Subscriber("/motor_sign", Drive_command, self.ctrlSIGNCB)  # 표지판 ID (어린이 보호구역 신호 등) 구독
        rospy.Subscriber("/motor_rabacon", Drive_command, self.ctrlRabaconCB)  # rabacon 미션용 주행 데이터 구독
        
        
        rospy.Subscriber("obstacles", Obstacles, self.staticObstacle_callback)  # 장애물 데이터 구독
        rospy.Subscriber("/raw_obstacles_rubbercone", Obstacles, self.rubberconeObstacleCB)

        self.drive_pub = rospy.Publisher("high_level/ackermann_cmd_mux/input/nav_0", AckermannDriveStamped, queue_size=1)  # 차량 주행 명령 퍼블리셔
        self.mode_pub = rospy.Publisher('/mode', String, queue_size=1)

        self.bridge = CvBridge()  # CV-Bridge 초기화

        self.version = rospy.get_param('~version', 'safe')

        rospy.loginfo(f"PLANNER_FULL: {self.version}")

        self.steer = 0.0  # 조향각 초기화
        self.motor = 0.0  # 모터 속도 초기화

        self.sign_mode_flag = False
        self.rabacon_mode_flag = False
        self.static_mode_flag = False
        self.dynamic_mode_flag = False
        self.lane_mode_flag = True

        # mode
        self.mode = ''
        self.prev_mode = ''

        self.start_flag = False
        self.kill_flag = False

        self.steer_queue_size = 30
        self.steer_queue = np.zeros(self.steer_queue_size, dtype=int)

        self.pid = PID(0.7, 0.0008, 0.15)

        self.static_obstacles = []
        self.rubbercone_obstacles = []

        rate = rospy.Rate(30)  # 루프 주기 설정
        while not rospy.is_shutdown():  # ROS 노드가 종료될 때까지 반복

            # MODE 판별
            if self.sign_mode_flag == True:
                self.mode = 'SIGN'
            elif self.rabacon_mode_flag == True:
                self.mode = 'RABACON'
            elif self.static_mode_flag == True:
                self.mode = 'STATIC'
            elif self.dynamic_mode_flag == True:
                self.mode = 'DYNAMIC'
            else:
                self.mode = 'LANE'

            self.prev_mode = self.mode

            self.mode_pub.publish(self.mode)

            # MODE에 따른 motor, steer 설정
            if self.mode != '':
                if self.mode == 'SIGN':
                    self.motor = self.ctrl_sign.speed
                    self.steer = self.ctrl_sign.angle
                elif self.mode == 'RABACON':
                    self.motor = self.ctrl_rabacon.speed
                    self.steer = self.ctrl_rabacon.angle
                elif self.mode == 'STATIC':
                    self.motor = self.ctrl_static.speed
                    self.steer = self.ctrl_static.angle
                elif self.mode == "DYNAMIC":
                    self.motor = self.ctrl_dynamic.speed
                    self.steer = self.ctrl_dynamic.angle
                else:               # LANE
                    self.motor = self.ctrl_lane.speed
                    self.steer = self.ctrl_lane.angle
            else:
                rospy.logwarn("SOMETHING WRONG")
                continue

            # --------------------------- 장애물 인지시 감속 --------------------------- # 
            if (len(self.static_obstacles) > 0) and (self.mode != "RABACON") and (self.mode != "SIGN"):

                if (-10 <= np.mean(self.steer_queue) <= 10) and (np.var(self.steer_queue) < 150):
                    # 특정 roi에 인지가 들어오면 일단 감속, 우리의 차에 맞는 스티어링 값에 따라 값 조정 해야함!!!!!!!!!!!!!!!
                    for obstacle in self.static_obstacles:
                        if (0 < obstacle.x < 1.5) and (-0.25 <= obstacle.y <= 0.25):
                            self.motor = 0.35 #실제 감속 속도에 맞게 튜닝

            # --------------------------- 장애물 인지시 감속 --------------------------- # 


            # --------------------------- 라바콘 인지시 감속 --------------------------- # 
            # if len(self.rubbercone_obstacles) > 0:
            #     # 특정 roi에 인지가 들어오면 일단 감속
            #     for obstacle in self.rubbercone_obstacles:
            #         if (0 < obstacle.x < 2.0) and (-0.45 <= obstacle.y <= 0.45):
            #             self.motor = 30
            # --------------------------- 라바콘 인지시 감속 --------------------------- # 

            rospy.loginfo(f"MODE: {self.mode}")
            rospy.loginfo(f"SPEED: {self.motor}")

            self.publishCtrlCmd(self.motor, self.steer)


            cv2.waitKey(1)  # 키 입력 대기
            rate.sleep()  # 주기마다 대기
                
        cv2.destroyAllWindows()  # 창 닫기

    def publishCtrlCmd(self, motor_msg, servo_msg):

        self.publishing_data = AckermannDriveStamped()  # AckermannDriveStamped 메시지 객체 생성
        self.publishing_data.header.stamp = rospy.Time.now()  # 메시지에 현재 시간을 기록
        self.publishing_data.header.frame_id = "base_link"  # 메시지의 참조 프레임을 "base_link"로 설정
        self.publishing_data.drive.steering_angle = servo_msg * 0.003  # 차선 위치 오차에 따라 조향 각도 결정
        self.publishing_data.drive.speed = motor_msg  # 차선 주행 속도(speed_lane)를 설정
        self.drive_pub.publish(self.publishing_data)  # 설정한 속도와 조향 각도 메시지를 퍼블리시하여 차량에 적용


    def ctrlLaneCB(self, msg):
        self.ctrl_lane.speed = msg.speed
        self.ctrl_lane.angle = msg.angle
        self.lane_mode_flag = msg.flag

    def ctrlStaticCB(self, msg):
        self.ctrl_static.speed = msg.speed
        self.ctrl_static.angle = msg.angle
        self.static_mode_flag = msg.flag

    def ctrlDynamicCB(self, msg):
        self.ctrl_dynamic.speed = msg.speed
        self.ctrl_dynamic.angle = msg.angle
        self.dynamic_mode_flag = msg.flag

    def ctrlRabaconCB(self, msg):
        self.ctrl_rabacon.speed = msg.speed
        self.ctrl_rabacon.angle = msg.angle
        self.rabacon_mode_flag = msg.flag

    def ctrlSIGNCB(self, msg):
        self.ctrl_sign.speed = msg.speed
        self.ctrl_sign.angle = msg.angle
        self.sign_mode_flag = msg.flag

        
    def staticObstacle_callback(self, msg):
        self.static_obstacles = []
        for circle in msg.circles:
            x = circle.center.x
            y = circle.center.y
            distance = (x**2 + y**2) ** 0.5  # 유클리드 거리 계산
            obstacle = Obstacle(x, y, distance)
            self.static_obstacles.append(obstacle)
        
        self.static_obstacles.sort(key=lambda obs: obs.distance)

        if len(self.static_obstacles) > 0:
            self.closest_static_obstacle = self.static_obstacles[0]
        else:
            self.closest_static_obstacle = Obstacle()

    def rubberconeObstacleCB(self, msg):
        self.rubbercone_obstacles = []
        for circle in msg.circles:
            x = circle.center.x
            y = circle.center.y
            distance = (x**2 + y**2) ** 0.5  # 유클리드 거리 계산
            obstacle = Obstacle(x, y, distance)
            self.rubbercone_obstacles.append(obstacle)
        
        self.rubbercone_obstacles.sort(key=lambda obs: obs.distance)

        if len(self.rubbercone_obstacles) > 0:
            self.closest_rubbercone_obstacle = self.rubbercone_obstacles[0]
        else:
            self.closest_rubbercone_obstacle = Obstacle()

if __name__ == '__main__':
    try:
        controller = Controller()  
    except rospy.ROSInterruptException:
        pass 