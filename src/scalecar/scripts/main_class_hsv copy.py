#!/usr/bin/env python

from this import d
import rospy
from time import sleep, time
import statistics
from collections import deque

# Steering_angle --> 입력 가능한 범위가 -0.34 ~ +0.34 까지 입력 가능
# speed --> 실차량 기준은 -10m/s ~ 10m/s (권장 사항은 -2.5~ 2.5m/s 만 사용하는 것을 권장)


## HSV 50 50 30 그늘 있을 때
## battery 9.47

from ackermann_msgs.msg import AckermannDriveStamped
from std_msgs.msg import Int32, String, Float32
from sensor_msgs.msg import Image
from fiducial_msgs.msg import Fiducial, FiducialArray
from obstacle_detector.msg import Obstacles

from cv_bridge import CvBridge
import cv2
import numpy as np


class MotorController:
    def __init__(self):
        rospy.init_node('motor_controller', anonymous=True)
        
        # 제어 명령을 위한 Publisher
        self.motor_pub = rospy.Publisher('/motor_cmd', Int32, queue_size=1)
        self.servo_pub = rospy.Publisher('/servo_cmd', Int32, queue_size=1)
        self.drive_pub = rospy.Publisher("high_level/ackermann_cmd_mux/input/nav_0", AckermannDriveStamped, queue_size=1)

        # 초기 상태 변수 설정
        self.x_location = None
        self.motor_speed = 0.7  # 기본 속도 설정
        self.angle = 0.0

        # 주기적인 데이터 처리 설정 (30 Hz)
        rospy.Timer(rospy.Duration(1.0 / 30.0), self.timer_callback)

        # 카메라 데이터 Subscriber
        self.camera_sub = rospy.Subscriber('/lane_x_location', Float32, self.camera_callback)

    def camera_callback(self, data):
        # 카메라의 x 위치 데이터 처리
        self.x_location = data.data

    def process_data(self):
        # x_location에 따라 속도와 조향 각도 설정
        self.angle = (self.x_location - 256) * 0.003  # 중심 기준으로 조향 계산
        publishing_data = AckermannDriveStamped()
        publishing_data.header.stamp = rospy.Time.now()
        publishing_data.header.frame_id = "base_link"
        publishing_data.drive.steering_angle = self.angle * 0.003
        publishing_data.drive.speed = self.motor_speed
        self.drive_pub.publish(publishing_data)

    def timer_callback(self, event):
        if self.x_location is not None:
            self.process_data()

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    controller = MotorController()
    try:
        controller.run()
    except rospy.ROSInterruptException:
        pass