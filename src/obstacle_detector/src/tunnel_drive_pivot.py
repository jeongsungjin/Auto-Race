#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from obstacle_detector.msg import Obstacles
from std_msgs.msg import String, Int32, Bool
from sensor_msgs.msg import PointCloud  
from lane_detection.msg import Drive_command
import math

class TUNNEL:
    def __init__(self):
        # Subscribers
        rospy.Subscriber("/mode", String, self.modeCB)  # /mode를 modeCB로 연결
        rospy.Subscriber("/roi_points_tunnel", PointCloud, self.roiPointsCB)  # /roi_points_tunnel을 roiPointsCB로 연결
        rospy.Subscriber("/yellow_pixel", Int32, self.yellowCB)
        rospy.Subscriber("/crossing_gate_done", Int32, self.crossing_gate_done_callback)

        # Publishers
        self.ctrl_cmd_pub = rospy.Publisher('/motor_tunnel', Drive_command, queue_size=1)

        # 메시지 객체 초기화
        self.ctrl_cmd_msg = Drive_command()

        # 변수 초기화
        self.roi_points = []  # ROI 포인트 리스트
        self.x_points = []
        self.y_points = []
        self.mode = ''  # 현재 모드
        self.flag = False
        self.yellow_pixels = 0  # 노란색 픽셀 개수
        self.speed = 0.3  # 기본 속도
        self.steer = 0.0  # 조향각
        self.crossing_completed = True #원래 트루임 테스트 하기위해 잠시 올린것
        self.rate = rospy.Rate(30)

        # 메인 루프
        while not rospy.is_shutdown():
            self.control_loop()
            self.rate.sleep()

    def control_loop(self):
        if self.mode in ['RABACON', 'SIGN', 'DYNAMIC', 'STATIC']:
            return

        # 노란색 픽셀 수와 ROI 포인트 기반 제어
        if self.yellow_pixels < 8000 and self.crossing_completed == True:
            if self.y_points and self.y_points[0] < 0 :  # ROI 첫 포인트의 y값이 0보다 작을 경우
                self.steer += 3
                self.publishCtrlCmd(self.speed, self.steer, True)

            elif self.y_points and self.y_points[0] > 0:  # ROI 첫 포인트의 y값이 0보다 클 경우
                self.steer -= 3
                self.publishCtrlCmd(self.speed, self.steer, True)

            else:  
                self.steer = 0.0
                self.publishCtrlCmd(self.speed, self.steer, True)
        else:
            self.publishCtrlCmd(self.speed, self.steer, False)


    def crossing_gate_done_callback(self, msg):
        self.crossing_completed = msg.data

    def roiPointsCB(self, msg):
        self.roi_points = []  
        self.x_points = []  
        self.y_points = []  

        for point in msg.points:
            self.roi_points.append((point.x, point.y))  
            self.x_points.append(point.x)  
            self.y_points.append(point.y)  

    def yellowCB(self, msg):
        self.yellow_pixels = msg.data

    def modeCB(self, msg):
        self.mode = msg.data  # /mode 토픽 데이터 저장

    def publishCtrlCmd(self, motor_msg, servo_msg, flag):
        self.ctrl_cmd_msg.speed = (motor_msg)
        self.ctrl_cmd_msg.angle = (servo_msg)
        self.ctrl_cmd_msg.flag = flag
        self.ctrl_cmd_pub.publish(self.ctrl_cmd_msg)

if __name__ == '__main__':
    rospy.init_node('tunnel_drive', anonymous=True)
    try:
        tunnel_drive = TUNNEL()
    except rospy.ROSInterruptException:
        pass
