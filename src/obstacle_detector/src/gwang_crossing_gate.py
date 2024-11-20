#!/usr/bin/env python
#-*- coding: utf-8 -*-

import rospy
import numpy as np
from sensor_msgs.msg import LaserScan, PointCloud
from obstacle_detector.msg import Obstacles
from std_msgs.msg import Float64, String, Int32, Bool
from ackermann_msgs.msg import AckermannDriveStamped
from obstacle_detector.msg import Drive_command
import math

class Obstacle:
    def __init__(self, x=None, y=None, distance=None):
        self.x = x
        self.y = y
        self.distance = distance


class CrossingGate():
    def __init__(self):
        rospy.Subscriber("/roi_points", PointCloud, self.roiPointsCB)
        rospy.Subscriber("/mode", String, self.modeCB)
        rospy.Subscriber("/white_cnt", Int32, self.whiteCB)
        rospy.Subscriber("/roundabout_done", Bool, self.roundabout_done_callback)

        self.ctrl_cmd_pub = rospy.Publisher('/motor_static', Drive_command, queue_size=1)
        self.ctrl_cmd_msg = Drive_command()
        self.crossing_gate_done_pub = rospy.Publisher('/crossing_gate_done', Bool, queue_size=1)

        self.roi_points = []
        self.x_points = []
        self.y_points = []
        self.speed = 0
        self.angle = 0
        self.closest_obstacle = None
        self.y_min = -0.5
        self.y_max = 0.5
        self.point_count_threshold = 50  # 차단기로 간주할 점의 개수 기준
        self.white_cnt = 0
        self.stop_cnt = 0
        # self.avg_y = None
        self.mode = ''
        self.flag= False
        self.stop_flag = False
        self.forever_true = False
        self.roundabout_completed = False #임시임!!!!!
        self.crossing_gate_done_flag = False
        self.point_count_in_y_range = 0  # y 범위 내 점 개수

        self.version = rospy.get_param('~version', 'safe')
        self.direction = rospy.get_param('~direction', 'left')

        if self.version == 'fast':
            self.speed = 0.7 # 일단
        else:
            self.speed = 0.4

        rospy.loginfo(f"STATIC: {self.version}")

    
        rate = rospy.Rate(30)
        while not rospy.is_shutdown():
            if self.mode == 'RABACON' or self.mode == 'SIGN' or self.mode == 'DYNAMIC' or self.mode == 'ROUNDABOUT':
                continue

            if (self.x_points) and (self.y_points) and (self.roundabout_completed == True) and (self.crossing_gate_done_flag == False):
                if (-0.55 < self.x_points[0] < 0) and (-0.15 <= self.y_points[0] <= 0.35) and (len(self.x_points) > 55): # 좌표기반 말고 뭐든지.. 새로운 조건을 and로 주세요 카메라를 쓰던, 라이다클러스터링을 쓰던, 카운터를 쓰던
                    self.publishCtrlCmd(0.0 , 0.0, True)
                    self.stop_flag = True
                elif self.stop_flag == True:
                    self.forever_true = True
                    self.crossing_gate_done_flag = True
                    self.publishCtrlCmd(self.speed , self.angle, False)

            else:
                self.publishCtrlCmd(self.speed , self.angle, False)
            
            self.publish_crossing_gate(self.crossing_gate_done_flag)


            rate.sleep()
                        
    def whiteCB(self, msg):
        self.white_cnt = msg.data

    def roundabout_done_callback(self, msg):
        self.roundabout_completed = msg.data

    def roiPointsCB(self, msg):
        self.roi_points = []
        self.x_points = []
        self.y_points = []

        for point in msg.points:
            self.roi_points.append((point.x, point.y))
            self.x_points.append(point.x)
            self.y_points.append(point.y)
    
    def modeCB(self, msg):
        self.mode = msg.data

    def publish_crossing_gate(self, crossing_gate_done):
        self.crossing_gate_done_pub.publish(crossing_gate_done)

    def publishCtrlCmd(self, motor_msg, servo_msg, flag):
        self.ctrl_cmd_msg.speed = (motor_msg)  # 모터 속도 설정
        self.ctrl_cmd_msg.angle = (servo_msg)  # 조향각 설정
        self.ctrl_cmd_msg.flag = flag
        self.ctrl_cmd_pub.publish(self.ctrl_cmd_msg)  # 명령 퍼블리시


if __name__ == '__main__':
    rospy.init_node('crossing_gate', anonymous=True)
    try:
        crossing_gate = CrossingGate()
    except rospy.ROSInterruptException:
        pass
