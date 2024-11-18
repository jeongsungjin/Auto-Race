#!/usr/bin/env python
#-*- coding: utf-8 -*-

import rospy
from sensor_msgs.msg import LaserScan
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
        rospy.Subscriber("/raw_obstacles", Obstacles, self.obstacleCB)
        rospy.Subscriber("/mode", String, self.modeCB)
        rospy.Subscriber("/scan", LaserScan, self.lidar_callback)
        rospy.Subscriber("/white_cnt", Int32, self.whiteCB)
        rospy.Subscriber("/roundabout_done", Bool, self.roundabout_done_callback)

        self.ctrl_cmd_pub = rospy.Publisher('/motor_static', Drive_command, queue_size=1)
        self.ctrl_cmd_msg = Drive_command()
        self.crossing_gate_done_pub = rospy.Publisher('/crossing_gate_done', Bool, queue_size=1)

        self.obstacles = []
        self.speed = 0
        self.angle = 0
        self.closest_obstacle = None
        self.y_min = -0.5
        self.y_max = 0.5
        self.point_count_threshold = 50  # 차단기로 간주할 점의 개수 기준
        self.white_cnt = 0

        self.gate_obstacle_y_list = []
        self.gate_obstacle_y_list_max_len = 20   # 튜닝필요
        self.y_diff_threshold = 0.2
        # self.avg_y = None
        self.mode = ''
        self.flag= False
        self.stop_flag = False
        self.roundabout_completed = False
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

            if len(self.obstacles) > 0 and self.white_cnt > 300 and self.roundabout_completed == True and self.crossing_gate_done_flag == False:
                if (-0.69 < self.closest_obstacle.x < 0) and (-0.3 <= self.closest_obstacle.y <= 0.3): # 좌표기반 말고 뭐든지.. 새로운 조건을 and로 주세요 카메라를 쓰던, 라이다클러스터링을 쓰던, 카운터를 쓰던
                    self.publishCtrlCmd(0.0 , 0.0, True)
                    self.stop_flag = True
                else:
                    if self.stop_flag == True:
                        self.crossing_gate_done_flag = True
                        self.publishCtrlCmd(self.speed , self.angle, False)
                        self.publish_crossing_gate(self.crossing_gate_done_flag)
            else:
                self.publishCtrlCmd(self.speed , self.angle, False)
                #임시임!
                self.crossing_gate_done_flag = True
                self.publish_crossing_gate(self.crossing_gate_done_flag)


            rate.sleep()
                        
    def whiteCB(self, msg):
        self.white_cnt = msg.data

    def roundabout_done_callback(self, msg):
        self.roundabout_completed = msg.data

    def lidar_callback(self, msg):
        # y 범위 내 점 개수 초기화
        self.point_count_in_y_range = 0

        for i, distance in enumerate(msg.ranges):
            # 유효한 거리만 사용
            if distance < msg.range_min or distance > msg.range_max:
                continue
            
            # y 좌표 계산
            y = distance * math.sin(msg.angle_min + i * msg.angle_increment)

            # 특정 y 범위 내에 있는 점만 카운트
            if self.y_min <= y <= self.y_max:
                self.point_count_in_y_range += 1

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

    def modeCB(self, msg):
        self.mode = msg.data

    def publish_crossing_gate(self, crossing_gate_done):
        self.crossing_gate_done_pub.publish(crossing_gate_done)

    def publishCtrlCmd(self, motor_msg, servo_msg, flag):
        self.ctrl_cmd_msg.speed = round(motor_msg)  # 모터 속도 설정
        self.ctrl_cmd_msg.angle = round(servo_msg)  # 조향각 설정
        self.ctrl_cmd_msg.flag = flag
        self.ctrl_cmd_pub.publish(self.ctrl_cmd_msg)  # 명령 퍼블리시


if __name__ == '__main__':
    rospy.init_node('crossing_gate', anonymous=True)
    try:
        crossing_gate = CrossingGate()
    except rospy.ROSInterruptException:
        pass
