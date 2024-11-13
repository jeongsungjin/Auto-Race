#!/usr/bin/env python
#-*- coding: utf-8 -*-

import rospy
from obstacle_detector.msg import Obstacles
from std_msgs.msg import Float64, String
from ackermann_msgs.msg import AckermannDriveStamped
from obstacle_detector.msg import Drive_command

class Obstacle:
    def __init__(self, x=None, y=None, distance=None):
        self.x = x
        self.y = y
        self.distance = distance


class CrossingGate():
    def __init__(self):
        rospy.Subscriber("/raw_obstacles_static", Obstacles, self.obstacleCB)
        rospy.Subscriber("/mode", String, self.modeCB)


        self.ctrl_cmd_pub = rospy.Publisher('/motor_static', Drive_command, queue_size=1)
        self.ctrl_cmd_msg = Drive_command()

        self.obstacles = []
        self.steer = 0
        self.speed = 0
        self.angle = 0

        self.gate_obstacle_y_list = []
        self.gate_obstacle_y_list_max_len = 20   # 튜닝필요
        self.y_diff_threshold = 0.2
        # self.avg_y = None
        self.mode = ''
        self.flag= False

        self.version = rospy.get_param('~version', 'safe')
        self.direction = rospy.get_param('~direction', 'left')

        if self.version == 'fast':
            self.speed = 0.7 # 일단
        else:
            self.speed = 0.7

        rospy.loginfo(f"STATIC: {self.version}")

        

        rate = rospy.Rate(30)
        while not rospy.is_shutdown():

            if self.mode == 'RABACON' or self.mode == 'SIGN' or self.mode == 'DYNAMIC':
                continue

            if len(self.obstacles) > 0:

                for obstacle in self.obstacles:
                    if len(-2.5 < obstacle.x < 0) and (-0.5 <= obstacle.y <= 0.5): # 좌표기반 말고 뭐든지.. 새로운 조건을 and로 주세요 카메라를 쓰던, 라이다클러스터링을 쓰던, 카운터를 쓰던

                        self.publishCtrlCmd(0.0 , 0.0, True)

                        self.gate_obstacle_y_list.append(obstacle.y)
                        if len(self.gate_obstacle_y_list) > self.gate_obstacle_y_list_max_len:
                            self.gate_obstacle_y_list.pop(0)

                        if len(self.gate_obstacle_y_list) > 2:
                            self.gate_obstacle_y_diff = abs(self.gate_obstacle_y_list[1]-self.gate_obstacle_y_list[-2])
                            
                            if len(self.gate_obstacle_y_diff) > self.y_diff_threshold:
                                self.flag=False
                                continue

            self.publishCtrlCmd(self.speed , self.angle, self.flag)

            rate.sleep()
                        

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
