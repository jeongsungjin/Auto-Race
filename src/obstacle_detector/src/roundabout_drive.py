#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from obstacle_detector.msg import Obstacles
from std_msgs.msg import String
from lane_detection.msg import Drive_command
import math

class Obstacle:
    def __init__(self, x=None, y=None, distance=None):
        self.x = x
        self.y = y
        self.distance = distance

class ROUNDABOUT:
    def __init__(self):
        rospy.Subscriber("/raw_obstacles_static", Obstacles, self.obstacleCB)
        rospy.Subscriber("/mode", String, self.modeCB)
        
        self.ctrl_cmd_pub = rospy.Publisher('/motor_roundabout', Drive_command, queue_size=1)
        self.ctrl_cmd_msg = Drive_command()

        # 변수 초기화
        self.obstacles = []
        self.prev_obstacle = None
        self.distance_threshold = 0.3
        self.mode = ''
        self.flag = False
        self.speed = 0.5

        self.rate = rospy.Rate(30)

        while not rospy.is_shutdown():
            if self.mode == 'RABACON' or self.mode == 'SIGN' or self.mode == 'DYNAMIC':
                continue

            if (len(self.obstacles) > 0):
                closest_obstacle = self.obstacles[0]
                self.processObstacle(closest_obstacle)
            
            self.rate.sleep()

    def obstacleCB(self, msg):

        self.obstacles = []
        for circle in msg.circles:
            x = circle.center.x
            y = circle.center.y
            distance = math.sqrt(x**2 + y**2)
            obstacle = Obstacle(x, y, distance)
            self.obstacles.append(obstacle)

        self.obstacles.sort(key=lambda obs: obs.distance)

    def processObstacle(self, closest_obstacle):
        # 장애물과의 거리 변화 비교
        if self.prev_distance is not None:
            distance_diff = self.prev_distance - closest_obstacle.distance

            # 장애물이 다가오고 있다면 flag 활성화 및 정지
            if distance_diff > 0.05:  # 다가오는 중 (0.05m 이상 가까워질 때)
                rospy.loginfo("장애물이 다가옴: 정지")
                self.publishCtrlCmd(0.0, 0.0, True)
                self.flag = True  # flag를 활성화해 회전 교차로 상태 알림

            # 장애물이 멀어지고 있다면 flag 비활성화 및 주행 재개
            elif distance_diff < -self.distance_threshold:  # 멀어지고 있는 경우
                rospy.loginfo("장애물이 멀어짐: 주행 재개")
                self.publishCtrlCmd(self.speed, 0.0, True)
                self.flag = False  # flag를 비활성화해 회전 교차로 상태 해제

        # 이전 장애물의 거리를 현재 거리로 업데이트
        self.prev_distance = closest_obstacle.distance


    def modeCB(self, msg):
        self.mode = msg.data

    def publishCtrlCmd(self, motor_msg, servo_msg, flag):
        self.ctrl_cmd_msg.speed = round(motor_msg)
        self.ctrl_cmd_msg.angle = round(servo_msg)
        self.ctrl_cmd_msg.flag = flag
        self.ctrl_cmd_pub.publish(self.ctrl_cmd_msg)

if __name__ == '__main__':
    rospy.init_node('roundabout_drive', anonymous=True)
    try:
        roundabout_drive = ROUNDABOUT()
    except rospy.ROSInterruptException:
        pass
