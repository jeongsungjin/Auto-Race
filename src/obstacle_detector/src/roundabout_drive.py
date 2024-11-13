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

    def detectRoundabout(self, msg):
        for circle in msg.circles:
            x = circle.center.x
            y = circle.center.y
            distance = math.sqrt(x**2 + y**2)

            if -1.0 < x < 0 and abs(y) < 1.0:
                if self.prev_obstacle:
                    distance_diff = abs(self.prev_obstacle.distance - distance)
                    if distance_diff < self.distance_threshold:
                        self.flag = True
                        rospy.loginfo("회전 교차로 차량 감지: 회전 교차로 모드 활성화")
                        return
                self.prev_obstacle = Obstacle(x, y, distance)
        self.flag = False

    def obstacleCB(self, msg):
        if not self.flag:
            return

        self.obstacles = []
        for circle in msg.circles:
            x = circle.center.x
            y = circle.center.y
            distance = math.sqrt(x**2 + y**2)
            obstacle = Obstacle(x, y, distance)
            self.obstacles.append(obstacle)

        self.obstacles.sort(key=lambda obs: obs.distance)

    def processObstacle(self, closest_obstacle):
        if self.prev_obstacle is not None:
            distance_diff = abs(self.prev_obstacle.distance - closest_obstacle.distance)
            if distance_diff > self.distance_threshold:
                rospy.loginfo("장애물이 멀어짐: 주행 재개 및 회전 교차로 모드 비활성화")
                self.publishCtrlCmd(self.speed, 0.0, True)
                self.flag = False  # 회전 교차로 모드 비활성화
            else:
                rospy.loginfo("장애물이 가까워짐: 정지")
                self.publishCtrlCmd(0.0, 0.0, True)
        else:
            rospy.loginfo("장애물 감지: 정지")
            self.flag = False
        self.prev_obstacle = closest_obstacle

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
