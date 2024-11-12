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


class StaticAvoidance():
    def __init__(self):
        rospy.Subscriber("/raw_obstacles_static", Obstacles, self.obstacleCB)
        rospy.Subscriber("/heading", Float64, self.headingCB)
        rospy.Subscriber("/mode", String, self.modeCB)


        self.ctrl_cmd_pub = rospy.Publisher('/motor_static', Drive_command, queue_size=1)
        self.ctrl_cmd_msg = Drive_command()

        # self.state
        # L: Lane
        # A: Avoid
        # R: Return
        self.state = 'L'

        self.obstacles = []
        self.is_static = False
        self.steer = 0
        self.speed = 0
        self.is_left = False

        self.closest_obstacle = Obstacle()

        # self.distance_threshold = 0.45
        self.distance_threshold = 1.2       # 정적 회피 시작거리 
        self.distance_threshold_max = 1.25   # 정적 회피중 다른 장애물과의 구분을 위한 임계점
        self.margin_y = 0.4

        self.angle = 0

        self.static_obstacle_cnt = 0
        self.static_obstacle_cnt_threshold = 15
        self.frames_without_obstacle = 0
        self.allowed_frames_without_obstacle = 5

        self.real_heading = None
        self.gt_heading = None

        self.avoid_heading = None
        self.return_heading = None

        self.local_heading = None

        self.last_n_obstacles_y = []
        self.len_last_n_obstacles_y = 5
        # self.avg_y = None
        self.mode = ''

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
                #-----------------------------------------------------------------------------------#
                for obstacle in self.obstacles:
                    continue
                #-----------------------------------------------------------------------------------#

            # self.static_pub.publish(self.steer)
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


    def update_last_n_obstacles_y(self, y, n):
        self.last_n_obstacles_y.append(y)
        if len(self.last_n_obstacles_y) > n:
            self.last_n_obstacles_y.pop(0)

    def publishCtrlCmd(self, motor_msg, servo_msg, flag):
        self.ctrl_cmd_msg.speed = round(motor_msg)  # 모터 속도 설정
        self.ctrl_cmd_msg.angle = round(servo_msg)  # 조향각 설정
        self.ctrl_cmd_msg.flag = flag
        self.ctrl_cmd_pub.publish(self.ctrl_cmd_msg)  # 명령 퍼블리시


if __name__ == '__main__':
    rospy.init_node('static_obstacle_avoidance', anonymous=True)
    try:
        static_obstacle_avoidance = StaticAvoidance()
    except rospy.ROSInterruptException:
        pass
