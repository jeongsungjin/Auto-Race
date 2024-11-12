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


class DynamicAvoidance():
    def __init__(self):
        rospy.Subscriber("/raw_obstacles_static", Obstacles, self.obstacleCB)
        rospy.Subscriber("/mode", String, self.modeCB)

        self.ctrl_cmd_pub = rospy.Publisher('/motor_dynamic', Drive_command, queue_size=1)
        self.ctrl_cmd_msg = Drive_command()

        self.ctrl_lane = Drive_command()
        self.obstacles = []  # __init__에서 초기화

        self.closest_obstacle = Obstacle()
        self.speed = 0
        self.angle = 0
        self.dynamic_obstacle_y_list = []
        self.dynamic_obstacle_y_list_max_len = 20
        self.obstacle_y_diff_threshold = 0.2
        self.mode = ''
        self.flag = False
        self.version = rospy.get_param('~version', 'safe')
        self.direction = rospy.get_param('~direction', 'left')

        if self.version == 'fast':
            self.speed = 0.7 
        else:
            self.speed = 0.35

        rospy.loginfo(f"STATIC: {self.version}")

    
        rate = rospy.Rate(30)
        while not rospy.is_shutdown():

            if self.mode == 'RABACON' or self.mode == 'SIGN':
                continue

            if len(self.obstacles) > 0:
                for obstacle in self.obstacles:
                    if (-0.6 < obstacle.x < 0) and (-0.6 <= obstacle.y <= 0.6):
                        self.flag = True
                        self.speed = 0.35
                        self.dynamic_obstacle_y_list.append(obstacle.y)
                        if len(self.dynamic_obstacle_y_list) > self.dynamic_obstacle_y_list_max_len:
                            self.dynamic_obstacle_y_list.pop(0)
            else:
                self.dynamic_obstacle_y_list = []
                self.flag = False


            # Obstacle 이 차폭내에 들어올 때 멈추기
            if len(self.dynamic_obstacle_y_list) >= 2:
                self.flag = True
                if -0.3 < self.dynamic_obstacle_y_list[-1] < 0.3:
                    print('정지!!!')
                    self.publishCtrlCmd(0.0, self.ctrl_lane.angle, self.flag)
                    continue
                self.flag = False


            # Obstacle 이 움직일 때 멈추기
            if len(self.dynamic_obstacle_y_list) >= 2:
                self.flag = True
                # print("동적 Y좌표 차이 : ", self.obstacle_y_diff)
                self.obstacle_y_diff = abs(self.dynamic_obstacle_y_list[1] - self.dynamic_obstacle_y_list[-2])
                if self.obstacle_y_diff > self.obstacle_y_diff_threshold:
                    print('정지!!!')
                    self.publishCtrlCmd(0.0, self.ctrl_lane.angle, self.flag)
                    continue
                self.flag = False

            self.publishCtrlCmd(self.speed, self.angle, self.flag)

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

    def ctrlLaneCB(self, msg):
        self.ctrl_lane.speed = msg.speed
        self.ctrl_lane.angle = msg.angle
        self.lane_mode_flag = msg.flag

    def modeCB(self, msg):
        self.mode = msg.data

    def publishCtrlCmd(self, motor_msg, servo_msg, flag):
        self.ctrl_cmd_msg.speed = round(motor_msg)  # 모터 속도 설정
        self.ctrl_cmd_msg.angle = round(servo_msg)  # 조향각 설정
        self.ctrl_cmd_msg.flag = flag
        self.ctrl_cmd_pub.publish(self.ctrl_cmd_msg)  # 명령 퍼블리시


if __name__ == '__main__':
    rospy.init_node('dynamic_obstacle_avoidance', anonymous=True)
    try:
        dynamic_obstacle_avoidance = DynamicAvoidance()
    except rospy.ROSInterruptException:
        pass
