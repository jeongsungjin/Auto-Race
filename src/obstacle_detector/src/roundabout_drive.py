#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from obstacle_detector.msg import Obstacles
from std_msgs.msg import String, Int32, Bool
from lane_detection.msg import Drive_command
import math

class Obstacle:
    def __init__(self, x=None, y=None, distance=None):
        self.x = x
        self.y = y
        self.distance = distance

class ROUNDABOUT:
    def __init__(self):
        rospy.Subscriber("/raw_obstacles_roundabout", Obstacles, self.obstacleCB)
        rospy.Subscriber("/mode", String, self.modeCB)
        rospy.Subscriber("/white_cnt", Int32, self.whiteCB)

        self.ctrl_cmd_pub = rospy.Publisher('/motor_roundabout', Drive_command, queue_size=1)
        self.lane_topic_pub = rospy.Publisher("/lane_topic", String, queue_size=1)  
        self.roundabout_done_pub = rospy.Publisher("/roundabout_done", Bool, queue_size=1)  

        self.ctrl_cmd_msg = Drive_command()

        # 변수 초기화
        self.obstacles = []
        self.prev_obstacle = None
        self.distance_threshold = 0.3
        self.mode = ''
        self.flag = False
        self.roundabout_done_flag = False
        self.speed = 0.5
        self.lane_topic = ""

        self.white_cnt = 0
        self.rate = rospy.Rate(30)

        while not rospy.is_shutdown():
            if self.mode == 'RABACON' or self.mode == 'SIGN' or self.mode == 'DYNAMIC' or self.mode == 'STATIC':
                continue

            print("white count!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!1",self.white_cnt)

            if (len(self.obstacles) > 0) and 250 < self.white_cnt < 800:
                self.publishCtrlCmd(0.0 , 0.0, True)
                self.publish_Lane_topic("RIGHT")
            elif self.white_cnt > 800:
                self.roundabout_done_flag = True
                self.publish_roundabout_done(self.roundabout_done_flag)
            else:
                self.publishCtrlCmd(0.0 , 0.0, False)




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

    def modeCB(self, msg):
        self.mode = msg.data

    def whiteCB(self, msg):
        self.white_cnt = msg.data
    def publish_roundabout_done(self, roundabout_done):
        self.roundabout_done_pub.publish(roundabout_done)

    def publish_Lane_topic(self, lane_topic):
        self.lane_topic_pub.publish(lane_topic)
        

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
