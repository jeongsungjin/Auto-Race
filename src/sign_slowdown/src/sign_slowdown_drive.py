#!/usr/bin/env python
#-*- coding: utf-8 -*-

import rospy
from std_msgs.msg import Int32, String
from sensor_msgs.msg import Image
from fiducial_msgs.msg import Fiducial, FiducialArray
from ackermann_msgs.msg import AckermannDriveStamped  # AckermannDrive 메시지를 퍼블리시하기 위한 import
from sign_slowdown.msg import Drive_command

class Sign():
    def __init__(self):
        rospy.init_node('sign_slowdown', anonymous=True)  # 노드 초기화 추가

        self.sign_data = ""  # 표지판 데이터 저장
        self.A_cnt = 0        
        self.B_cnt = 0
        self.flag = False 
        self.lane_topic = ""
        self.speed_lane = 0.7 
        self.steer_lane = 0.0 
        self.ctrl_lane = Drive_command()
        self.lane_topic_pub = rospy.Publisher("/lane_topic", String, queue_size=1)  

        rospy.Subscriber("/fiducial_vertices", FiducialArray, self.sign_callback)
        rospy.Subscriber("/motor_lane", Drive_command, self.ctrlLaneCB)

        self.no_sign_cnt = 0
        
        self.rate = rospy.Rate(30)  # 30hz

        self.ctrl_cmd_msg = Drive_command()

        self.version = rospy.get_param('~version', 'safe')

        rospy.loginfo(f"SIGN: {self.version}")

    def run(self):
        while not rospy.is_shutdown():
            # rospy.loginfo("################## ID : {}".format(self.sign_data))            if self.lane_topic:  # self.lane_topic이 None이 아니면 퍼블리시
            self.publish_Lane_topic(self.lane_topic)
            self.rate.sleep()


    def sign_callback(self, _data):
        if (len(_data.fiducials) > 0):
            self.sign_data = _data.fiducials[0].fiducial_id
            if isinstance(self.sign_data, int):
                if self.sign_data == 0:  # 0을 "A"로 매핑
                    self.A_cnt += 1                                
                    if self.A_cnt >= 20:
                        self.lane_topic = "LEFT"
                        self.A_cnt = 0
                elif self.sign_data == 1:  # 1을 "B"로 매핑
                    self.B_cnt += 1                                
                    if self.B_cnt >= 20:
                        self.lane_topic = "RIGHT"
                        self.B_cnt = 0
            else:
                rospy.logwarn("Unknown fiducial_id format")



    def ctrlLaneCB(self, msg):
        self.ctrl_lane.speed = msg.speed
        self.ctrl_lane.angle = msg.angle
        self.lane_mode_flag = msg.flag
        
    def publish_Lane_topic(self, lane_topic):
        self.lane_topic_pub.publish(lane_topic)
        

if __name__ == '__main__':
    try:
        node = Sign()
        node.run()
    except rospy.ROSInterruptException:
        pass