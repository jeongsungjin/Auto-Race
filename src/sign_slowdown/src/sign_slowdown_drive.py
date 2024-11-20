#!/usr/bin/env python
#-*- coding: utf-8 -*-

import rospy
from std_msgs.msg import Int32, String, Bool
from sensor_msgs.msg import Image

from ackermann_msgs.msg import AckermannDriveStamped  # AckermannDrive 메시지를 퍼블리시하기 위한 import
from sign_slowdown.msg import Drive_command
from ar_track_alvar_msgs.msg import AlvarMarkers

class Sign():
    def __init__(self):
        rospy.init_node('sign_slowdown', anonymous=True)  # 노드 초기화 추가


        self.lane_topic_pub = rospy.Publisher("/lane_topic", String, queue_size=1)  
        self.tunnel_done_pub = rospy.Publisher('/tunnel_done', Bool, queue_size=1)

        rospy.Subscriber("/ar_pose_marker", AlvarMarkers, self.sign_callback, queue_size=1)
        rospy.Subscriber("/motor_lane", Drive_command, self.ctrlLaneCB)
        
        self.lane_topic = ""
        self.lane_dir = ""
        self.speed_lane = 0.7 
        self.steer_lane = 0.0 
        self.ctrl_lane = Drive_command()

        self.sign_data = ""  # 표지판 데이터 저장
        self.A_cnt = 0        
        self.B_cnt = 0
        self.flag = False 
        self.tunnel_done_flag = False

        self.no_sign_cnt = 0
        
        self.rate = rospy.Rate(30)  # 30hz

        self.ctrl_cmd_msg = Drive_command()

        self.version = rospy.get_param('~version', 'safe')

        rospy.loginfo(f"SIGN: {self.version}")

    def run(self):
        while not rospy.is_shutdown():
            if (self.A_cnt + self.B_cnt) >= 5:
                if (self.A_cnt > self.B_cnt):
                    self.tunnel_done_flag = True
                    self.publish_tunnel_done(self.tunnel_done_flag)
                    # rospy.loginfo("################## ID : {}".format(self.sign_data))            if self.lane_topic:  # self.lane_topic이 None이 아니면 퍼블리시
                    self.lane_dir = "LEFT"
                    self.publish_Lane_topic(self.lane_dir)
                else:
                    self.tunnel_done_flag = True
                    self.publish_tunnel_done(self.tunnel_done_flag)

                    self.lane_dir = "RIGHT"
                    self.publish_Lane_topic(self.lane_dir)
            else:
                self.tunnel_done_flag = False
                self.publish_tunnel_done(self.tunnel_done_flag)


            self.rate.sleep()


    def sign_callback(self, _data):
        if not _data.markers:
            rospy.loginfo("No markers detected.")
            self.sign_data = None
            self.lane_topic = ""
            return

        # 첫 번째 마커의 ID를 가져오기
        self.sign_data = _data.markers[0].id

        rospy.loginfo(f"Detected marker ID: {self.sign_data}")

        # ID를 기준으로 A와 B를 구분하고 카운터를 증가
        if self.sign_data == 0:  # ID가 0인 경우
            self.A_cnt += 1
        elif self.sign_data == 4:  # ID가 1인 경우
            self.B_cnt += 1      
                          
        else:
            rospy.logwarn(f"Unknown marker ID: {self.sign_data}")

    def publish_tunnel_done(self, tunnel_done):
        self.tunnel_done_pub.publish(tunnel_done)



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