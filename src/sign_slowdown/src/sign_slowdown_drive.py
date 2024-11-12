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

        self.sign_data = 0  # 표지판 데이터 저장
        self.child_cnt = 0
        self.slow_down_flag = 0  # 감속 플래그
        self.slow_flag = False  # 감속 상태 여부 플래그
        self.slow_t1 = 0  # 감속 시작 시간
        self.speed_slow = 0.35  
        self.speed_lane = 0.7  
        self.ctrl_lane = Drive_command()
        self.ctrl_cmd_pub = rospy.Publisher("/motor_sign", Drive_command, queue_size=1)  # 모터 제어 퍼블리셔
        rospy.Subscriber("/fiducial_vertices", FiducialArray, self.child_sign_callback)
        rospy.Subscriber("/motor_lane", Drive_command, self.ctrlLaneCB)

        self.no_sign_cnt = 0
        
        self.rate = rospy.Rate(30)  # 30hz

        self.ctrl_cmd_msg = Drive_command()

        self.version = rospy.get_param('~version', 'safe')

        rospy.loginfo(f"SIGN: {self.version}")

    def run(self):
        while not rospy.is_shutdown():
            if self.slow_down_flag == 1:
                if self.sign_data == 3:
                    rospy.loginfo(" ===============   SLOW DETECTED, WAIT!!!! ============")
                    self.publishCtrlCmd(self.speed_lane, self.ctrl_lane.angle, self.slow_flag)
                elif self.sign_data == 0:
                    self.child_cnt = 0
                    if self.slow_flag == False:
                        self.slow_t1 = rospy.get_time()
                        self.slow_flag = True
                    t2 = rospy.get_time()
                    while t2 - self.slow_t1 <= 15:
                        rospy.loginfo("************* SLOW DOWN *****************")
                        self.publishCtrlCmd(self.speed_slow, self.ctrl_lane.angle, self.slow_flag)
                        t2 = rospy.get_time()
                    self.slow_down_flag = 0
                    self.slow_flag = False

            self.rate.sleep()


    def child_sign_callback(self, _data):
        if len(_data.fiducials) > 0:
            self.sign_data = _data.fiducials[0].fiducial_id
            if self.sign_data == 3:
                self.child_cnt += 1
                if self.child_cnt >= 20:
                    self.slow_down_flag = 1
                    self.child_cnt = 0
        else:
            self.no_sign_cnt += 1
            if self.no_sign_cnt > 20:
                self.sign_data = 0
                self.no_sign_cnt = 0


    def ctrlLaneCB(self, msg):
        self.ctrl_lane.speed = msg.speed
        self.ctrl_lane.angle = msg.angle
        self.lane_mode_flag = msg.flag
        
    def publishCtrlCmd(self, motor_msg, servo_msg, flag):
        self.ctrl_cmd_msg.speed = motor_msg  # 모터 속도 설정
        self.ctrl_cmd_msg.angle = servo_msg  # 조향각 설정
        self.ctrl_cmd_msg.flag = flag
        self.ctrl_cmd_pub.publish(self.ctrl_cmd_msg)  # 명령 퍼블리시
        

if __name__ == '__main__':
    try:
        node = Sign()
        node.run()
    except rospy.ROSInterruptException:
        pass