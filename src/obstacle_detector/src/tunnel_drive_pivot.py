#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from obstacle_detector.msg import Obstacles
from std_msgs.msg import String, Int32, Bool
from sensor_msgs.msg import PointCloud  
from lane_detection.msg import Drive_command
import math

class TUNNEL:
    def __init__(self):
        # Subscribers
        rospy.Subscriber("/mode", String, self.modeCB)  # /mode를 modeCB로 연결
        rospy.Subscriber("/roi_points_tunnel", PointCloud, self.roiPointsCB)  # /roi_points_tunnel을 roiPointsCB로 연결
        rospy.Subscriber("/yellow_pixel", Int32, self.yellowCB)
        rospy.Subscriber("/crossing_gate_done", Bool, self.crossing_gate_done_callback)

        # Publishers
        self.ctrl_cmd_pub = rospy.Publisher('/motor_tunnel', Drive_command, queue_size=1)
        self.lane_topic_pub = rospy.Publisher("/lane_topic", String, queue_size=1)  

        # 메시지 객체 초기화
        self.ctrl_cmd_msg = Drive_command()

        # 변수 초기화
        self.roi_points = []  # ROI 포인트 리스트
        self.mode = ''  # 현재 모드
        self.flag = False
        self.yellow_pixels = 0  # 노란색 픽셀 개수
        self.speed = 0.3  # 기본 속도
        self.steer = 0.0  # 조향각
        self.crossing_completed = False
        self.rate = rospy.Rate(30)

        # 메인 루프
        while not rospy.is_shutdown():
            self.control_loop()
            self.rate.sleep()

    def control_loop(self):
        """
        주행 제어 루프
        """
        # 특정 모드에서는 동작하지 않음
        if self.mode in ['RABACON', 'SIGN', 'DYNAMIC', 'STATIC']:
            return

        # 노란색 픽셀 수와 ROI 포인트 기반 제어
        if self.yellow_pixels < 3000 and len(self.roi_points) > 0 and self.crossing_completed == True:
            if self.roi_points[0][1] < 0 and self.steer < 1000:  # ROI 첫 포인트의 y값이 0보다 작을 경우
                self.steer += 2
            elif self.roi_points[0][1] > 0 and self.steer > -1000:  # ROI 첫 포인트의 y값이 0보다 클 경우
                self.steer -= 2
            else:  # y값이 0일 경우
                self.steer = 0.0
            self.publishCtrlCmd(self.speed, self.steer, True)
        else:
            self.publishCtrlCmd(self.speed, self.steer, False)

    def crossing_gate_done_callback(self, msg):
        self.crossing_completed = msg.data

    def roiPointsCB(self, msg):
        """
        ROI 포인트 데이터 수신 콜백 함수
        """
        self.roi_points = []  
        for point in msg.points:
            self.roi_points.append((point.x, point.y))  # ROI 포인트 저장

    def yellowCB(self, msg):
        """
        노란색 픽셀 개수 수신 콜백 함수
        """
        self.yellow_pixels = msg.data

    def modeCB(self, msg):
        """
        모드 데이터 수신 콜백 함수
        """
        self.mode = msg.data  # /mode 토픽 데이터 저장

    def publishCtrlCmd(self, motor_msg, servo_msg, flag):
        """
        제어 명령 퍼블리시 함수
        """
        self.ctrl_cmd_msg.speed = round(motor_msg, 2)
        self.ctrl_cmd_msg.angle = round(servo_msg, 2)
        self.ctrl_cmd_msg.flag = flag
        self.ctrl_cmd_pub.publish(self.ctrl_cmd_msg)

if __name__ == '__main__':
    rospy.init_node('tunnel_drive', anonymous=True)
    try:
        tunnel_drive = TUNNEL()
    except rospy.ROSInterruptException:
        pass
