#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from obstacle_detector.msg import Obstacles
from std_msgs.msg import String, Int32, Bool
from sensor_msgs.msg import PointCloud
from lane_detection.msg import Drive_command

class TUNNEL:
    def __init__(self):
        # Subscribers
        rospy.Subscriber("/mode", String, self.modeCB)
        rospy.Subscriber("/roi_points_tunnel", PointCloud, self.roiPointsCB)
        rospy.Subscriber("/yellow_pixel", Int32, self.yellowCB)
        rospy.Subscriber("/crossing_gate_done", Bool, self.crossing_gate_done_callback)

        # Publisher
        self.ctrl_cmd_pub = rospy.Publisher('/motor_tunnel', Drive_command, queue_size=1)
        self.tunnel_done_pub = rospy.Publisher('/tunnel_done', Bool, queue_size=1)

        # 변수 초기화
        self.roi_points = []
        self.x_points = []
        self.y_points = []
        self.mode = ''
        self.yellow_pixels = 0
        self.crossing_completed = False
        self.tunnel_done_flag = False
        self.tunnel_in = False
        self.speed = 0.3
        self.steer = 0.0
        self.max_steer = 100.0  # 조향각 제한
        self.steer_decay = 0.9  # 감쇠 비율
        self.rate = rospy.Rate(30)

        # 메인 루프
        while not rospy.is_shutdown():
            self.control_loop()
            self.rate.sleep()

    def control_loop(self):
        # 특정 모드에서는 조향 제어를 하지 않음
        if self.mode in ['RABACON', 'SIGN', 'DYNAMIC', 'STATIC']:
            return
        print("터널이 받는 스태틱 플래그", self.crossing_completed)
        # 노란색 픽셀 및 게이트 상태 확인
        if self.yellow_pixels < 1000 and self.crossing_completed == True and self.tunnel_done_flag == False:
            self.tunnel_in = True
            # ROI 첫 번째 포인트 기준으로 조향 결정
            if self.y_points:
                if self.y_points[0] < 0:  # 왼쪽 벽에 가까운 경우
                    self.steer += 50
                elif self.y_points[0] > 0:  # 오른쪽 벽에 가까운 경우
                    self.steer -= 50
            else:  # 중앙에 가까운 경우
                self.steer = 0.0  # 감쇠 적용
            # 조향값 제한
            self.steer = max(min(self.steer, self.max_steer), -self.max_steer)
            # 제어 명령 퍼블리시
            self.publishCtrlCmd(self.speed, self.steer, True)

        else:
            # 조건을 충족하지 못할 경우 제어 비활성화
            self.publishCtrlCmd(self.speed, 0.0, False)

        if self.yellow_pixels >= 3500 and self.tunnel_in == True:
            self.tunnel_done_flag = True
            self.publish_tunnel_done(self.tunnel_done_flag)
            self.publishCtrlCmd(self.speed, self.steer, False)



    def crossing_gate_done_callback(self, msg):
        self.crossing_completed = msg.data

    def publish_tunnel_done(self, tunnel_done):
        self.tunnel_done_pub.publish(tunnel_done)

    def roiPointsCB(self, msg):
        self.roi_points = []
        self.x_points = []
        self.y_points = []

        for point in msg.points:
            self.roi_points.append((point.x, point.y))
            self.x_points.append(point.x)
            self.y_points.append(point.y)

    def yellowCB(self, msg):
        self.yellow_pixels = msg.data

    def modeCB(self, msg):
        self.mode = msg.data

    def publishCtrlCmd(self, motor_msg, servo_msg, flag):
        self.ctrl_cmd_msg = Drive_command()
        self.ctrl_cmd_msg.speed = motor_msg
        self.ctrl_cmd_msg.angle = servo_msg
        self.ctrl_cmd_msg.flag = flag
        self.ctrl_cmd_pub.publish(self.ctrl_cmd_msg)

if __name__ == '__main__':
    rospy.init_node('tunnel_drive', anonymous=True)
    try:
        tunnel_drive = TUNNEL()
    except rospy.ROSInterruptException:
        pass
