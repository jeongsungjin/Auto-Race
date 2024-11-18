#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from obstacle_detector.msg import Obstacles
from std_msgs.msg import String, Int32, Bool, Float64
from sensor_msgs.msg import PointCloud  
from lane_detection.msg import Drive_command
import math

class PARKING:
    def __init__(self):
        # Subscribers
        rospy.Subscriber("/mode", String, self.modeCB)  # /mode를 modeCB로 연결
        rospy.Subscriber("/roi_points_parking", PointCloud, self.roiPoints_parkingCB)  # /roi_points_tunnel을 roiPointsCB로 연결
        rospy.Subscriber("/roi_points_roundabout", PointCloud, self.roiPointsCB)  # /roi_points_tunnel을 roiPointsCB로 연결

        rospy.Subscriber("/is_blue", Bool, self.BlueCB)
        rospy.Subscriber("/tunnel_done", Bool, self.tunnel_done_callback) # 콜백 만들어서 주석 풀어야함!
    
        rospy.Subscriber("/motor_lane", Drive_command, self.ctrlLaneCB)  # 카메라 이미지 구독 (차선 인식용)
        rospy.Subscriber("/heading", Float64, self.headingCB)

        # Publishers
        self.ctrl_cmd_pub = rospy.Publisher('/motor_parking', Drive_command, queue_size=1)

        # 메시지 객체 초기화
        self.ctrl_cmd_msg = Drive_command()
        self.ctrl_lane = Drive_command()

        # 변수 초기화
        self.roi_points = []  # ROI 포인트 리스트 (넓음, 회전 교차로 관심영역임)
        self.parking_roi_points = []  # ROI 포인트 리스트 (주차공간 내부 초근접 관심영역 앞뒤 30, 좌우 20)
        self.x_points = []  
        self.y_points = []  

        self.mode = ''  # 현재 모드
        self.is_blue = False
        #주차 순서 플래그
        self.tunnel_done_flag = False #원래 false임!!!
        self.real_parking_start = False
        self.right_enter_done = False
        self.left_back_done = False
        self.right_back_done  = False
        self.parking_finish = False
        self.back_mid = False
        self.no_wall_cnt = 0
        self.left_back_cnt = 0
        self.back_mid_cnt = 0
        self.speed = 0.2  # 기본 속도
        self.steer = 0.0  # 조향각
        self.rate = rospy.Rate(30)

        # 메인 루프
        while not rospy.is_shutdown():
            if self.mode in ['RABACON', 'SIGN', 'ROUNDABOUT', 'STATIC']:
                continue

            # 1번 로직
            if self.is_blue == True and self.tunnel_done_flag == True:
                print("주차 시작! 리얼 파킹 트루!!")
                self.real_parking_start = True


            if self.real_parking_start == True and self.tunnel_done_flag == True:

                if len(self.roi_points) > 0:
                    self.publishCtrlCmd(self.ctrl_lane.speed, self.ctrl_lane.angle, True)
                    # print("라이다 인지 없을때까지 차선 보고 주행!!!!!!")

                elif len(self.roi_points) == 0 and self.no_wall_cnt < 40 and self.right_enter_done == False:                    
                    self.no_wall_cnt += 1
                    self.steer += 70  # 우회전 조향량
                    self.speed = 0.2
                    self.publishCtrlCmd(self.speed, self.steer, True)
                    print("우조향 량, 우조향 직진 카운트", self.speed, self.steer, self.no_wall_cnt)
                    if self.no_wall_cnt == 40:
                        self.right_enter_done = True
                        print("우조향 직진 끝!")
                
                if (self.right_enter_done == True) and (self.left_back_cnt < 70) and (self.left_back_done == False):                    
                    self.left_back_cnt += 1
                    self.left_steer = -300  # 좌측 조향량
                    self.left_steer -= 70
                    self.speed = -0.2  # 후진 속도
                    self.publishCtrlCmd(self.speed, self.left_steer, True)
                    print("좌측 후진 조향량, 좌측 후진 카운트", self.steer, self.left_back_cnt)
                    if self.left_back_cnt == 70:
                        self.left_back_done =True

                if self.left_back_done == True and self.right_back_done == False:
                    if self.back_mid == False and self.back_mid_cnt < 20:
                        self.back_mid_cnt += 1
                        self.publishCtrlCmd(-0.2, 0.0, True)
                        print("핸들풀고 후진 조향량, 핸들풀고 후진 카운트", 0.0, self.left_back_cnt)
                        if self.back_mid_cnt == 20:
                            self.back_mid = True
                    else:
                        if self.x_points and (self.x_points[0] > 0):
                            print("후진 하다 멈춤")
                            while(1):
                                self.publishCtrlCmd(0.0, 0.0, True)
                                self.right_back_done = True # 우조향 후진 끝
                                continue
                        else:
                            self.right_steer = 0
                            self.right_steer += 10
                            self.speed = -0.2
                            self.publishCtrlCmd(self.speed, self.steer, True)
                            print("우조향 후진")
                        
                # if self.right_back_done == True and self.parking_finish == False:

                #     if self.x_points[0] < 0:
                #         self.parking_finish = True # 쑤셔넣기 끝
                #         self.publishCtrlCmd(0.0, self.steer, True)
                #         continue

                #     self.left_steer = -100
                #     self.left_steer -= 10
                #     self.speed = 0.2
                #     self.publishCtrlCmd(self.speed, self.steer, True)

                            

            self.rate.sleep()


    def headingCB(self, msg):
        self.real_heading = msg.data
        if self.gt_heading != None:
            self.local_heading = self.real_heading - self.gt_heading
            if self.local_heading > 180:
                self.local_heading -= 360
            elif self.local_heading < -180:
                self.local_heading += 360
        else:
            self.local_heading = None

    def roiPointsCB(self, msg):
        self.roi_points = []  
        for point in msg.points:
            self.roi_points.append((point.x, point.y))  # ROI 포인트 저장

    def roiPoints_parkingCB(self, msg):
        self.parking_roi_points = []  
        self.x_points = []  
        self.y_points = []  

        for point in msg.points:
            self.parking_roi_points.append((point.x, point.y))  
            self.x_points.append(point.x)  
            self.y_points.append(point.y)  

    def tunnel_done_callback(self, msg):
        self.tunnel_done_flag = msg.data

    def BlueCB(self, msg):
        self.is_blue = msg.data

    def modeCB(self, msg):
        self.mode = msg.data  # /mode 토픽 데이터 저장

    def ctrlLaneCB(self, msg):
        self.ctrl_lane.speed = msg.speed
        self.ctrl_lane.angle = msg.angle
        self.lane_mode_flag = msg.flag

    def publishCtrlCmd(self, motor_msg, servo_msg, flag):

        self.ctrl_cmd_msg.speed = round(motor_msg, 2)
        self.ctrl_cmd_msg.angle = round(servo_msg, 2)
        self.ctrl_cmd_msg.flag = flag
        self.ctrl_cmd_pub.publish(self.ctrl_cmd_msg)

if __name__ == '__main__':
    rospy.init_node('tunnel_drive', anonymous=True)
    try:
        parking_drive = PARKING()
    except rospy.ROSInterruptException:
        pass
