#!/usr/bin/env python
#-*- coding: utf-8 -*-

from this import d
import rospy
from time import sleep, time
import statistics
from collections import deque

# Steering_angle --> 입력 가능한 범위가 -0.34 ~ +0.34 까지 입력 가능
# speed --> 실차량 기준은 -10m/s ~ 10m/s (권장 사항은 -2.5~ 2.5m/s 만 사용하는 것을 권장)


## HSV 50 50 30 그늘 있을 때
## battery 9.47

from ackermann_msgs.msg import AckermannDriveStamped
from std_msgs.msg import Int32, String, Float32
from sensor_msgs.msg import Image
from fiducial_msgs.msg import Fiducial, FiducialArray
from obstacle_detector.msg import Obstacles

from cv_bridge import CvBridge
import cv2
import numpy as np

from warper import Warper
from slidewindow import SlideWindow
from rabacon_drive import ClusterLidar
# from What_is_obstacle import DecideObstacle

class Controller():

    def __init__(self):
        # 객체 초기화
        self.warper = Warper()  # 이미지 왜곡을 보정하는 객체 (Perspective 변환 등)
        self.slidewindow = SlideWindow()  # 슬라이딩 윈도우 기법을 통해 차선 탐지하는 객체
        self.bridge = CvBridge()  # ROS 이미지 메시지를 OpenCV 이미지로 변환하는 브릿지
        self.rabacon_mission = ClusterLidar()  # LiDAR 기반 장애물 탐지 및 클러스터링 객체
        # self.decide_obstacle = DecideObstacle()  # 장애물 분류를 위한 객체 (사용 중지)

        # 초기 설정
        self.current_lane = "RIGHT"  # 차량이 처음 주행할 차선 설정 (우측 차선)
        self.is_safe = True  # 주행 안전 여부, 초기값은 안전하다고 가정
        self.initialized = False  # 차선 콜백 함수에서 초기화가 완료되었는지 여부
        self.speed_lane = 0.7  # 기본 주행 속도
        self.speed_rabacon = 0.5  # 특정 미션 (rabacon)에 사용되는 속도
        self.speed_turn = 0.3  # 차선 변경 시 사용되는 속도
        self.speed_slow = 0.3  # 서행이 필요한 구간에서의 속도 (예: 어린이 보호구역)
        self.speed_stop = 0.0  # 정지 속도

        # 슬라이드 윈도우의 반환값 변수 초기화
        self.slide_img = None  # 슬라이드 윈도우로 생성된 이미지
        self.slide_x_location = 0  # 슬라이드 윈도우에서 검출된 x 위치
        self.current_lane_window = ""  # 현재 주행 중인 차선을 슬라이드 윈도우가 반환한 결과값으로 저장

        # 특정 미션(rabacon)을 위한 변수
        self.rabacon_mission_flag = False  # rabacon 미션 활성화 여부

        # 서행 구역 (어린이 보호구역) 미션을 위한 변수
        self.slow_flag = 0  # 서행 플래그, 미션이 시작되었는지 여부
        self.slow_down_flag = 0  # 서행이 활성화되었는지 여부
        self.slow_t1 = 0.0  # 서행 시작 시간
        self.sign_data = 0  # 표지판 데이터 (어린이 보호구역 신호 등)
        self.child_cnt = 0  # 어린이 보호구역 진입 횟수 카운트

        # 정적 장애물 회피를 위한 변수
        self.static_flag = 0  # 정적 장애물 감지 플래그
        self.turn_left_flag = 0  # 왼쪽 차선 변경 플래그
        self.turn_right_flag = 0  # 오른쪽 차선 변경 플래그

        # 동적 장애물 회피를 위한 변수
        self.dynamic_flag = 0  # 동적 장애물 감지 플래그
        self.dynamic_flag2 = 0  # 동적 장애물의 보조 플래그 (타입 확인 등)

        # 장애물 종류 인식을 위한 변수
        #self.stop_cnt = 0  # 사용하지 않는 정지 카운트

        self.obstacle_kind = "None"  # 현재 감지된 장애물의 종류
        self.obstacle_img = []  # 장애물 이미지 데이터 저장

        # 장애물 거리 정보를 저장하는 리스트
        self.y_list = []  # 장애물의 y 위치 데이터 리스트
        self.y_list_sort = []  # y 위치 데이터를 정렬한 리스트
        self.dynamic_obs_cnt = 0  # 동적 장애물 감지 횟수 카운트
        self.static_cnt = 0  # 정적 장애물 감지 횟수 카운트

        # ROS 타이머 및 퍼블리셔, 서브스크라이버 설정
        rospy.Timer(rospy.Duration(1.0/30.0), self.timer_callback)  # 30Hz 주기로 콜백 호출
        self.drive_pub = rospy.Publisher("high_level/ackermann_cmd_mux/input/nav_0", AckermannDriveStamped, queue_size=1)  # 차량 주행 명령 퍼블리셔

        # 이미지 처리 및 장애물 관련 데이터 구독
        rospy.Subscriber("usb_cam/image_rect_color", Image, self.lane_callback)  # 카메라 이미지 구독 (차선 인식용)
        # rospy.Subscriber("obstacle_mission", String, self.warning_callback)  # 사용되지 않는 장애물 미션 구독
        rospy.Subscriber("lidar_warning", String, self.warning_callback)  # LiDAR 기반 객체 탐지 경고 신호 구독 (안전/경고)
        rospy.Subscriber("object_condition", Float32, self.object_callback)  # 장애물 상태 데이터 구독
       
        rospy.Subscriber("sign_id", Int32, self.child_sign_callback)  # 표지판 ID (어린이 보호구역 신호 등) 구독
        # rospy.Subscriber("lidar_warning", String, self.warning_callback)  # 사용되지 않는 LiDAR 경고 신호 구독
        rospy.Subscriber("rabacon_drive", Float32, self.rabacon_callback)  # rabacon 미션용 주행 데이터 구독
        rospy.Subscriber("obstacles", Obstacles, self.rabacon_callback)  # 장애물 데이터 구독
     

    def object_callback(self, _data):
        # self.dynamic_flag2 = _data.data
        if self.dynamic_flag != 1 or len(self.y_list) <= 19:
            self.y_list.append(_data.data)
            if len(self.y_list) >= 21 :
                del self.y_list[0]
        # rospy.loginfo("data:{}, last:{}, length:{}".format(_data.data, self.y_list[-1], len(self.y_list)))
       
        # if _data.data == "STATIC":
        #     rospy.logwarn("STATIC DeteCTED!!!!! TURN ")
        #     self.static_flag = 1
        # elif _data.data == "DYNAMIC":
        #     self.dynamic_flag = 1
        #     self.y_list = []
            #rospy.loginfo("DYNAMIC DETECTED, Pleas Wait !!")
        else:
            rospy.logwarn("Unknown warning state!")
            # rospy.logwarn("Data is : {}".format(_data.data))

    def warning_callback(self, _data):

        if self.rabacon_mission_flag == True :
            self.rabacon_mission_flag = True
        elif _data.data == "safe":
            self.is_safe = True
            #self.stop_cnt = 0
            self.y_list = []
            if self.dynamic_flag == 1 :
                self.dynamic_obs_cnt += 1
                if self.dynamic_obs_cnt >= 50 : # 50
                    self.dynamic_flag = 0
                    self.dynamic_obs_cnt = 0
            self.static_flag = 0
            # self.dynamic_flag = 0
        elif _data.data == "WARNING":
            self.is_safe = False
            rospy.loginfo("WARNING!")
        else:
            pass
            # rospy.logwarn("Unkown warning state!!")
            # rospy.logwarn("Data is : {}".format(_data.data))
       

    def rabacon_callback(self, _data) :

        self.rabacon_mission_flag = _data.data
        if self.rabacon_mission_flag < 10.0 :
            self.rabacon_mission = 1
            # self.y_list = []
            # self.y_list_sort = []
        else :
            self.rabacon_mission = 0



    def lane_callback(self, _data):  
         
# ========  preprocessing via gray ==============================

        # cv_image = self.bridge.imgmsg_to_cv2(_data, "bgr8")

        # if self.initialized == False:
        #     cv2.namedWindow("lane_image", cv2.WINDOW_NORMAL)
        #     cv2.createTrackbar('gray', 'lane_image', 0, 255, nothing)
        #     self.initialized = True

        # gray = cv2.getTrackbarPos('gray', 'lane_image')

        # lane_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        # lane_image2 = cv2.inRange(lane_image, gray, 255)

        # cv2.imshow("lane_image", lane_image2)

        # self.lane_detection(lane_image2)

        # cv2.waitKey(1)

# ========== preprocessing via HSV ==========================

        if self.initialized == False:
            cv2.namedWindow("Simulator_Image", cv2.WINDOW_NORMAL)
            cv2.createTrackbar('low_H', 'Simulator_Image', 50, 255, nothing)
            cv2.createTrackbar('low_S', 'Simulator_Image', 50, 255, nothing)
            cv2.createTrackbar('low_V', 'Simulator_Image', 50, 255, nothing)
            cv2.createTrackbar('high_H', 'Simulator_Image', 255, 255, nothing)
            cv2.createTrackbar('high_S', 'Simulator_Image', 255, 255, nothing)
            cv2.createTrackbar('high_V', 'Simulator_Image', 255, 255, nothing)
            self.initialized = True

        cv2_image = self.bridge.imgmsg_to_cv2(_data)
        self.obstacle_img = cv2_image


        cv2.imshow("original", cv2_image)

        low_H = cv2.getTrackbarPos('low_H', 'Simulator_Image')
        low_S = cv2.getTrackbarPos('low_S', 'Simulator_Image')
        low_V = cv2.getTrackbarPos('low_V', 'Simulator_Image')
        high_H = cv2.getTrackbarPos('high_H', 'Simulator_Image')
        high_S = cv2.getTrackbarPos('high_S', 'Simulator_Image')
        high_V = cv2.getTrackbarPos('high_V', 'Simulator_Image')


        cv2.cvtColor(cv2_image, cv2.COLOR_BGR2HSV) # BGR to HSV

        lower_lane = np.array([low_H, low_S, low_V]) #
        upper_lane = np.array([high_H, high_S, high_V])

        lane_image = cv2.inRange(cv2_image, lower_lane, upper_lane)

        cv2.imshow("Lane Image", lane_image)
        self.lane_detection(lane_image)

        cv2.waitKey(1)


    def timer_callback(self, _event):
        try :
            self.follow_lane()
        except :
            pass


 
    def lane_detection(self, lane_image) :
        kernel_size = 5
        blur_img = cv2.GaussianBlur(lane_image,(kernel_size, kernel_size), 0)
        warped_img = self.warper.warp(blur_img)
        cv2.imshow("warped_img", warped_img)
        self.slide_img, self.slide_x_location, self.current_lane_window = self.slidewindow.slidewindow(warped_img)
        cv2.imshow("slide_img", self.slide_img)
        # rospy.loginfo("CURRENT LANE WINDOW: {}".format(self.current_lane_window))


    def follow_lane(self): # 차선데이터를 받아서 차선을 따라서 주행하도록 하는 함수
               
        # slow_down_sign
        if self.slow_down_flag == 1:  # 차량이 감속해야 하는 조건을 확인하는 플래그가 활성화되었는지 확인
                
            # rospy.loginfo("sign data :{}".format(self.sign_data))
            if self.sign_data == 3:  # sign_data가 3인 경우, 차량이 대기해야 할 상황을 의미
                rospy.loginfo(" ===============   SLOW DETECTED, WAIT!!!! ============")
                self.error_lane = 280 - self.slide_x_location  # 차량의 차선 위치 오차(error)를 계산 (280은 기준 위치)
                                                            # error가 음수면 오른쪽 차선에 가까움, 양수면 왼쪽 차선에 가까움을 의미
                publishing_data = AckermannDriveStamped()  # AckermannDriveStamped 메시지 객체 생성
                publishing_data.header.stamp = rospy.Time.now()  # 메시지에 현재 시간을 기록
                publishing_data.header.frame_id = "base_link"  # 메시지의 참조 프레임을 "base_link"로 설정
                publishing_data.drive.steering_angle = self.error_lane * 0.003  # 차선 위치 오차에 따라 조향 각도 결정
                publishing_data.drive.speed = self.speed_lane  # 차선 주행 속도(speed_lane)를 설정
                self.drive_pub.publish(publishing_data)  # 설정한 속도와 조향 각도 메시지를 퍼블리시하여 차량에 적용

            elif self.sign_data == 0:  # sign_data가 0인 경우, 차량이 천천히 감속해야 할 상황을 의미
                rospy.loginfo("************* SLOW DOWN *****************")        
                self.child_cnt = 0  # 감속 시 특정 동작이 누적되지 않도록 카운트를 초기화
                if self.slow_flag == 0:  # 감속 플래그(slow_flag)가 0일 경우 (이전 감속이 아닌 경우)
                    self.slow_t1 = rospy.get_time()  # 감속 시작 시간을 기록
                    self.slow_flag = 1  # 감속 플래그를 활성화하여 감속 중임을 표시
                t2 = rospy.get_time()  # 현재 시간을 t2에 저장 (시간 경과 확인용)
                # rospy.loginfo("t1 :{}, t2 : {}".format(self.slow_t1, t2))
                # 차량이 15초 동안 감속 상태를 유지해야 함
                while t2 - self.slow_t1 <= 15:  # 현재 시간(t2)과 감속 시작 시간(slow_t1) 차이가 15초 이하일 동안 반복
                    rospy.loginfo("************* SLOW DOWN *****************")
                    # rospy.loginfo("slow_down time{}, {}".format(self.slow_t1, t2))
                    self.error_lane = 280 - self.slide_x_location  # 차량의 차선 위치 오차를 다시 계산
                    publishing_data = AckermannDriveStamped()  # 새로운 AckermannDriveStamped 메시지 생성
                    publishing_data.header.stamp = rospy.Time.now()  # 현재 시간을 메시지에 기록
                    publishing_data.header.frame_id = "base_link"  # 참조 프레임을 "base_link"로 설정
                    publishing_data.drive.steering_angle = self.error_lane * 0.003  # 차선 위치 오차에 따라 조향 각도 설정
                    publishing_data.drive.speed = self.speed_slow  # 속도를 느리게 설정(speed_slow)
                    self.drive_pub.publish(publishing_data)  # 퍼블리시하여 차량이 속도와 조향에 따라 주행하도록 함
                    t2 = rospy.get_time()  # 현재 시간을 다시 갱신하여 루프 종료 조건을 확인
                self.slow_down_flag = 0  # 감속 완료 후 slow_down_flag를 비활성화하여 초기화
                self.slow_flag = 0  # slow_flag도 초기화하여 이후 감속 조건에서 다시 사용 가능하게 함
                # self.y_list = []  # y_list라는 데이터 목록을 초기화 (사용하지 않는 경우로 보이며 주석 처리됨)

       
        # rabacon
        elif self.rabacon_mission == 1:  # rabacon 미션이 활성화된 경우
            self.rabacon_drive()  # rabacon 미션을 위한 주행 함수 호출
            ## RABACON_OUT_LANE : COMPETITION DAY
            self.current_lane = "RIGHT"  # 현재 차선을 "RIGHT"로 설정 (이후에 차선 선택 관련 논리를 적용 가능)
            # self.y_list = []  # y_list 초기화 (주석 처리됨)
            # self.current_lane = "LEFT"  # 주석 처리된 다른 차선 방향 설정 옵션 (LEFT로 설정 가능)

        # obstacle
        elif self.is_safe == False:  # 주행이 안전하지 않은 상황 (장애물 발견 시)
            self.y_list_sort = sorted(self.y_list, key=lambda x: x)  # y_list를 오름차순으로 정렬하여 y_list_sort에 저장
            rospy.loginfo("Y_LIST{}".format(self.y_list))  # y_list의 현재 상태를 로그로 출력
            
            # 장애물의 위치 정보를 기반으로 차량의 안전 상태를 분석
            if len(self.y_list) <= 19:  # y_list 길이가 19 이하이면 장애물이 가까이 있음 (안전하지 않은 상태)
                self.stop()  # 차량 정지 함수 호출
                rospy.loginfo("obstacle_stop, dynamic_flag = {}", format(self.dynamic_flag))  # 장애물로 인해 정지된 상태 로그 출력

            elif abs(statistics.mean(self.y_list_sort[0:1]) - statistics.mean(self.y_list_sort[-2:-1])) >= 0.17 or self.y_list_sort[10] < -0.15:
                # 정렬된 y_list_sort에서 양 끝 부분 평균 차이가 0.17 이상이거나 10번째 요소가 -0.15보다 작으면 동적 장애물로 판단
                self.dynamic_flag = 1  # 동적 장애물 플래그 활성화
                self.static_flag = 0  # 정적 장애물 플래그 비활성화
                # self.y_list.clear()  # y_list를 초기화 (주석 처리됨)
                rospy.loginfo("dynamic")  # 동적 장애물로 판단된 상태 로그 출력
                rospy.loginfo(abs(statistics.mean(self.y_list_sort[0:1]) - statistics.mean(self.y_list_sort[-2:-1])))  # 평균 차이 로그 출력
            else:
                # 동적 장애물 조건을 만족하지 않을 경우, 정적 장애물로 판단
                self.static_cnt += 1  # 정적 장애물 카운트 증가
                if self.static_cnt >= 10:  # 정적 장애물 조건을 10번 이상 만족한 경우
                    self.static_flag = 1  # 정적 장애물 플래그 활성화
                    self.dynamic_flag = 0  # 동적 장애물 플래그 비활성화
                    self.static_cnt = 0  # 정적 카운트 초기화
                rospy.loginfo("static")  # 정적 장애물로 판단된 상태 로그 출력
                rospy.loginfo(abs(statistics.mean(self.y_list_sort[0:1]) - statistics.mean(self.y_list_sort[-2:-1])))  # 평균 차이 로그 출력

        # dynamic
        if self.dynamic_flag == 1 and self.is_safe == False:  # 동적 장애물이 감지되었고 안전하지 않은 상태일 때
            rospy.logwarn("DYNAMIC OBSTACLE")  # 동적 장애물이 감지되었음을 경고 메시지로 출력
            self.stop()  # 차량 정지 함수 호출

        # static obstacle
        elif self.static_flag == 1:  # 정적 장애물이 감지되었을 때
            rospy.loginfo("STATIC OBSTACLE")  # 정적 장애물이 감지되었음을 정보 메시지로 출력

            # 현재 차선이 "RIGHT"일 때 (우측 차선 주행 중일 때)
            if self.current_lane == "RIGHT":
                rospy.logwarn("IN RIGHT")  # 현재 우측 차선 주행 중임을 경고 메시지로 출력
                if self.turn_left_flag == 0:  # 왼쪽으로의 차선 변경 시작 시간을 초기화하지 않았을 때
                    self.turn_left_t1 = rospy.get_time()  # 현재 시간을 차선 변경 시작 시간으로 저장
                    self.turn_left_flag = 1  # 차선 변경 시작 플래그 설정
                t2 = rospy.get_time()  # 현재 시간 갱신

                # 왼쪽으로 차선 변경 (1초 동안 수행)
                while t2 - self.turn_left_t1 <= 1.0:
                    self.change_line_left()  # 왼쪽 차선 변경 함수 호출
                    t2 = rospy.get_time()  # 시간 갱신

                # 오른쪽으로 되돌아가는 차선 변경 (1.25초 동안 수행)
                while t2 - self.turn_left_t1 <= 1.25:
                    self.change_line_right()  # 오른쪽 차선 변경 함수 호출
                    t2 = rospy.get_time()  # 시간 갱신

                # 차선 변경 후 현재 차선을 "LEFT"로 설정
                self.current_lane = "LEFT"
                self.static_flag = 0  # 정적 장애물 플래그 초기화
                self.turn_left_flag = 0  # 왼쪽 차선 변경 플래그 초기화

            # 현재 차선이 "LEFT"일 때 (좌측 차선 주행 중일 때)
            elif self.current_lane == "LEFT":
                rospy.logwarn("IN LEFT")  # 현재 좌측 차선 주행 중임을 경고 메시지로 출력
                if self.turn_right_flag == 0:  # 오른쪽으로의 차선 변경 시작 시간을 초기화하지 않았을 때
                    self.turn_right_t1 = rospy.get_time()  # 현재 시간을 차선 변경 시작 시간으로 저장
                    self.turn_right_flag = 1  # 차선 변경 시작 플래그 설정
                t2 = rospy.get_time()  # 현재 시간 갱신

                # 오른쪽으로 차선 변경 (1초 동안 수행)
                while t2 - self.turn_right_t1 <= 1.0:
                    self.change_line_right()  # 오른쪽 차선 변경 함수 호출
                    t2 = rospy.get_time()  # 시간 갱신

                # 왼쪽으로 되돌아가는 차선 변경 (1.25초 동안 수행)
                while t2 - self.turn_right_t1 <= 1.25:
                    self.change_line_left()  # 왼쪽 차선 변경 함수 호출
                    t2 = rospy.get_time()  # 시간 갱신

                # 차선 변경 후 현재 차선을 "RIGHT"로 설정
                self.current_lane = "RIGHT"
                self.static_flag = 0  # 정적 장애물 플래그 초기화
                self.turn_right_flag = 0  # 오른쪽 차선 변경 플래그 초기화
        
           
           
           
        # no obstalce, no rabacon, no slow sign, just drive
        else:
            rospy.loginfo("DEFAULT DRIVE!!!!!!!!!")
            self.error_lane = 280 - self.slide_x_location # error가 음수 --> 오른쪽 차선이랑 멈 / error가 양수 --> 오른쪽 차선이랑 가까움
            publishing_data = AckermannDriveStamped()
            publishing_data.header.stamp = rospy.Time.now() # 이 데이터를 보낼 때의 시점
            publishing_data.header.frame_id = "base_link"
            publishing_data.drive.steering_angle = self.error_lane * 0.003 # error 정도에 따라서 조향
            publishing_data.drive.speed = self.speed_lane
            self.drive_pub.publish(publishing_data) #  하는 부분
            # if self.current_lane_window == "MID" :
            #     publishing_data = AckermannDriveStamped()
            #     publishing_data.header.stamp = rospy.Time.now() # 이 데이터를 보낼 때의 시점
            #     publishing_data.header.frame_id = "base_link"
            #     publishing_data.drive.steering_angle = self.error_lane * 0.003 # error 정도에 따라서 조향
            #     publishing_data.drive.speed = self.speed_lane
            #     self.drive_pub.publish(publishing_data) #  하는 부분


    # if the car find any obstacle, the car should stop.
    def stop(self):
        publishing_data = AckermannDriveStamped()
        publishing_data.header.stamp = rospy.Time.now() # 이 데이터를 보낼 때의 시점
        publishing_data.header.frame_id = "base_link"
        publishing_data.drive.steering_angle = 0.0
        publishing_data.drive.speed = self.speed_stop
        self.drive_pub.publish(publishing_data) # 실제 publish 하는 부분
        #rospy.loginfo("Stop Vehicle!")
        #self.stop_cnt += 1
        #print("STOP_CNT{}".format(self.stop_cnt))
       


    ####################################################################################
    # protect child
    def child_sign_callback(self, _data):
        try :
            # : {}".format(_data.data))

            if _data.data == 3:
                self.child_cnt += 1
                if self.child_cnt >=20 :
                    self.sign_data = _data.data
                    self.slow_down_flag = 1
                    self.child_cnt = 0
            else :
                self.sign_data = 0
                # self.slow_down_flag = 0
            #rospy.loginfo(" sign data_callback  : {}".format(self.sign_data))

            #if _data.data == 3:
            #    self.slow_down_flag = 1
               
        except :
            pass
       
    ####################################################################################
    # dynamic obstacle

    ####################################################################################
    # static obstacle
    def change_line_left(self) :
       
        #rospy.loginfo("change_left!!!!!!!!!!!!!!!!!")
        publishing_data = AckermannDriveStamped()
        publishing_data.header.stamp = rospy.Time.now() # 이 데이터를 보낼 때의 시점
        publishing_data.header.frame_id = "base_link"
        publishing_data.drive.speed = self.speed_turn
        publishing_data.drive.steering_angle = 0.3
        self.drive_pub.publish(publishing_data)

    def change_line_right(self) :
       
        #rospy.loginfo("change_right!!!!!!!!!!!!!!!!!")
        publishing_data = AckermannDriveStamped()
        #left_time = time()
        publishing_data.header.stamp = rospy.Time.now() # 이 데이터를 보낼 때의 시점
        publishing_data.header.frame_id = "base_link"
        publishing_data.drive.speed = self.speed_turn
        publishing_data.drive.steering_angle = -0.3
        self.drive_pub.publish(publishing_data)
       
    ####################################################################################
    # Rabacon Mission
    def rabacon_drive(self) :

        rospy.loginfo("rabacon_drive!!!!!!!!!!!!!!!!!")
        publishing_data = AckermannDriveStamped()
        publishing_data.header.stamp = rospy.Time.now() # 이 데이터를 보낼 때의 시점
        publishing_data.header.frame_id = "base_link"
        publishing_data.drive.speed = self.speed_rabacon
        publishing_data.drive.steering_angle = self.rabacon_mission_flag * -1.0
        #print(self.rabacon_mission_flag * -1)
        self.drive_pub.publish(publishing_data)
       

    ####################################################################################


def nothing(x):
    pass


def run():

    rospy.init_node("main_class_run")
    control = Controller()
    rospy.Timer(rospy.Duration(1.0/30.0), control.timer_callback)
    rospy.spin()

if __name__ == "__main__":
    run()
