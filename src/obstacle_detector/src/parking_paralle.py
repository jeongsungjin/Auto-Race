#!/usr/bin/env python
# -*- coding: utf-8 -*-


import rospy
import math
import numpy as np
from obstacle_detector.msg import Obstacles
from std_msgs.msg import Float64, String
from obstacle_detector.msg import Drive_command
from ackermann_msgs.msg import AckermannDriveStamped
from sensor_msgs.msg import LaserScan

class Obstacle:
    def __init__(self, x=None, y=None, distance=None):
        self.x = x
        self.y = y
        self.distance = distance

class ParalleParking:
    def __init__(self):
        rospy.init_node("parking_paralle", anonymous=True)

        # 구독 및 퍼블리셔 설정
        rospy.Subscriber("/raw_obstacles", Obstacles, self.update_objects)
        self.ctrl_cmd_pub = rospy.Publisher('/motor_parking', Drive_command, queue_size=1)

        self.ctrl_cmd_msg = Drive_command()

        # 주차 관련 변수
        self.objects = []  # 장애물 리스트
        self.left_wall = []  # 왼쪽 벽 점들
        self.rear_wall = []  # 후면 벽 점들
        self.parking_center = None  # 주차 공간 중심
        self.path_points = []  # 생성된 경로
        self.state = "SEARCHING"  # 상태 머신: SEARCHING, FORWARD, BACKWARD, COMPLETE
        self.motor_speed = 0.3  # 기본 주행 속도
        self.lookahead_distance = 0.2  # Pure Pursuit의 목표 점 거리
        self.go_count = 0

        rospy.loginfo("Parking Assistant Initialized.")

        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            self.execute_state_machine()
            rate.sleep()

    def update_objects(self, data):
        """장애물 데이터를 갱신"""
        self.objects = [
            Obstacle(
                x=circle.center.x,
                y=circle.center.y,
                distance=self.get_distance(Obstacle(0, 0), Obstacle(circle.center.x, circle.center.y))
            )
            for circle in data.circles
        ]
        self.objects.sort(key=lambda obj: obj.distance)  # 거리 기준으로 정렬
        self.detect_parking_space()

    def get_distance(self, obj1, obj2):
        """두 Obstacle 객체 간 거리 계산"""
        return math.sqrt((obj1.x - obj2.x) ** 2 + (obj1.y - obj2.y) ** 2)

    def detect_parking_space(self):
        """주행 중 왼쪽에 있는 ㄷ자 벽 탐지"""
        self.left_wall = []
        self.rear_wall = []

        for obj in self.objects:
            if obj.y > 0.5:  # 왼쪽 벽 (y > 0.5m)
                self.left_wall.append(obj)
            elif 0.4 <= obj.x <= 0.6:  # 후면 벽 (x 범위)
                self.rear_wall.append(obj)

        if len(self.left_wall) > 3 and len(self.rear_wall) > 1:
            rospy.loginfo("Parking space detected.")
            self.calculate_parking_center()

    def calculate_parking_center(self):
        """주차 공간 중심 계산"""
        left_wall_y = [obj.y for obj in self.left_wall]
        rear_wall_x = [obj.x for obj in self.rear_wall]

        # 주차 공간 중심 계산
        self.parking_center = Obstacle(
            x=(max(rear_wall_x) + min(rear_wall_x)) / 2,
            y=(max(left_wall_y) + min(left_wall_y)) / 2,
        )
        rospy.loginfo(f"Parking center: X={self.parking_center.x:.2f}, Y={self.parking_center.y:.2f}")
        self.state = "FORWARD"

    def generate_bezier_path(self, start, control, end):
        """Bezier Curve를 사용하여 경로 생성"""
        t_values = np.linspace(0, 1, 50)
        path = []
        for t in t_values:
            x = (1 - t)**2 * start[0] + 2 * (1 - t) * t * control[0] + t**2 * end[0]
            y = (1 - t)**2 * start[1] + 2 * (1 - t) * t * control[1] + t**2 * end[1]
            path.append(Obstacle(x=x, y=y))
        return path

    def follow_path(self):
        """Pure Pursuit 경로 추적"""
        if not self.path_points:
            rospy.logwarn("No path points to follow.")
            return

        for point in self.path_points:
            target_x = point.x
            target_y = point.y
            distance = self.get_distance(Obstacle(0, 0), point)

            if distance < self.lookahead_distance:
                continue

            angle = math.atan2(target_y, target_x)
            self.publish_ctrl_cmd(-0.2, math.degrees(angle))
            rospy.sleep(0.1)

        rospy.loginfo("Path following complete.")
        self.publish_ctrl_cmd(0.0, 0.0)  # 정지

    def execute_state_machine(self):
        """상태 머신 실행"""
        if self.state == "SEARCHING":
            rospy.loginfo("Searching for parking space...")
            self.publish_ctrl_cmd(self.motor_speed, 0.0)  # 직진

        elif self.state == "FORWARD":
            rospy.loginfo("Moving forward to prepare for parking...")
            if self.go_count < 30 :
                self.publish_ctrl_cmd(self.motor_speed, 0.0) 
                self.go_count +=1 # 전진
            else :
                self.state = "BACKWARD"

        elif self.state == "BACKWARD":
            rospy.loginfo("Reversing into parking space...")
            self.reverse_into_parking_space()

        elif self.state == "COMPLETE":
            rospy.loginfo("Parking complete.")
            self.publish_ctrl_cmd(0.0, 0.0)  # 정지s

    def reverse_into_parking_space(self):
        """주차 공간으로 후진"""
        if self.parking_center:
            rospy.loginfo(f"Reversing towards X={self.parking_center.x:.2f}, Y={self.parking_center.y:.2f}")
            start = (0, 0)
            control = (-0.2, self.parking_center.y / 2)
            end = (self.parking_center.x, self.parking_center.y)

            self.path_points = self.generate_bezier_path(start, control, end)
            self.follow_path()
            self.state = "COMPLETE"

    def publish_ctrl_cmd(self, motor_msg, servo_msg):
        """제어 명령 퍼블리시"""
        self.ctrl_cmd_msg.speed = motor_msg
        self.ctrl_cmd_msg.angle = servo_msg
        self.ctrl_cmd_pub.publish(self.ctrl_cmd_msg)


if __name__ == "__main__":
    try:
        ParalleParking()
    except rospy.ROSInterruptException:
        pass
        
        
        