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


class StaticAvoidance():
    def __init__(self):
        rospy.Subscriber("/raw_obstacles", Obstacles, self.obstacleCB)
        rospy.Subscriber("/heading", Float64, self.headingCB)
        rospy.Subscriber("/mode", String, self.modeCB)


        self.ctrl_cmd_pub = rospy.Publisher('/motor_static', Drive_command, queue_size=1)
        self.ctrl_cmd_msg = Drive_command()

        # self.state
        # L: Lane
        # A: Avoid
        # R: Return
        self.state = 'L'

        self.obstacles = []
        self.is_static = False
        self.steer = 0
        self.speed = 0
        self.is_left = False

        self.closest_obstacle = Obstacle()

        # self.distance_threshold = 0.45
        self.distance_threshold = 1.2       # 정적 회피 시작거리 
        self.distance_threshold_max = 1.25   # 정적 회피중 다른 장애물과의 구분을 위한 임계점
        self.margin_y = 0.4

        self.angle = 0

        self.static_obstacle_cnt = 0
        self.static_obstacle_cnt_threshold = 15
        self.frames_without_obstacle = 0
        self.allowed_frames_without_obstacle = 5

        self.real_heading = None
        self.gt_heading = None

        self.avoid_heading = None
        self.return_heading = None

        self.local_heading = None

        self.last_n_obstacles_y = []
        self.len_last_n_obstacles_y = 5
        # self.avg_y = None

        self.mode = ''

        self.version = rospy.get_param('~version', 'safe')
        self.direction = rospy.get_param('~direction', 'left')

        if self.version == 'fast':
            self.speed = 0.7 # 일단
        else:
            self.speed = 0.7

        rospy.loginfo(f"STATIC: {self.version}")

        

        rate = rospy.Rate(30)
        while not rospy.is_shutdown():

            if self.mode == 'RABACON' or self.mode == 'SIGN':
                continue

            if len(self.obstacles) > 0:
                # 특정 roi에 인지가 들어오면 일단 감속
                for obstacle in self.obstacles:
                    
                    if (-1.2 < obstacle.x < 0) and (-0.25 <= obstacle.y <= 0.25):
                        self.static_obstacle_cnt += 1
                        self.update_last_n_obstacles_y(obstacle.y, self.len_last_n_obstacles_y)
                        #print(f"거리: {self.obstacles[0].distance:.4f}")
                        self.frames_without_obstacle = 0
                        break
                    else:
                        self.frames_without_obstacle += 1
                        if self.frames_without_obstacle > self.allowed_frames_without_obstacle:
                            # print('장애물이 없어져 카운트 감소 중...111')
                            self.static_obstacle_cnt -= 1
                        #self.static_obstacle_cnt -= 1
            else:  
                self.frames_without_obstacle += 1
                if self.frames_without_obstacle > self.allowed_frames_without_obstacle:
                    # print('장애물이 없어져 카운트 감소 중...222')
                    self.static_obstacle_cnt -= 1
                #self.static_obstacle_cnt -= 1

            # 완전히 차선 폭에 들어오는 장애물이 몇번 연속으로 찍힌다? cnt++ -> 특정 숫자 이상이면 그러면 아예 정적 모드
            # 하지만 계속 안들어오면 비례하게 cnt--

            if self.static_obstacle_cnt < 0:
                self.static_obstacle_cnt = 0
            elif self.static_obstacle_cnt > self.static_obstacle_cnt_threshold:
                self.static_obstacle_cnt = self.static_obstacle_cnt_threshold


            if self.state == 'L':
                if self.static_obstacle_cnt == self.static_obstacle_cnt_threshold:

                    # self.avg_y = sum(self.last_n_obstacles_y) / len(self.last_n_obstacles_y)
                    # 가운데 : 0.039
                    # 왼쪽   : 0.149, 0.147

                    # gt heading
                    self.gt_heading = self.real_heading

                    # 장애물의 위치가 중간 or 오른쪽 -> 왼쪽으로 회피
                    if self.direction == 'left': # -> 장애물의 위치 기반 방향 선택
                        self.avoid_heading = 30
                        self.return_heading = -20

                    # 장애물의 위치가 왼쪽 -> 오른쪽으로 회피 
                    else:
                        # 잘 되지만 크게 피함
                        # self.avoid_heading = -27.5
                        # self.return_heading = 20

                        # --------------튜닝 해야 하는 값 ------------------------#
                        if self.version == 'fast':
                            self.avoid_heading = -25
                            self.return_heading = 10
                        else:
                            self.avoid_heading = -25
                            self.return_heading = 10

                        #------------------------------------------------------#
                    # flag
                    self.state = 'A'
                    self.is_static = True

            elif self.state == 'A':
                if self.local_heading != None:

                    # 장애물의 위치가 중간 or 오른쪽 -> 왼쪽으로 회피
                    if self.direction == 'left': # -> 장애물의 위치 기반 방향 선택
                        if (self.avoid_heading > self.local_heading): # 목표 heading에 도달하지 못했으면 좌조향
                            self.angle = -1 * abs(self.local_heading - self.avoid_heading) # 계수 튜닝 필요 할 수도
                        else:
                            self.state = 'R'

                    # 장애물의 위치가 왼쪽 -> 오른쪽으로 회피 
                    else :
                        if (self.avoid_heading < self.local_heading): # 목표 heading에 도달하지 못했으면 우조향
                            self.angle = 1 * abs(self.local_heading - self.avoid_heading) # 계수 튜닝 필요 할 수도
                        else:
                            self.state = 'R'

            elif self.state == 'R':

                if self.local_heading != None:
                    
                    # 장애물의 위치가 중간 or 오른쪽 -> 왼쪽으로 회피 -> 오른쪽 복귀
                    if self.direction == 'left': # -> 장애물의 위치 기반 방향 선택

                        if (self.return_heading < self.local_heading): # 목표 heading에 도달하지 못했으면 우조향
                            self.angle = 1 * abs(self.local_heading - self.return_heading) # 계수 튜닝 필요 할 수도
                        else:
                            self.state = 'L'
                            self.gt_heading = None
                            self.avoid_heading = None
                            self.return_heading = None
                            self.static_obstacle_cnt = 0
                            self.is_static = False

                    # 장애물의 위치가 왼쪽 -> 오른쪽으로 회피 -> 왼쪽 복귀
                    else:
                        if (self.return_heading > self.local_heading): # 목표 heading에 도달하지 못했으면 좌조향
                            self.angle = -1 * abs(self.local_heading - self.return_heading) # 계수 튜닝 필요 할 수도
                        else:
                            self.state = 'L'
                            self.gt_heading = None
                            self.avoid_heading = None
                            self.return_heading = None
                            self.static_obstacle_cnt = 0
                            self.is_static = False
            

            # 정적 모드를 들어감과 동시에 현재의 heading을 gt heading으로 정하기. 
            
            # 일정 기준 heading을 달성할 때 까지 오른쪽으로 쭉 가기 

            # 다시 일정 기준 heading을 만족하도록 돌아오게 하기. 


            # rospy.loginfo(f"STATE: {self.state}")
            # rospy.loginfo(f"COUNT: {self.static_obstacle_cnt}")
            # rospy.loginfo(f"GT: {self.gt_heading}")
            # rospy.loginfo(f"AVOID: {self.avoid_heading}")
            # rospy.loginfo(f"RETURN: {self.return_heading}")

            self.publishCtrlCmd(self.speed, self.angle, self.is_static)

            # self.static_pub.publish(self.steer)
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

    def modeCB(self, msg):
        self.mode = msg.data


    def update_last_n_obstacles_y(self, y, n):
        self.last_n_obstacles_y.append(y)
        if len(self.last_n_obstacles_y) > n:
            self.last_n_obstacles_y.pop(0)

    def publishCtrlCmd(self, motor_msg, servo_msg, flag):
        self.ctrl_cmd_msg.speed = round(motor_msg)  # 모터 속도 설정
        self.ctrl_cmd_msg.angle = round(servo_msg)  # 조향각 설정
        self.ctrl_cmd_msg.flag = flag
        self.ctrl_cmd_pub.publish(self.ctrl_cmd_msg)  # 명령 퍼블리시


if __name__ == '__main__':
    rospy.init_node('static_obstacle_avoidance', anonymous=True)
    try:
        static_obstacle_avoidance = StaticAvoidance()
    except rospy.ROSInterruptException:
        pass
