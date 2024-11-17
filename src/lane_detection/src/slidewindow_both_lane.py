#!/usr/bin/env python
import cv2
import numpy as np
import rospy
from std_msgs.msg import String, Int32

class SlideWindow:

    def __init__(self):
        rospy.Subscriber('/lane_topic', String, self.lane_callback)

        self.current_line = "DEFAULT"
        self.lane_side = "BOTH"
        self.left_fit = None
        self.right_fit = None
        self.x_previous = 320

    def lane_callback(self, msg):
        if msg.data == "LEFT":
            self.lane_side = "LEFT"
        elif msg.data == "RIGHT":
            self.lane_side = "RIGHT"
        else:
            self.lane_side = "BOTH"

    def slidewindow(self, img):
        x_location = 320
        out_img = np.dstack((img, img, img)) * 255
        height, width = img.shape[0], img.shape[1]
        self.white_cnt = 0

        window_height = 20
        nwindows = 20
        nonzero = img.nonzero()
        nonzeroy = np.array(nonzero[0])
        nonzerox = np.array(nonzero[1])
        margin = 40
        minpix = 0
        left_lane_inds = np.array([], dtype=int)
        right_lane_inds = np.array([], dtype=int)

        win_h1 = 380 
        win_h2 = 480
        
        
        # win_l_w_l = 185 - 80
        # win_l_w_r = 185 + 80
        # win_r_w_l = 455 - 80
        # win_r_w_r = 455 + 80

        win_l_w_l = 145 - 80
        win_l_w_r = 145 + 80
        win_r_w_l = 495 - 80
        win_r_w_r = 495 + 80
        
        circle_height = 100
        road_width = 0.5
        half_road_width = 0.5 * road_width
        
        print(self.lane_side)
        if self.lane_side == "LEFT" or self.lane_side == "BOTH":
            pts_left = np.array([[win_l_w_l, win_h2], [win_l_w_l, win_h1], [win_l_w_r, win_h1], [win_l_w_r, win_h2]], np.int32)
            cv2.polylines(out_img, [pts_left], False, (0, 255, 0), 1)
            good_left_inds = ((nonzerox >= win_l_w_l) & (nonzeroy <= win_h2) & (nonzeroy > win_h1) & (nonzerox <= win_l_w_r)).nonzero()[0]
            if len(good_left_inds) > 0:
                line_flag = 1
                left_lane_inds = np.concatenate((left_lane_inds, good_left_inds))
            else:
                line_flag = 3

        if self.lane_side == "RIGHT" or self.lane_side == "BOTH":
            pts_right = np.array([[win_r_w_l, win_h2], [win_r_w_l, win_h1], [win_r_w_r, win_h1], [win_r_w_r, win_h2]], np.int32)
            cv2.polylines(out_img, [pts_right], False, (255, 0, 0), 1)
            good_right_inds = ((nonzerox >= win_r_w_l) & (nonzeroy <= win_h2) & (nonzeroy > win_h1) & (nonzerox <= win_r_w_r)).nonzero()[0]
            if len(good_right_inds) > 0:
                line_flag = 2
                right_lane_inds = np.concatenate((right_lane_inds, good_right_inds))
            else:
                line_flag = 3
        y_current = height - 1
        x_current = None

        if line_flag == 1 and len(left_lane_inds) > 0:
            x_current = int(np.mean(nonzerox[left_lane_inds]))
        elif line_flag == 2 and len(right_lane_inds) > 0:
            x_current = int(np.mean(nonzerox[right_lane_inds]))
        else:
            self.current_line = "MID"
            alpha = 0.9  # 이동 평균 계수 (0.9에 가까울수록 더 느리게 변화)
            self.x_previous = int(alpha * self.x_previous + (1 - alpha) * x_location)
            x_location = self.x_previous

        for window in range(nwindows):
            if line_flag == 1: 
                # rectangle x,y range init
                win_y_low = y_current - (window + 1) * window_height
                win_y_high = y_current - (window) * window_height
                win_x_low = x_current - margin
                win_x_high = x_current + margin
                
                # draw rectangle
                # 0.33 is for width of the road
                cv2.rectangle(out_img, (win_x_low, win_y_low), (win_x_high, win_y_high), (0, 255, 0), 1)
                cv2.rectangle(out_img, (win_x_low + int(width * road_width), win_y_low), (win_x_high + int(width * road_width), win_y_high), (255, 0, 0), 1)
                
                # indicies of dots in nonzerox in one square
                good_left_inds = ((nonzeroy >= win_y_low) & (nonzeroy < win_y_high) & (nonzerox >= win_x_low) & (nonzerox < win_x_high)).nonzero()[0]
                
                # check num of indicies in square and put next location to current 
                if len(good_left_inds) > minpix:
                    x_current = int(np.mean(nonzerox[good_left_inds]))
                
                elif nonzeroy[left_lane_inds] != [] and nonzerox[left_lane_inds] != []:
                    p_left = np.polyfit(nonzeroy[left_lane_inds], nonzerox[left_lane_inds], 2) 
                    x_current = int(np.polyval(p_left, win_y_high))
                    
                # 338~344 is for recognize line which is yellow line in processed image(you can check in imshow)
                # print("win:", win_y_low)          
                          
                if circle_height - 10 <= win_y_low < circle_height + 10:
                    # 0.165 is the half of the road(0.33)
                    x_location = int(x_current + width * half_road_width)
                    # self.x_previous = x_location
                    cv2.circle(out_img, (x_location, circle_height), 10, (0, 0, 255), 5)

            elif line_flag == 2: # change line from left to right above(if)
                
                win_y_low = y_current - (window + 1) * window_height
                win_y_high = y_current - (window) * window_height
                win_x_low = x_current - margin
                win_x_high = x_current + margin
                
                
                cv2.rectangle(out_img, (win_x_low - int(width * road_width), win_y_low), (win_x_high - int(width * road_width), win_y_high), (0, 255, 0), 1)
                cv2.rectangle(out_img, (win_x_low, win_y_low), (win_x_high, win_y_high), (255, 0, 0), 1)
                
                good_right_inds = ((nonzeroy >= win_y_low) & (nonzeroy < win_y_high) & (nonzerox >= win_x_low) & (nonzerox < win_x_high)).nonzero()[0]

                if len(good_right_inds) > minpix:
                    x_current = int(np.mean(nonzerox[good_right_inds]))

                elif nonzeroy[right_lane_inds] != [] and nonzerox[right_lane_inds] != []:
                    p_right = np.polyfit(nonzeroy[right_lane_inds], nonzerox[right_lane_inds], 2) 
                    x_current = int(np.polyval(p_right, win_y_high))
                    
                if circle_height - 10 <= win_y_low < circle_height + 10:
                    # 0.165 is the half of the road(0.33)
                    x_location = int(x_current - width * half_road_width) 
                    # self.x_previous = x_location
                    cv2.circle(out_img, (x_location, circle_height), 10, (0, 0, 255), 5)
            

        return out_img, x_location, self.current_line
