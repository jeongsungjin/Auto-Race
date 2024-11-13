#!/usr/bin/env python3

import rospy
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

def publish_image():
    rospy.init_node('image_publisher', anonymous=True)
    image_pub = rospy.Publisher('/usb_cam/image_rect_color', Image, queue_size=10)
    bridge = CvBridge()

    cap = cv2.VideoCapture('/dev/video4')

    if not cap.isOpened():
        return

    while not rospy.is_shutdown():
        ret, frame = cap.read()
        if not ret:
            break
        image_msg = bridge.cv2_to_imgmsg(frame, "bgr8")

        image_pub.publish(image_msg)
        rospy.Rate(30).sleep()

    cap.release()

if __name__ == '__main__':
    try:
        publish_image()
    except rospy.ROSInterruptException:
        pass