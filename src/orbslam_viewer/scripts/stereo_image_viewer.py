#!/usr/bin/env python
import rospy
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

bridge = CvBridge()

def callback(data):
    frame = bridge.imgmsg_to_cv2(data, "bgr8")
    cv2.imshow('Stereo SLAM Debug Feed', frame)
    cv2.waitKey(1)
    rospy.loginfo('Stereo debug image received!')

def listener():
    rospy.init_node('stereo_viewer')
    # Subscribe to the stereo debug image topic
    rospy.Subscriber('/orb_slam2_stereo/debug_image', Image, callback)
    rospy.spin()

if __name__ == '__main__':
    listener()
