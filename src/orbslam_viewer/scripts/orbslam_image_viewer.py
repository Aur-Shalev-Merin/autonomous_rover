#!/usr/bin/env python
import rospy
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
bridge = CvBridge()
def callback(data):
    try:
        frame = bridge.imgmsg_to_cv2(data, "bgr8")
        cv2.imshow('ORB-SLAM2 Stereo Tracking', frame)
        cv2.waitKey(1)
        rospy.loginfo('Debug image received!')
    except Exception as e:
        rospy.logerr("Error processing image: {}".format(e))
def listener():
    rospy.init_node('orbslam_stereo_viewer')
    rospy.loginfo("Subscribing to /orb_slam2_stereo/debug_image")
    rospy.Subscriber('/orb_slam2_mono/debug_image', Image, callback)
    rospy.loginfo("Waiting for messages...")
    rospy.spin()
if __name__ == '__main__':
    try:
        listener()
    except rospy.ROSInterruptException:
        pass
