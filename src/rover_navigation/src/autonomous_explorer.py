#!/usr/bin/env python
import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import random
import math

class AutonomousExplorer:
    def __init__(self):
        rospy.init_node("autonomous_explorer")
        
        # Create move_base action client
        self.client = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        rospy.loginfo("Waiting for move_base action server...")
        self.client.wait_for_server()
        rospy.loginfo("Connected to move_base action server")
        
        # Set exploration parameters
        self.exploration_range = 2.0  # meters
        
        # Start exploration
        self.explore()
        
    def explore(self):
        while not rospy.is_shutdown():
            # Create a goal in front of the robot
            goal = MoveBaseGoal()
            goal.target_pose.header.frame_id = "base_link"
            goal.target_pose.header.stamp = rospy.Time.now()
            
            # Set forward-focused goal
            angle = random.uniform(-math.pi/4, math.pi/4)  # +/- 45 degrees
            distance = random.uniform(1.0, self.exploration_range)
            
            goal.target_pose.pose.position.x = distance * math.cos(angle)
            goal.target_pose.pose.position.y = distance * math.sin(angle)
            goal.target_pose.pose.orientation.w = 1.0
            
            rospy.loginfo("Sending goal: x=%f, y=%f", 
                         goal.target_pose.pose.position.x, 
                         goal.target_pose.pose.position.y)
            
            # Send goal and wait
            self.client.send_goal(goal)
            self.client.wait_for_result(rospy.Duration(60))
            
            rospy.sleep(1)  # Short pause before next goal

if __name__ == "__main__":
    try:
        explorer = AutonomousExplorer()
    except rospy.ROSInterruptException:
        pass
