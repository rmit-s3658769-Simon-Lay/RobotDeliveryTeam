#!/usr/bin/env python

import rospy
import actionlib
import argparse
from geometry_msgs.msg import Twist
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal


def motion_demo():
    velocity_publisher = rospy.Publisher('/mobility_base/cmd_vel', Twist, queue_size=1)
    rospy.init_node('motion_demo');

    vel_msg = Twist()
    speed = 10
    distance = 10

    #move forward
    vel_msg.linear.x = abs(speed)
    vel_msg.linear.y = 0
    vel_msg.linear.z = 0
    vel_msg.angular.x = 0
    vel_msg.angular.y = 0
    vel_msg.angular.z = 0
    
    while not rospy.is_shutdown():
	t0 = rospy.Time.now().to_sec()
	current_distance = 0
	while(current_distance < distance):
	   velocity_publisher.publish(vel_msg)
	   t1 = rospy.Time.now().to_sec()
	   current_distance = speed*(t1-t0)
	vel_msg.linear.x = 0
	velocity_publisher.publish(vel_msg)
    pub.publish(twist_output)

if __name__ == '__main__':
  motion_demo()

