#!/usr/bin/env python

import math
import rospy
import actionlib
import argparse

from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from tf.transformations import quaternion_from_euler

parser = argparse.ArgumentParser(prog='rotate.py', description='Tells Rosie to rotate')
parser.add_argument("-r", '--roll', action="store", type=float, default=0, help="Roll rotation in degrees, currently does not work for Rosie")
parser.add_argument("-p", '--pitch', action="store", type=float, default=0, help="Pitch rotation in degrees, currently does not work for Rosie")
parser.add_argument("-y", '--yaw', action="store", type=float, default=0, help="Yaw rotation in degrees, positive for anti-clockwise")
args = parser.parse_args();

def movebase_client():

    client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
    rospy.loginfo("Waiting for server...")
    client.wait_for_server()

    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "base_link"
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.pose.position.x =  0.0;
    goal.target_pose.pose.position.y =  0.0;
    goal.target_pose.pose.position.z =  0.0;
    q = quaternion_from_euler(math.radians(args.roll), math.radians(args.pitch), math.radians(args.yaw));
    goal.target_pose.pose.orientation.x = q[0];
    goal.target_pose.pose.orientation.y = q[1];
    goal.target_pose.pose.orientation.z = q[2];
    goal.target_pose.pose.orientation.w = q[3];
    client.send_goal(goal)
    rospy.loginfo("Waiting for response...")
    wait = client.wait_for_result()
    if not wait:
        rospy.logerr("Action server not available!")
        rospy.signal_shutdown("Action server not available!")
    else:
        return client.get_result()

if __name__ == '__main__':
    try:
        rospy.init_node('movebase_client_py')
        result = movebase_client()
        if result:
            rospy.loginfo("Goal execution done!")
        else:
            rospy.loginfo("Goal execution unsuccessful!")
    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation test finished.")
