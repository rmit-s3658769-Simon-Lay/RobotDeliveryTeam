#!/usr/bin/env python

import rospy
import actionlib
import argparse

from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

parser = argparse.ArgumentParser(prog='move.py', description='Tells Rosie to move to a position relative to her location')
parser.add_argument("-x", '--x_axis', action="store", type=float, default=0, help="Location on the X axis, positive to move forward")
parser.add_argument("-y", '--y_axis', action="store", type=float, default=0, help="Location on the Y axis, positive to move to the left")
args = parser.parse_args();

def movebase_client():

    client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
    rospy.loginfo("Waiting for server...")
    client.wait_for_server()

    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "base_link"
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.pose.position.x =  args.x_axis;
    goal.target_pose.pose.position.y =  args.y_axis;
    goal.target_pose.pose.position.z =  0.0;
    goal.target_pose.pose.orientation.x = 0.0;
    goal.target_pose.pose.orientation.y = 0.0;
    goal.target_pose.pose.orientation.z = 0.0;
    goal.target_pose.pose.orientation.w = 1.0;
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
