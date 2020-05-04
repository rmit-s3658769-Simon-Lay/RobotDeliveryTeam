#!/usr/bin/env python

import rospy
import actionlib

from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from tf.transformations import quaternion_from_euler

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
    q = quaternion_from_euler(90, 0, 0);
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
