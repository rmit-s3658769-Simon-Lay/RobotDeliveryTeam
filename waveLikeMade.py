#!/usr/bin/env python


import time


""" Rospy is a pure Python client library for ROS. The rospy client API enables Python
Programmers to quickly interface with ROS Topics, Services, and Parameters. """
import rospy
import baxter_interface
from baxter_interface import CHECK_VERSION

""" Std_msgs contains wrappers for ROS primitive types, which are documented in the msg
specification: https://wiki.ros.org/msg """
from std_msgs.msg import (
    UInt16,
    )

class Waver(object):
    def __init__(self):
        # Waves like made
        """ You can create a handle to publish messages to a topic using the
	rospy.Publisher class. The most common usage for this is to provide
        the name of the topic and the messsage class/type of the topic. You
        can then call publish() on that handle to publish a message:
        https://wiki.ros.org/rospy/Overview/Publishers%20and%20Subscribers """
        self.publishingRate = rospy.Publisher("robot/joint_state_publisher",
                                              UInt16, queue_size = 10)
        """ Note: "As of Hydro it is recommended to use the new asynchronous
	publishing behavior which is more in line with the behavior of roscpp."
	We are using kinetic (I think.) it is from 2016 which is newer then hydro
	so we should use the async method as recommended if possible """
        # Messages will be dropped after queue_size has been exceeded (presumably
        # this only happens if we are publishing faster then rospy can send messages.)
        self.leftArm = baxter_interface.limb.Limb("left")
        self.rightArm = baxter_interface.limb.Limb("right")
        self.leftGripper = baxter_interface.gripper.Gripper("left")
        self.rightGripper = baxter_interface.gripper.Gripper("right")
        self.leftJointNames = self.leftArm.joint_names()
        self.rightJointNames = self.rightArm.joint_names()

        # Control parameters
        self.rate = 100.0 # Hz

        rospy.loginfo("Getting robot state... ")
        self.robotState = baxter_interface.RobotEnable(CHECK_VERSION)
        self.initState = self.robotState.state().enabled
        rospy.loginfo("Enabling robot... ")
        self.robotState.enable()

        # Set joint state publishing to 500Hz
        self.publishingRate.publish(self.rate)


        
    def resetControlModes(self):
        rate = rospy.Rate(self.rate)
        for _ in xrange(100):   # https://www.pythoncentral.io/how-to-use-pythons-xrange-and-range/
            if rospy.is_shutdown():
                return False
            """ The method exit_control_mode() of the limb interface switches to
	position controller mode from torque/velocity controller and saves the
	current joint angles as the current joint position. Finally, it checks
	if the robot was initially disabled and if so disables it:
		https://sdk.rethinkrobotics.com/intera/Joint_Torque_Springs_Example  """
            self.leftArm.exit_control_mode()
            self.rightArm.exit_control_mode()
            self.publishingRate.publish(100) # 100Hz default joint state rate
            """ We think this puts the thread to sleep for a time t that is based on the rate:
		https://stackoverflow.com/questions/23227024/difference-between-spin-and-rate-sleep-in-ros  """
            rate.sleep()
        return True

    
    def moveArmsToSafetyPosition(self):
        # Set both arms back into a pose that is deemed to be relatively safe
        rospy.loginfo("Moving to neutral pose...")
        """ The move_to_neutral() method of the limb interface, moves all the joints to their neutral pose.
	This method employs the position controller to send the joints to a pre-defined neutral position:
	https://sdk.rethinkrobotics.com/intera/Joint_Torque_Springs_Example  """
        rate = rospy.Rate(self.rate)
        self.leftGripper.open()
        self.rightGripper.open()
        self.__moveLeftArmToSafetyPosition(rate)
        self.__moveRightArmToSafetyPosition(rate)


    """ Joint/s should be within this range to be considered in position
	This is basically the tolerance we are aiming for when moving joints
	(also why doesn't python have constants or case statments >:(0) """
    JOINT_PLAY = 0.0005
    LEFT_ARM_JOINTS = ["left_w2", "left_w1", "left_w0", "left_e1", "left_e0", "left_s1", "left_s0"]
    RIGHT_ARM_JOINTS = ["right_w2", "right_w1", "right_w0", "right_e1", "right_e0", "right_s1", "right_s0"]


    def __moveLeftArmToSafetyPosition(self, rate):
        # Set left arm into a pose that is deemed to be relatively safe
        ARM = "left"
        rospy.loginfo("moving " + ARM + " arm to the designated safety position")
        jointPositions = {self.LEFT_ARM_JOINTS[0]: 0.0, self.LEFT_ARM_JOINTS[1]: 0.1, self.LEFT_ARM_JOINTS[2]: 0.0,
                          self.LEFT_ARM_JOINTS[3]: 0.5, self.LEFT_ARM_JOINTS[4]: 0.0, self.LEFT_ARM_JOINTS[5]: 1.1,
                          self.LEFT_ARM_JOINTS[6]: 0.8}
        while True:
            self.publishingRate.publish(self.rate) # Set publishing rate
            self.leftArm.set_joint_positions(jointPositions)
            rate.sleep()
            nInPosition = self.__getNumberOfArmAndGripperJointsInPosition(ARM, jointPositions)
            if(nInPosition == len(self.LEFT_ARM_JOINTS) -1):
                rospy.loginfo(ARM + " arm moved to safety position")
                break


    def __moveRightArmToSafetyPosition(self, rate):
        # Set right arm into a pose that is deemed to be relatively safe
        ARM = "right"
        rospy.loginfo("moving " + ARM + " arm to the designated safety position")
        jointPositions = {self.RIGHT_ARM_JOINTS[0]: 0.0, self.RIGHT_ARM_JOINTS[1]: 0.1, self.RIGHT_ARM_JOINTS[2]: 0.0,
                          self.RIGHT_ARM_JOINTS[3]: 0.5, self.RIGHT_ARM_JOINTS[4]: 0.0, self.RIGHT_ARM_JOINTS[5]: 1.1,
                          self.RIGHT_ARM_JOINTS[6]: -0.8}
        while True:
            self.publishingRate.publish(self.rate) # Set publishing rate
            self.rightArm.set_joint_positions(jointPositions)
            rate.sleep()
            nInPosition = self.__getNumberOfArmAndGripperJointsInPosition(ARM, jointPositions)
            if(nInPosition == len(self.LEFT_ARM_JOINTS) -1):
            	rospy.loginfo(ARM + " arm moved to safety position")
            	break

    def cleanShutdown(self):
        rospy.loginfo("\nExiting example...")
        # Return to normal
        self.resetControlModes()
        if not self.initState:
            rospy.loginfo("Disabling robot...")
            self.robotState.disable()
        return True


    def waveRightArm(self, waveFor):
        rate = rospy.Rate(self.rate)
        self.moveArmsToSafetyPosition()
        rospy.loginfo("Hello I am waving at you :)")
        waveFactor = .15
	startTime = time.time()
	while True:
            if((time.time() - startTime) >= waveFor):
            	break
            self.__waveRightArm(rate, "inward")
            time.sleep(waveFactor)
            self.__waveRightArm(rate, "outward")
        time.sleep(waveFactor)
        self.__waveRightArm(rate, "inward") # Arm will be closer to body and this is a safer position to move to the nutural postion from
        time.sleep(waveFactor)


    def waveLeftArm(self):
        rate = rospy.Rate(self.rate)
        self.moveArmsToSafetyPosition()
        rospy.loginfo("Hello I am waving at you :)")
        

    def __waveRightArm(self, rate, direction):
        # Directions we can move in
        DIRECTION = ["inward", "outward"]
        ARM = "right"
        jointPositions = {}
        if(direction == DIRECTION[0]):
            jointPositions = {self.RIGHT_ARM_JOINTS[0]: 0.0, self.RIGHT_ARM_JOINTS[1]: 0.4, self.RIGHT_ARM_JOINTS[2]: -0.2,
                              self.RIGHT_ARM_JOINTS[3]: 2.06, self.RIGHT_ARM_JOINTS[4]: 3.0, self.RIGHT_ARM_JOINTS[5]: 2.5,
                              self.RIGHT_ARM_JOINTS[6]: 0.6}
        else:
            if(direction == DIRECTION[1]):
                jointPositions = {self.RIGHT_ARM_JOINTS[0]: 0.0, self.RIGHT_ARM_JOINTS[1]: 0.4, self.RIGHT_ARM_JOINTS[2]: 0.5,
                                  self.RIGHT_ARM_JOINTS[3]: 2.0, self.RIGHT_ARM_JOINTS[4]: 3.0, self.RIGHT_ARM_JOINTS[5]: 2.5,
                                  self.RIGHT_ARM_JOINTS[6]: 0.3}
            else:
                rospy.logerr("Error in __waveArms, pos = " + direction + ", but the only option/s currently implemented are " + DIRECTION)
                return
        while True:
            self.publishingRate.publish(self.rate) # Set publishing rate
            self.rightArm.set_joint_positions(jointPositions)
            rate.sleep()
            nInPosition = self.__getNumberOfArmAndGripperJointsInPosition(ARM, jointPositions)
            if(nInPosition == len(self.RIGHT_ARM_JOINTS) -1):
                rospy.loginfo(ARM + " has been waved in direction " + direction + "!")
                break;
        
            
    def __getNumberOfArmAndGripperJointsInPosition(self, ARM, jointPositions):
        # Note that ARM and jointPositions must be for the same arm
        nInPosition = 0 # Number of joints that are in position
        ARMS = ["left", "right"]

        for iter in range(0, len(ARMS)):
            if(ARM == ARMS[iter]):
                if(ARM == ARMS[0]):
                    for iter in range(0, len(self.LEFT_ARM_JOINTS)):
                        if(self.leftArm.joint_angle(self.LEFT_ARM_JOINTS[iter]) >= (jointPositions[self.LEFT_ARM_JOINTS[iter]] - self.JOINT_PLAY) and
                           self.leftArm.joint_angle(self.LEFT_ARM_JOINTS[iter]) <= (jointPositions[self.LEFT_ARM_JOINTS[iter]] + self.JOINT_PLAY)):
                            nInPosition += 1
            else:
                if(ARM == ARMS[1]):
                    for iter in range(0, len(self.RIGHT_ARM_JOINTS)):
                        if(self.rightArm.joint_angle(self.RIGHT_ARM_JOINTS[iter]) >= (jointPositions[self.RIGHT_ARM_JOINTS[iter]] - self.JOINT_PLAY) and
                           self.rightArm.joint_angle(self.RIGHT_ARM_JOINTS[iter]) <= (jointPositions[self.RIGHT_ARM_JOINTS[iter]] + self.JOINT_PLAY)):
                            nInPosition += 1
            return nInPosition

        # We should only reach this point if ARM contains an invalid value, i.e. one that is not in ARMS
        rospy.logerr("Error in  __getNumberOfArmAndGripperJointsInPosition(), invalid value passed via veriable (" + ARM +
              ") ARM. Valid values are " + ARMS)


def main():
    waver = Waver()
    rospy.on_shutdown(waver.cleanShutdown)
    waver.waveRightArm(6)
    time.sleep(1)
    waver.moveArmsToSafetyPosition()
    time.sleep(2)

    rospy.loginfo("Finished.")

if __name__ == "__main__":
    rospy.loginfo("Initializing node... ")
    rospy.init_node("waver")
    main()
