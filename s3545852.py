#!/usr/bin/env python

#import argparse

""" rospy is a pure Python client library for ROS. The rospy client API enables Python
programmers to quickly interface with ROS Topics, Services, and Parameters. """
import rospy
import baxter_interface
from baxter_interface import CHECK_VERSION

""" std_msgs contains wrappers for ROS primitive types, which are documented in the msg
specification: https://wiki.ros.org/msg """
from std_msgs.msg import (
    UInt16,
    )

# # Limb
# """ This is the main API for the robot's arms, and is instantiated by: """
# from baxter_interface import Limb
# right_arm = Limb("right")
# left_arm = Limb("left")
# """ Its main uses are:
# 	* Quering the joint states
# 	* Switching between control modes
# 	* And sending Joint Commands (position, velocity, or torque) """

# # Gripper
# """ This is the main API for interacting with Baxter's grippers, and is instantiated
# by: """
# from baxter_interface import Gripper
# right_gripper = Gripper('right')
# left_gripper = Gripper('left')
# """ Its main uses are:

# 	* Sending open/close commands to the gripper
# 	* Querying the state/properties of the gripper
# 	* Reacting to grippers being plugged/unplugged
# 	* Calibrating the gripper
# 	* And controlling aspects of how the gripper acts (velocity, moving force,
# 	* holding force, dead band, vacuum threshold, etc.) """

# # Navigator
# """ This is the main API for responding to interaction with Baxter's navigator
# interfaces, and is instantiated by: """
# from baxter_interface import Navigator
# right_arm_navigator = Navigator('right')
# left_arm_navigator = Navigator('left')
# right_torso_navigator = Navigator('torso_right')
# left_torso_navigator = Navigator('torso_left')
# """ Its main uses are:
# 	* Querying the state of the wheel
# 	* Responding to wheel and button interactions
# 	* And controlling the navigator lights """

# # Robot Enable
# """ This is the API responsible for enabling/disabling the robot, as well as
# running version verification, and instantiated by: """
# from baxter_interface import RobotEnable
# baxter = RobotEnable()
# """ Its main uses are:
# 	* Performing Enable, Disable, Stop, Reset on the robot
# 	* Getting the current robot state
# 	* And verifying the version of the software """

# # Camera
# """ This is the main API for interacting with the cameras on the Baxter Research
# Robot, and is instantiated by: """
# from baxter_interface import CameraController
# left_hand_camera = CameraController('left_hand_camera')
# right_hand_camera = CameraController('right_hand_camera')
# head_camera = CameraController('head_camera')
# """ Its main uses are:
# 	* Opening/closing cameras
# 	* Updating camera resolution to another valid resolution mode
# 		* Valid Modes:
# 		* (1280, 800)
# 		* (960, 600)
# 		* (640, 400)
# 		* (480, 300)
# 		* (384, 240)
# 		* (320, 200)
# 	* Getting/Setting camera settings (fps, exposure, gain, white balance, etc.) """

# # Analog IO
# """ This is the catchall API for interacting with analog Input/Output interfaces in
# the Baxter Research Robot, and is instantiated by: """
# from baxter_interface import AnalogIO
# # <component name> = AnalogIO(<component id>)
# """ Available Analog Components:
# 	* left_hand_range, right_hand_range
# 	* left_itb_wheel, right_itb_wheel, torso_left_itb_wheel, torso_right_itb_wheel
# 	* left_vacuum_sensor_analog, right_vacuum_sensor_analog
# 	* torso_fan
# Available options:
# 	* _on_io_state: react to state changes
# 	* state
# 	* is_output: check to see if the component is capable of output
# 	* set_output
# For more information on the vacuum end-effector, see Vacuum Gripper Interface.  """

# # Digital IO
# """ This is the catchall API for interacting with digital Input/Output interfaces in
# the Baxter Research Robot, and is instantiated by: """
# from baxter_interface import DigitalIO
# # <component name> = DigitalIO(<component id>)
# """ A full list of available digital IO components is available here:
# 	https://sdk.rethinkrobotics.com/wiki/API_Reference#Digital_IO
# Available options:
# 	* _on_io_state: react to state changes
# 	* state
# 	* is_output: check to see if the component is capable of output
# 	* set_output """

# # Head
# """ This is the API for dealing with head motion, and is instantiated by: """
# from baxter_interface import Head
# # <component name> = Head()
# """ Available commands:
# 	* _on_head_state: respond to state changes
#     	* pan, nodding, panning: state values
#     	* set_pan, command_nod """

class Pusher(object):
    def __init__(self):
        """ 'Pushes' green button or something idk """
        """ You can create a handle to publish messages to a topic using the
	rospy.Publisher class. The most common usage for this is to provide
	the name of the topic and the message class/type of the topic. You
	can then call publish() on that handle to publish a message:
	https://wiki.ros.org/rospy/Overview/Publishers%20and%20Subscribers  """
        self.publishingRate = rospy.Publisher("robot/joint_state_publishrate",
                                         UInt16, queue_size = 10)
        """ Note: "As of Hydro it is recommended to use the new asynchronous
	publishing behavior which is more in line with the behavior of roscpp."
	We are using kinetic (I think.) it is from 2016 which is newer then hydro
	so we should use the async method as recommended if possible """
        # Messages will be dropped after queue_size has been exceeded (presumably
        # this only happens if we are publishing faster then rospy can send messages.)
        self.leftArm = baxter_interface.limb.Limb("left")
        self.rightArm = baxter_interface.limb.Limb("right")
        self.leftJointNames = self.leftArm.joint_names()
        self.rightJointNames = self.rightArm.joint_names()

        # Control parameters
        self.rate = 500.0 # Hz

        print("Getting robot state... ")
        self.robotState = baxter_interface.RobotEnable(CHECK_VERSION)
        self.initState = self.robotState.state().enabled
        print("Enabling robot... ")
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
            """ We thing this puts the thread to sleep for a time t that is based on the rate:
		https://stackoverflow.com/questions/23227024/difference-between-spin-and-rate-sleep-in-ros  """
            rate.sleep()
        return True

    def setNeutral(self):
        # Set both arms back into a neutral pose.
        print("Moving to neutral pose...")
        """ The move_to_neutral() method of the limb interface, moves all the joints to their neutral pose.
	This method employs the position controller to send the joints to a pre-defined neutral position:
	https://sdk.rethinkrobotics.com/intera/Joint_Torque_Springs_Example  """
        self.leftArm.move_to_neutral()
        self.rightArm.move_to_neutral()

    def cleanShutdown(self):
        print("\nExiting example...")
        # Return to normal
        self.resetControlModes()
        self.setNeutral()
        if not self.initState:
            print("Disabling robot...")
            self.robotState.disable()
        return True

    def push(self):
        print("we are in Puther.push")
        self.setNeutral()
        """ Execute the all important pushing sequence """
        rate = rospy.Rate(self.rate)
	start = rospy.Time.now()

        while not rospy.is_shutdown():
            self.publishingRate.publish(self.rate)
            elapsed = rospy.Time.now() - start
            print(type(elapsed))
            print("just printed!")
            newVelocity = time.Duration(0.005) * elapsed
            self.leftArm.set_joint_velocities("left_s0", newVelocity)
            rate.sleep()        # Sleep based on rate
            print("we are in loop!")
	""" self.leftJointNames
		left_s0
		left_s1
		left_e0
		left_e1
		left_w0
		left_w1
		left_w2 """

            # self.leftArm.set_joint_velocities([(joint, 0.01*elapsed)
            # for iter, joint in enumerate(self.leftJointNames)])

def main():
     # arg_format = argparse.RawDescriptionHelpFormatter
    # parser = argparse.ArgumentParser(formatter_class = arg_fmt,
    #                                  description = main.__doc__)
    # parser.parse_args(rospy.myargv()[1:])

    print("Initializing node... ")
    rospy.init_node("rsdk_pusher")

    pusher = Pusher()
    rospy.on_shutdown(pusher.cleanShutdown)
    pusher.push()

    print("Finished.")

if __name__ == "__main__":
    main()
