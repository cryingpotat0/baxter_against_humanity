#!/usr/bin/env python
import rospy
from baxter_interface import Gripper
#from intera_interface import gripper as robot_gripper

#rospy.init_node('gripper_test')

#Set up the left gripper
#left_gripper = robot_gripper.Gripper('left')

#Calibrate the gripper (other commands won't work unless you do this first)
#print('Calibrating...')
#right_gripper.calibrate()
#rospy.sleep(2.0)

#Close the right gripper
#print('Closing...')
#right_gripper.close()
#rospy.sleep(1.0)

#Open the right gripper
#print('Opening...')
#right_gripper.open()
#rospy.sleep(1.0)
#print('Done!')

rospy.init_node("vacuum_test")

#Set up the left suction cup

l_vacuum_sensor = baxter_interface.AnalogIO('left_vacuum_sensor_analog')




