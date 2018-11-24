#! /usr/bin/env python

import rospy
import actionlib

from control_msgs.msg import SetupArmAction
from tuck_arms import Tuck


class SetupArmServer:
  def __init__(self):
    self.server = actionlib.SimpleActionServer('setup_arm', SetupArmAction, self.execute, False)
    self.tucker = Tuck(True)
    rospy.on_shutdown(self.tucker.clean_shutdown)
    self.untucker = Tuck(False)
    rospy.on_shutdown(self.untucker.clean_shutdown)
    print("Starting SetupArm Server")
    self.server.start()

  def execute(self, goal):
    # Do lots of awesome groundbreaking robot stuff here
    if goal.untucked:
      self.untucker.supervised_tuck()
    else:
      self.tucker.supervised_tuck()
    self.server.set_succeeded()

if __name__ == '__main__':
  rospy.init_node('setup_arm_server')
  server = SetupArmServer()
  rospy.spin()