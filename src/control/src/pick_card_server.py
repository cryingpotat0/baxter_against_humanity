#! /usr/bin/env python

import rospy
import actionlib

from control_msgs.msg import PickCardAction


class PickCardServer:
  def __init__(self):
    self.server = actionlib.SimpleActionServer('pick_card', PickCardAction, self.execute, False)
    print("Starting PickCard Server")
    self.server.start()

  def execute(self, goal):
    # Do lots of awesome groundbreaking robot stuff here
    print("executing PickCard goal")

if __name__ == '__main__':
  rospy.init_node('pick_card_server')
  server = PickCardServer()
  rospy.spin()