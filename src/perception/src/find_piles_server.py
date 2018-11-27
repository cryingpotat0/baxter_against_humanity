#! /usr/bin/env python

import rospy
import actionlib

from perception_msgs.msg import FindPilesAction


class FindPilesServer:
  def __init__(self):
    self.server = actionlib.SimpleActionServer('find_piles', FindPilesAction, self.execute, False)
    print("Starting FindPiles Server")
    self.server.start()

  def execute(self, goal):
    # Do lots of awesome groundbreaking robot stuff here
    print("executing FindPiles goal")

if __name__ == '__main__':
  rospy.init_node('find_piles_server')
  server = FindPilesServer()
  rospy.spin()