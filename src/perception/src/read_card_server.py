#! /usr/bin/env python

import rospy
import actionlib

from perception_msgs.msg import ReadCardAction


class ReadCardServer:
  def __init__(self):
    self.server = actionlib.SimpleActionServer('read_card', ReadCardAction, self.execute, False)
    print("Starting ReadCard Server")
    self.server.start()

  def execute(self, goal):
    # Do lots of awesome groundbreaking robot stuff here
    print("executing ReadCard goal")

if __name__ == '__main__':
  rospy.init_node('read_card_server')
  server = ReadCardServer()
  rospy.spin()