#! /usr/bin/env python

import rospy
import actionlib

from bah_control_msgs.msg import ViewCardHeadAction


class ViewCardHeadServer:
  def __init__(self):
    self.server = actionlib.SimpleActionServer('view_card_head', ViewCardHeadAction, self.execute, False)
    print("Starting ViewCardHead Server")
    self.server.start()

  def execute(self, goal):
    # Do lots of awesome groundbreaking robot stuff here
    print("executing ViewCardHead goal")

if __name__ == '__main__':
  rospy.init_node('view_card_head_server')
  server = ViewCardHeadServer()
  rospy.spin()