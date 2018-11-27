#! /usr/bin/env python

import rospy
import actionlib

from control_msgs.msg import PlaceCardAction


class PlaceCardServer:
  def __init__(self):
    self.server = actionlib.SimpleActionServer('place_card', PlaceCardAction, self.execute, False)
    print("Starting PlaceCard Server")
    self.server.start()

  def execute(self, goal):
    # Do lots of awesome groundbreaking robot stuff here
    print("executing PlaceCard goal")

if __name__ == '__main__':
  rospy.init_node('place_card_server')
  server = PlaceCardServer()
  rospy.spin()