#! /usr/bin/env python

import rospy
import actionlib

from perception_msgs.msg import FindPlayersAction

class FindPlayersServer:
  def __init__(self):
    self.server = actionlib.SimpleActionServer('find_players', FindPlayersAction, self.execute, False)
    print("Starting FindPlayers Server")
    self.server.start()

  def execute(self, goal):
    # Do lots of awesome groundbreaking robot stuff here
    if not goal.do_task: 
      self.server.set_succeeded()
      return

    self.server.set_succeeded()

if __name__ == '__main__':
  rospy.init_node('find_players_server')
  server = FindPlayersServer()
  rospy.spin()