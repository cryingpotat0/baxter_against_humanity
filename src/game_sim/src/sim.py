#! /usr/bin/env python

import roslib
roslib.load_manifest('perception_msgs')
import rospy
import actionlib
import time
from perception_msgs.msg import FindPlayersAction, FindPlayersGoal
from control_msgs.msg import SetupArmAction, SetupArmGoal

if __name__ == '__main__':
    rospy.init_node('game_sim')
    
    find_players_client = actionlib.SimpleActionClient('find_players', FindPlayersAction)
    find_players_client.wait_for_server()
    goal = FindPlayersGoal(1)
    # Fill in the goal here
    for _ in range(10):
    	time.sleep(2)
    	find_players_client.send_goal(goal)
    	find_players_client.wait_for_result(rospy.Duration.from_sec(50.0))
    	print(find_players_client.get_result())
    print("Done with find players")
    
    setup_arm_client = actionlib.SimpleActionClient('setup_arm', SetupArmAction)
    setup_arm_client.wait_for_server()
    goal = SetupArmGoal()
    goal.untucked = True
    # Fill in the goal here
    setup_arm_client.send_goal(goal)
    setup_arm_client.wait_for_result(rospy.Duration.from_sec(30.0))
    print("Done with setup arm")

