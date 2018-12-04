#! /usr/bin/env python

import roslib
roslib.load_manifest('perception_msgs')
import rospy
import actionlib
import time
from perception_msgs.msg import FindPlayersAction, FindPlayersGoal, FindPilesAction, FindPilesGoal
from bah_control_msgs.msg import SetupArmAction, SetupArmGoal, PlaceCardAction, PlaceCardGoal, PickCardAction, PickCardGoal

if __name__ == '__main__':
    rospy.init_node('game_sim')
    """
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
    
    setup_arm_client = actionlib.SimpleActionClient('setup_arm', SetupArmAction)
    setup_arm_client.wait_for_server()
    goal = SetupArmGoal()
    goal.untucked = False
    # Fill in the goal here
    setup_arm_client.send_goal(goal)
    setup_arm_client.wait_for_result(rospy.Duration.from_sec(30.0))
    print("Done with setup arm")
    """
    """
    print("started FindPiles")
    find_piles_client = actionlib.SimpleActionClient('find_piles', FindPilesAction)
    find_piles_client.wait_for_server()
    goal = FindPilesGoal(1)
    # Fill in the goal here
    find_piles_client.send_goal(goal)
    find_piles_client.wait_for_result(rospy.Duration.from_sec(30.0))
    print("Done with FindPiles")
    
    
    setup_arm_client = actionlib.SimpleActionClient('setup_arm', SetupArmAction)
    setup_arm_client.wait_for_server()
    goal = SetupArmGoal()
    goal.untucked = False
    # Fill in the goal here
    setup_arm_client.send_goal(goal)
    setup_arm_client.wait_for_result(rospy.Duration.from_sec(30.0))
    print("Done with setup arm")
    
    place_card_client = actionlib.SimpleActionClient('place_card', PlaceCardAction)
    place_card_client.wait_for_server()
    goal = PlaceCardGoal()
    #goal.card_pos = [0.6, -0.027, -0.14]
    goal.card_pos = [0.5045, -0.06, -0.08] #-0.08 - 244.0 / 1000 + 0.08] #-0.26] #- 293.0 / 1000 + 0.05]
    #goal.card_pos = [0.8, 0.0403, 0.1]
    # Fill in the goal here
    place_card_client.send_goal(goal)
    place_card_client.wait_for_result(rospy.Duration.from_sec(30.0))
    print("Done with setup arm")
    """
    pick_card_client = actionlib.SimpleActionClient('pick_card', PickCardAction)
    pick_card_client.wait_for_server()
    goal = PlaceCardGoal()
    #goal.card_pos = [0.6, -0.027, -0.14]
    goal.card_pos = [0.5045, -0.06, -0.08] #-0.08 - 244.0 / 1000 + 0.08] #-0.26] #- 293.0 / 1000 + 0.05]
    #goal.card_pos = [0.8, 0.0403, 0.1]
    # Fill in the goal here
    pick_card_client.send_goal(goal)
    pick_card_client.wait_for_result(rospy.Duration.from_sec(30.0))
    print("Done with setup arm")
    