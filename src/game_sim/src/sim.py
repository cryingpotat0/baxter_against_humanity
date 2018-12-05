#! /usr/bin/env python

import roslib
roslib.load_manifest('perception_msgs')
import rospy
import actionlib
import time
from perception_msgs.msg import FindPlayersAction, FindPlayersGoal, FindPilesAction, FindPilesGoal
from bah_control_msgs.msg import SetupArmAction, SetupArmGoal, PlaceCardAction, PlaceCardGoal, PickCardAction, PickCardGoal
import numpy as np
from moveit_msgs.srv import GetPositionIK, GetPositionIKRequest, GetPositionIKResponse
from moveit_commander import MoveGroupCommander


class GameSim():
    def __init__(self):
        self.card_piles = {}
        self.find_piles_client = actionlib.SimpleActionClient('find_piles', FindPilesAction)
        self.find_piles_client.wait_for_server()
        self.pick_card_client = actionlib.SimpleActionClient('pick_card', PickCardAction)
        self.pick_card_client.wait_for_server()
        self.place_card_client = actionlib.SimpleActionClient('place_card', PlaceCardAction)
        self.place_card_client.wait_for_server()

        self.compute_ik = rospy.ServiceProxy('compute_ik', GetPositionIK)

    def move_arm(self, pos, arm):
        #Construct the request
        compute_ik = self.compute_ik
        request = GetPositionIKRequest()
        request.ik_request.group_name = "{}_arm".format(arm)
        request.ik_request.ik_link_name = "{}_gripper".format(arm)
        request.ik_request.attempts = 20
        request.ik_request.pose_stamped.header.frame_id = "base"

        # Move gripper to an example pick, like 0.695 -0.063 -0.222
        
        #Set the desired orientation for the end effector HERE
        request.ik_request.pose_stamped.pose.position.x = pos[0]
        request.ik_request.pose_stamped.pose.position.y = pos[1]
        request.ik_request.pose_stamped.pose.position.z = pos[2] 
        # print(goal.card_pos[2] \
        #                         - self.get_height() / 1000.0 + 0.06, goal.card_pos[2], self.get_height())
        # request.ik_request.pose_stamped.pose.position.z = goal.card_pos[2] \
        #                         - self.get_height() / 1000.0 + 0.06
        request.ik_request.pose_stamped.pose.orientation.x = 0.0
        request.ik_request.pose_stamped.pose.orientation.y = 1.0
        request.ik_request.pose_stamped.pose.orientation.z = 0.0
        request.ik_request.pose_stamped.pose.orientation.w = 0.0

        try:
            #Send the request to the service
            #print("REQUEST:", request)
            response = compute_ik(request)
            
            #Print the response HERE
            #print("RESPONSE:", response)
            group = MoveGroupCommander("{}_arm".format(arm))

            # Setting position and orientation target
            group.set_pose_target(request.ik_request.pose_stamped)

            # TRY THIS
            # Setting just the position without specifying the orientation
            #group.set_position_target(goal.card_pos)

            # Plan IK and execute
            group.go()
            
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e

    def initialize(self):
        self.move_arm([0.8, -0.6, 0.2], "right")
        # self.move_arm([0.6992825269699097, -0.15996749699115753, -0.155], "right")#.14400000596046447], "right")

        
        print("started FindPiles")
        goal = FindPilesGoal(1)
        self.find_piles_client.send_goal(goal)
        r = self.find_piles_client.wait_for_result(rospy.Duration.from_sec(200.0))
        result = self.find_piles_client.get_result()
        self.parse_find_piles_result(result.positions)
        print("Done with FindPiles")
        print(self.card_piles)
        
        # goal = PlaceCardGoal()
        # # goal.card_pos = [0.686395, 0.13789, 0.1]#-0.209]
        # goal.card_pos = self.card_piles["white"]#[0.5045, -0.06, -0]#-0.08] #-0.08 - 244.0 / 1000 + 0.08] #-0.26] #- 293.0 / 1000 + 0.05]
        # # #goal.card_pos = [0.8, 0.0403, 0.1]
        # # # Fill in the goal here
        # self.pick_card_client.send_goal(goal)
        # self.pick_card_client.wait_for_result(rospy.Duration.from_sec(60.0))

        goal = PickCardGoal()
        #goal.card_pos = [0.6992825269699097, 0.15996749699115753, -0.15]
        goal.card_pos = self.card_piles["white"]#[0.5045, -0.06, -0]#-0.08] #-0.08 - 244.0 / 1000 + 0.08] #-0.26] #- 293.0 / 1000 + 0.05]
        # #goal.card_pos = [0.8, 0.0403, 0.1]
        # # Fill in the goal here
        self.pick_card_client.send_goal(goal)
        self.pick_card_client.wait_for_result(rospy.Duration.from_sec(60.0))
        
        # white_pos = goal.card_pos #self.card_piles["white"]
        white_pos = self.card_piles["white"]

        place_pos = [white_pos[0] - 0.15, white_pos[1], 0]
        self.move_arm(place_pos ,"right")

        # print("PlaceCardGoal")
        # white_pos = self.card_piles["white"]

        # # white_pos = goal.card_pos #self.card_piles["white"]
        # place_pos = [white_pos[0] - 0.15, white_pos[1], -0.165]
        # goal = PlaceCardGoal()
        # goal.card_pos = place_pos #-0.209]
        # #goal.card_pos = self.card_piles["white"]#[0.5045, -0.06, -0]#-0.08] #-0.08 - 244.0 / 1000 + 0.08] #-0.26] #- 293.0 / 1000 + 0.05]
        # #goal.card_pos = [0.8, 0.0403, 0.1]
        # # Fill in the goal here
        # self.place_card_client.send_goal(goal)
        # self.place_card_client.wait_for_result(rospy.Duration.from_sec(60.0))

        # new_pos = goal.card_pos
        # new_pos[2] += 1
        # self.move_arm(new_pos, "right")
        # self.move_arm([0.8, -0.6, 0.2], "right")


    def parse_find_piles_result(self, result):
        for pile in result:
            print(pile)
            name, x, y, z = pile.strip().split(',')
            x, y, z = float(x), float(y), float(z)
            self.card_piles[name] = np.array([x, y, z])
        return self.card_piles


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
    pick_card_client = actionlib.SimpleActionClient('pick_card', PickCardAction)
    pick_card_client.wait_for_server()
    goal = PlaceCardGoal()
    #goal.card_pos = [0.6, -0.027, -0.14]
    goal.card_pos = [1.0454425, 0.23084  , 0.05]#[0.5045, -0.06, -0]#-0.08] #-0.08 - 244.0 / 1000 + 0.08] #-0.26] #- 293.0 / 1000 + 0.05]
    #goal.card_pos = [0.8, 0.0403, 0.1]
    # Fill in the goal here
    pick_card_client.send_goal(goal)
    pick_card_client.wait_for_result(rospy.Duration.from_sec(30.0))
    print("Done with setup arm")
    """

    
if __name__ == '__main__':
    rospy.init_node('game_sim')
    game_sim = GameSim()
    game_sim.initialize()
