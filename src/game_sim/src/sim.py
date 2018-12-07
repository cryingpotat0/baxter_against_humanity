#! /usr/bin/env python

import roslib
roslib.load_manifest('perception_msgs')
import rospy
import actionlib
import time
from moveit_msgs.msg import OrientationConstraint, Constraints, CollisionObject
from perception_msgs.msg import FindPlayersAction, FindPlayersGoal, FindPilesAction, FindPilesGoal, ReadCardAction, ReadCardGoal
from bah_control_msgs.msg import SetupArmAction, SetupArmGoal, PlaceCardAction, PlaceCardGoal, PickCardAction, PickCardGoal
import numpy as np
from moveit_msgs.srv import GetPositionIK, GetPositionIKRequest, GetPositionIKResponse
from moveit_commander import MoveGroupCommander
from cv_bridge import CvBridge, CvBridgeError
import cv2
from geometry_msgs.msg import PoseStamped
from shape_msgs.msg import SolidPrimitive
from sensor_msgs.msg import Image


class GameSim():
    def __init__(self):
        self.cv_bridge = CvBridge()
        self.RIGHT_NEUTRAL = [0.7, -0.3, 0]#[0.050, 1.025, -0.133]#[0.9, -0.3, 0.18]
        self.card_piles = {"discard": self.RIGHT_NEUTRAL}
        self.card_text = {}
        self.find_piles_client = actionlib.SimpleActionClient('find_piles', FindPilesAction)
        self.find_piles_client.wait_for_server()
        self.pick_card_client = actionlib.SimpleActionClient('pick_card', PickCardAction)
        self.pick_card_client.wait_for_server()
        self.place_card_client = actionlib.SimpleActionClient('place_card', PlaceCardAction)
        self.place_card_client.wait_for_server()
        self.read_card_client = actionlib.SimpleActionClient('read_card', ReadCardAction)
        self.read_card_client.wait_for_server()

        self.compute_ik = rospy.ServiceProxy('compute_ik', GetPositionIK)
        self._planning_scene_publisher = rospy.Publisher('/collision_object', CollisionObject, queue_size=10)

    def add_box_obstacle(self, size, name, pose):
        """
        Adds a rectangular prism obstacle to the planning scene

        Inputs:
        size: 3x' ndarray; (x, y, z) size of the box (in the box's body frame)
        name: unique name of the obstacle (used for adding and removing)
        pose: geometry_msgs/PoseStamped object for the CoM of the box in relation to some frame
        """    

        # Create a CollisionObject, which will be added to the planning scene
        co = CollisionObject()
        co.operation = CollisionObject.ADD
        co.id = name
        co.header = pose.header

        # Create a box primitive, which will be inside the CollisionObject
        box = SolidPrimitive()
        box.type = SolidPrimitive.BOX
        box.dimensions = size

        # Fill the collision object with primitive(s)
        co.primitives = [box]
        co.primitive_poses = [pose.pose]

        # Publish the object
        self._planning_scene_publisher.publish(co)

    def remove_obstacle(self, name):
        """
        Removes an obstacle from the planning scene

        Inputs:
        name: unique name of the obstacle
        """

        co = CollisionObject()
        co.operation = CollisionObject.REMOVE
        co.id = name

        self._planning_scene_publisher.publish(co)


    def move_arm(self, pos, arm):
        #Construct the request
        compute_ik = self.compute_ik
        request = GetPositionIKRequest()
        request.ik_request.group_name = "{}_arm".format(arm)
        request.ik_request.ik_link_name = "{}_gripper".format(arm)
        request.ik_request.attempts = 50
        request.ik_request.pose_stamped.header.frame_id = "base"

        # Move gripper to an example pick, like 0.695 -0.063 -0.222
        # for the end effector HERE
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

        orien_const = OrientationConstraint()
        orien_const.link_name = "right_gripper";
        orien_const.header.frame_id = "base";
        orien_const.orientation.y = 1.0;
        orien_const.orientation.x = 0.0;
        orien_const.orientation.z = 0.0;

        orien_const.absolute_x_axis_tolerance = 0.5;
        orien_const.absolute_y_axis_tolerance = 0.5;
        orien_const.absolute_z_axis_tolerance = 0.5;
        orien_const.weight = 1.0;
        orien_const = [orien_const]

        pose = PoseStamped()
        #pose.header.frame_id = "base"
        
        #x, y, and z position
        #pose.pose.position.x = 0.65
        #pose.pose.position.y = 0.04
        #pose.pose.position.z = -0.26
        
        #Orientation as a quaternion
        #pose.pose.orientation.x = 0.00 
        #pose.pose.orientation.y = 0
        #pose.pose.orientation.z = 0.0
        #pose.pose.orientation.w = 1.0
        #self.add_box_obstacle([0.4, 1.2, 0.05], 'hello', pose)
        #Set the desired orientation
        self.remove_obstacle('hello')
        try:
            #Send the request to the service
            #print("REQUEST:", request)
            response = compute_ik(request)
            
            #Print the response HERE
            #print("RESPONSE:", response)
            group = MoveGroupCommander("{}_arm".format(arm))
            group.set_start_state_to_current_state()
            constraints = Constraints()
            constraints.orientation_constraints = orien_const
            group.set_path_constraints(constraints)
            # Setting position and orientation target
            group.set_pose_target(request.ik_request.pose_stamped)

            # TRY THIS
            # Setting just the position without specifying the orientation
            #group.set_position_target(goal.card_pos)
            plan = group.plan()
            group.execute(plan, True)
            # Plan IK and execute
            #group.go()
            
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e

    def initialize(self):
        # self.show_txt("hello I am baxter the funniest robot in this room")
        # self.move_arm(self.RIGHT_NEUTRAL, "right")
        # # self.move_arm([0.6992825269699097, -0.15996749699115753, -0.155], "right")#.14400000596046447], "right")
        # white_pos = [0.6897075176239014, 0.017024999484419823, 0.0]
        # white_pos[0] += -0.15
        # goal = PickCardGoal()
        # goal.card_pos = white_pos
        # self.pick_card_client.send_goal(goal)
        # self.pick_card_client.wait_for_result(rospy.Duration.from_sec(200.0))

        # waypoint = [0.6897075176239014, 0.017024999484419823, 0.0]
        # self.move_arm(waypoint, "right")
        # time.sleep(1000)
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

        NUM_OTHER_PLAYERS = 2
        NUM_WHITE_CARDS = 3
        X_DIFF = -0.15 #M BELOW (X IS TOWARDS ROBOT)
        Y_DIFF = -0.15 #M BELOW (X IS TOWARDS ROBOT)
        for i in range(NUM_WHITE_CARDS):
            curr_pos = list(self.card_piles["white"]) # copy white pile pos
            curr_pos[0] += X_DIFF
            curr_pos[1] += i * Y_DIFF
            curr_pos[2] = -0.165
            self.card_piles["my_white_{}".format(i)] = curr_pos

            goal = PickCardGoal()
            goal.card_pos = self.card_piles["white"]
            self.pick_card_client.send_goal(goal)
            self.pick_card_client.wait_for_result(rospy.Duration.from_sec(200.0))

            # REPLACE THIS WITH READ FROM HEAD
            waypoint = list(self.card_piles["white"])
            waypoint[2] = 0.1
            self.move_arm(waypoint, "right")
            # REPLACE THIS WITH READ FROM HEAD
            print("STARTED READ CARD")
            goal = ReadCardGoal()
            self.read_card_client.send_goal(goal)
            self.read_card_client.wait_for_result(rospy.Duration.from_sec(1000.0))
            result = self.read_card_client.get_result()
            self.card_text["my_white_{}".format(i)] = result.text
            self.show_txt(result.text)
            goal = PlaceCardGoal()
            goal.card_pos = curr_pos
            goal.card_pos[2] = -0.12
            self.place_card_client.send_goal(goal)
            self.place_card_client.wait_for_result(rospy.Duration.from_sec(200.0))
        
        #ROBOT START GAME BY PICKING UP BLACK CARD
        goal = PickCardGoal()
        goal.card_pos = self.card_piles["black"]
        self.pick_card_client.send_goal(goal)
        self.pick_card_client.wait_for_result(rospy.Duration.from_sec(200.0))
        
        waypoint = list(self.card_piles["white"])
        waypoint[2] = 0.1
        self.move_arm(waypoint, "right")


        goal = ReadCardGoal()
        self.read_card_client.send_goal(goal)
        self.read_card_client.wait_for_result(rospy.Duration.from_sec(200.0))
        result = self.read_card_client.get_result()
        self.show_txt(result.text)
        self.card_text["curr_black"] = result.text
        black_text = result.text

        curr_pos = list(self.card_piles["black"]) 
        curr_pos[1] -= 0.30
        goal = PlaceCardGoal()
        goal.card_pos = curr_pos
        self.place_card_client.send_goal(goal)
        self.place_card_client.wait_for_result(rospy.Duration.from_sec(200.0))

        # sleep for 30 seconds or smt while people choose cards

        # print("started FindPiles")
        # goal = FindPilesGoal(1)
        # self.find_piles_client.send_goal(goal)
        # r = self.find_piles_client.wait_for_result(rospy.Duration.from_sec(200.0))
        # result = self.find_piles_client.get_result()
        # self.parse_find_piles_result(result.positions)
        # print("Done with FindPiles")
        # print(self.card_piles)

        # self.card_piles["curr_white"]
        # curr_whites = []
        # for _ in range(NUM_OTHER_PLAYERS):
        #     goal = PickCardGoal()
        #     goal.card_pos = self.card_piles["curr_white"]
        #     self.pick_card_client.send_goal(goal)
        #     self.pick_card_client.wait_for_result(rospy.Duration.from_sec(200.0))

        #     goal = ReadCardGoal(1)
        #     self.read_card_client.send_goal(goal)
        #     self.read_card_client.wait_for_result(rospy.Duration.from_sec(200.0))
        #     result = self.read_card_client.get_result()
        #     curr_whites.append(result.text)

        #     goal = PlaceCardGoal()
        #     goal.card_pos = self.card_piles["discard"]
        #     self.place_card_client.send_goal(goal)
        #     self.place_card_client.wait_for_result(rospy.Duration.from_sec(200.0))
        curr_whites = []
        for card, text in self.card_text.items():
            if "my_white_" in card:
                curr_whites.append(text)
        print(curr_whites, self.card_text)
        best_white_text = self.choose_best_white(black_text, curr_whites)

        # display best white text
        self.show_txt(best_white_text)
        self.move_arm(self.RIGHT_NEUTRAL, "right")

    def show_img(self, img):
        img = cv2.resize(img, (1024, 600))
        # cv2.imshow('img', img)
        # cv2.waitKey(10000)
        msg = self.cv_bridge.cv2_to_imgmsg(img, encoding="bgr8")
        pub = rospy.Publisher('/robot/xdisplay', Image, latch=True, queue_size=1)
        pub.publish(msg)
        rospy.sleep(1)

    def show_txt(self, text):
        img = np.zeros((1024,1000, 3), np.uint8)

        # Write some Text

        font                   = cv2.FONT_HERSHEY_SIMPLEX
        bottomLeftCornerOfText = (10,500)
        fontScale              = 1
        fontColor              = (255,255,255)
        lineType               = 1

        cv2.putText(img,text, 
            bottomLeftCornerOfText, 
            font, 
            fontScale,
            fontColor,
            lineType)

        self.show_img(img)

    def choose_best_white(self, black, whites):
        return whites[0]

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
