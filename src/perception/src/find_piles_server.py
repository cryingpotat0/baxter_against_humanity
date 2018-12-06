#! /usr/bin/env python

import rospy
import actionlib
from moveit_msgs.srv import GetPositionIK, GetPositionIKRequest, GetPositionIKResponse
from moveit_commander import MoveGroupCommander
from perception_msgs.msg import FindPilesAction, FindPilesResult
from cv_bridge import CvBridge, CvBridgeError
import card_table_detection
from sensor_msgs.msg import Image
import cv2
import numpy as np
import baxter_interface

class FindPilesServer:
  def __init__(self):
    self.server = actionlib.SimpleActionServer('find_piles', FindPilesAction, self.execute, False)
    print("Starting FindPiles Server")
    self.server.start()
    rospy.wait_for_service('compute_ik')
    self.compute_ik = rospy.ServiceProxy('compute_ik', GetPositionIK)
    self.cv_bridge = CvBridge()
    self.piles = {}
    self.pile_coords = {}
    self.arm_pos = [0.65, 0.04, -0.11]
    self.IMG_HEIGHT = 800
    self.IMG_WIDTH = 1280

  def move_arm(self, pos):
    # Do lots of awesome groundbreaking robot stuff here
    compute_ik = self.compute_ik
    request = GetPositionIKRequest()
    request.ik_request.group_name = "left_arm"
    request.ik_request.ik_link_name = "left_hand_camera"
    request.ik_request.attempts = 20
    request.ik_request.pose_stamped.header.frame_id = "base"

    # Move gripper to an example place, like 0.695 -0.063 -0.222
    
    #Set the desired orientation for the end effector HERE
    request.ik_request.pose_stamped.pose.position.x = pos[0]#0.8
    request.ik_request.pose_stamped.pose.position.y = pos[1]#0.04#goal.card_pos[1]
    request.ik_request.pose_stamped.pose.position.z = pos[2]#0.1 #goal.card_pos[2]       
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
        group = MoveGroupCommander("left_arm")

        # Setting position and orientation target
        group.set_pose_target(request.ik_request.pose_stamped)

        # TRY THIS
        # Setting just the position without specifying the orientation
        #group.set_position_target(goal.card_pos)

        # Plan IK and execute
        group.go()
        
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

  def execute(self, goal):
    self.move_arm(pos=[0.750, 0.65, 0.054])
    ### TEST
    # self.piles = {"black": [10,2,3], "white": [5,6,7]}
    # result = FindPilesResult(self.piles_to_string())


    # self.server.set_succeeded(result)
    # return
    ### TEST


    self.move_arm(pos=self.arm_pos)
    img = self.update_piles()
    self.move_arm(pos=[0.750, 0.65, 0.054])
    cv2.destroyAllWindows()
    img = cv2.resize(img, (1024, 600))
    if img is not None:
      print("found good")
      msg = self.cv_bridge.cv2_to_imgmsg(img, encoding="bgr8")
      pub = rospy.Publisher('/robot/xdisplay', Image, latch=True, queue_size=1)
      pub.publish(msg)
      rospy.sleep(1)
      result = FindPilesResult(self.piles_to_string())


      self.server.set_succeeded(result)
    else: 
      print("did not find good")
        #failure

        #cv2.imshow("converted", frame_white)
        #cv2.waitKey(10)
    #cv2.destroyAllWindows()



  def recursive_find(self, pos, color):
      # color: black=True, white=False
      # pos: rough pos
    NUM_TRIES = 4
    pos = np.array(pos)
    x_c, y_c = self.IMG_WIDTH // 2, self.IMG_HEIGHT // 2
    # if color:
    #   pass
    #     # balck
    # else:
    pos[2] -= 0.1
    print("MOVING TO ", pos)
    curr_dist_from_center = float('inf')
    best_img = None
    thresh = 0.015
    while curr_dist_from_center > thresh:
      self.move_arm(pos)

      all_cards = []
      for _ in range(NUM_TRIES):
        frame = rospy.wait_for_message("cameras/left_hand_camera/image", Image)
        frame = self.cv_bridge.imgmsg_to_cv2(frame, "bgr8")
        if color:
          card, _, img = card_table_detection.get_contours(frame)
        else:
          _, card, img = card_table_detection.get_contours(frame)
        if len(card) == 1:
          best_img = img
          all_cards.append(card[0])
      all_cards = np.array(all_cards)
      if len(all_cards) == 0: break
      #print(all_cards)
      best_card = all_cards.mean(axis=0).astype(int)
      print(best_card)
      card_x_c, card_y_c = (best_card[0] + best_card[2]) // 2, (best_card[1] + best_card[3]) // 2
      
      new_pos = np.array(self.get_dist_from_center(best_card, pos) + (pos[2],))
      curr_dist_from_center = np.sqrt(sum((new_pos -pos) ** 2))
      print("DIST", curr_dist_from_center, card_x_c, card_y_c, x_c, y_c)
      print("POS", new_pos, pos)
      cv2.imshow('img', best_img)
      cv2.waitKey(1000)
      if curr_dist_from_center < thresh: break
      
      pos = new_pos
    pos[2] -= self.get_height() / 1000.0
    print("final pos ", pos, self.get_height())
    return pos

  def piles_to_string(self):
    ret = []
    for pile, pos in self.piles.items():
      ret.append(pile + "," + str(list(pos))[1:-1])
    return ret

  def update_piles(self):
    NUM_TRIES = 15
    if len(self.piles) == 0:
      # get only initial black and white piles
      try:
        all_blacks, all_whites = [], []
        best_img = None
        for _ in range(NUM_TRIES):
          frame = rospy.wait_for_message("cameras/left_hand_camera/image", Image)
          frame = self.cv_bridge.imgmsg_to_cv2(frame, "bgr8")
          blacks, whites, img = card_table_detection.get_contours(frame)
          # cv2.imshow('img', img)
          # cv2.waitKey(1000)

          if len(blacks) == 1:
            # should be exactly one black pile
            all_blacks.append(blacks[0])
          if len(whites) == 1:
            all_whites.append(whites[0])
          if len(whites) == 1 and len(blacks) == 1:
            best_img = img
        
        print(all_blacks, "blacks\n", all_whites, "whites")
        all_blacks = np.array(all_blacks)
        all_whites = np.array(all_whites)
        best_black = all_blacks.mean(axis=0).astype(int)
        best_white = all_whites.mean(axis=0).astype(int)
        print(best_white, best_black)
        cv2.imshow('img', best_img)
        cv2.waitKey(1000)
        self.pile_coords["white"] = best_white
        self.pile_coords["black"] = best_black
        
        white_pos = self.get_dist_from_center(best_white, self.arm_pos)
        white_pos = self.recursive_find(list(white_pos) + [self.arm_pos[2]], False)
        black_pos = self.get_dist_from_center(best_black, self.arm_pos)
        black_pos = self.recursive_find(list(black_pos) + [self.arm_pos[2]], True)
        print(white_pos, black_pos)
        self.piles["white"] = white_pos
        self.piles["black"] = black_pos
        return best_img
      except Exception as e:
        print("failed", str(e))
        return None
    elif len(self.piles) >= 2:
        black, white = self.piles["black"], self.piles["white"]
        all_blacks, all_whites = [], []
        best_img = None
        for _ in range(NUM_TRIES):
          frame = rospy.wait_for_message("cameras/left_hand_camera/image", Image)
          frame = self.cv_bridge.imgmsg_to_cv2(frame, "bgr8")
          blacks, whites, img = card_table_detection.get_contours(frame)
          # cv2.imshow('img', img)
          # cv2.waitKey(1000)

          if len(blacks) == 2:
            # should be exactly one black pile
            black = max(blacks, key=lambda black: (np.array(black) - self.pile_coords["black"]) ** 2)
            all_blacks.append(black)
          if len(whites) == 2:
            white = max(whites, key=lambda white: (np.array(white) - self.pile_coords["white"]) ** 2)
            all_whites.append(white)
          if len(whites) == 2 and len(blacks) == 2:
            best_img = img
        
        print(all_blacks, "blacks\n", all_whites, "whites")
        all_blacks = np.array(all_blacks)
        all_whites = np.array(all_whites)
        best_black = all_blacks.mean(axis=0).astype(int)
        best_white = all_whites.mean(axis=0).astype(int)
        print(best_white, best_black)
        cv2.imshow('img', best_img)
        cv2.waitKey(1000)
        white_pos = self.get_dist_from_center(best_white, self.arm_pos)
        white_pos = self.recursive_find(list(white_pos) + [self.arm_pos[2]], False)
        black_pos = self.get_dist_from_center(best_black, self.arm_pos)
        black_pos = self.recursive_find(list(black_pos) + [self.arm_pos[2]], True)
        self.piles["curr_black"] = black_pos
        self.piles["curr_white"] = white_pos
        # detect new white pile and black pile

  def get_height(self):
    dist = baxter_interface.analog_io.AnalogIO('left_hand_range').state()
    return dist
  
  def get_dist_from_center(self, card, curr_pos):
    IMG_HEIGHT = self.IMG_HEIGHT
    IMG_WIDTH = self.IMG_WIDTH
    x_c, y_c = IMG_WIDTH // 2, IMG_HEIGHT // 2
    card_x_c, card_y_c = (card[0] + card[2]) // 2, (card[1] + card[3]) // 2
    dist_x, dist_y = (x_c - card_x_c), (y_c - card_y_c)
    print(dist_x, dist_y, "distances")
    """
    delta_z = 0.24m
    f = 0.0012
    camera_dist = k * (pixel_dist)

    """
    height = self.get_height() / 1000.0
    print(height, "height")
    cc = 0.0025
    return curr_pos[0] + dist_y * cc * height, curr_pos[1] + dist_x * cc * height

if __name__ == '__main__':
  rospy.init_node('find_piles_server')
  server = FindPilesServer()
  rospy.spin()
