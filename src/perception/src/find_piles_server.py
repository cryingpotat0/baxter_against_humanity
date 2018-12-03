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
from ar_markers import detect_markers

class FindPilesServer:
  def __init__(self):
    self.server = actionlib.SimpleActionServer('find_piles', FindPilesAction, self.execute, False)
    print("Starting FindPiles Server")
    self.server.start()
    rospy.wait_for_service('compute_ik')
    self.compute_ik = rospy.ServiceProxy('compute_ik', GetPositionIK)
    self.cv_bridge = CvBridge()
    self.piles = {}
    self.arm_pos = [0.8, 0.04, 0.1]

  def move_arm(self):
    # Do lots of awesome groundbreaking robot stuff here
    compute_ik = self.compute_ik
    request = GetPositionIKRequest()
    request.ik_request.group_name = "left_arm"
    request.ik_request.ik_link_name = "left_gripper"
    request.ik_request.attempts = 20
    request.ik_request.pose_stamped.header.frame_id = "base"

    # Move gripper to an example place, like 0.695 -0.063 -0.222
    
    #Set the desired orientation for the end effector HERE
    request.ik_request.pose_stamped.pose.position.x = self.arm_pos[0]#0.8
    request.ik_request.pose_stamped.pose.position.y = self.arm_pos[1]#0.04#goal.card_pos[1]
    request.ik_request.pose_stamped.pose.position.z = self.arm_pos[2]#0.1 #goal.card_pos[2]       
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
    self.move_arm()
    img = self.update_piles()
    cv2.destroyAllWindows()
    img = cv2.resize(img, (1024, 600))
    if img is not None:
      print("found good")
      msg = self.cv_bridge.cv2_to_imgmsg(img, encoding="bgr8")
      pub = rospy.Publisher('/robot/xdisplay', Image, latch=True, queue_size=1)
      pub.publish(msg)
      rospy.sleep(1)
    else: 
      pass
        #failure

        #cv2.imshow("converted", frame_white)
        #cv2.waitKey(10)
    #cv2.destroyAllWindows()

  def update_piles(self):
    NUM_TRIES = 30
    if len(self.piles) == 0:
      # get only initial black and white piles
      try:
        all_blacks, all_whites = [], []
        best_img = None
        for _ in range(NUM_TRIES):
          frame = rospy.wait_for_message("cameras/left_hand_camera/image", Image)
          frame = self.cv_bridge.imgmsg_to_cv2(frame, "bgr8")
          blacks, whites, img = card_table_detection.get_contours(frame)
          cv2.imshow('img', img)
          cv2.waitKey(100)

          if len(blacks) == 1:
            # should be exactly one black pile
            all_blacks.append(blacks[0])
          if len(whites) == 1:
            all_whites.append(whites[0])
          if len(whites) == 1 and len(blacks) == 1:
            best_img = img
        #print(all_blacks, all_whites)
        all_blacks = np.array(all_blacks)
        all_whites = np.array(all_whites)
        best_black = all_blacks.mean(axis=0).astype(int)
        best_white = all_whites.mean(axis=0).astype(int)
        print(best_white)

        print(self.get_dist_from_center(best_white))
        

        return best_img
      except Exception as e:
        print("failed", str(e))
        return frame
  
  def get_dist_from_center(self, card):
    IMG_HEIGHT = 800
    IMG_WIDTH = 1280
    x_c, y_c = IMG_WIDTH // 2, IMG_HEIGHT // 2
    card_x_c, card_y_c = (card[0] + card[2]) // 2, (card[1] + card[3]) // 2
    dist_x, dist_y = (x_c - card_x_c), (y_c - card_y_c)
    print(dist_x, dist_y, "distances")
    """
    delta_z = 0.24m
    f = 0.0012
    camera_dist = k * (pixel_dist)

    """
    return self.arm_pos[0] + dist_x * 0.06 / 95.5, self.arm_pos[1] + dist_y * 0.08 / 135.5

  def get_xy_world(self, x_pixel, y_pixel):
    pass

if __name__ == '__main__':
  rospy.init_node('find_piles_server')
  server = FindPilesServer()
  rospy.spin()