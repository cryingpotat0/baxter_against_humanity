#! /usr/bin/env python
import rospy
import actionlib
from moveit_msgs.srv import GetPositionIK, GetPositionIKRequest, GetPositionIKResponse
from moveit_commander import MoveGroupCommander
import numpy as np
from numpy import linalg
from bah_control_msgs.msg import PlaceCardAction, PlaceCardResult
from geometry_msgs.msg import PoseStamped
#for the vacuum cup gripper
from baxter_interface import gripper as robot_gripper


class PlaceCardServer:
  def __init__(self):
	self.server = actionlib.SimpleActionServer('place_card', PlaceCardAction, self.execute, False)
	print("Starting PlaceCard Server")
	self.server.start()
	rospy.wait_for_service('compute_ik')
	self.compute_ik = rospy.ServiceProxy('compute_ik', GetPositionIK)
	self.left_gripper = robot_gripper.Gripper('left')


  def execute(self, goal):
    #Construct the request
    compute_ik = self.compute_ik
    request = GetPositionIKRequest()
    request.ik_request.group_name = "left_arm"
    request.ik_request.ik_link_name = "left_gripper"
    request.ik_request.attempts = 20
    request.ik_request.pose_stamped.header.frame_id = "base"

    # Move gripper to an example place, like 0.695 -0.063 -0.222
    
    #Set the desired orientation for the end effector HERE
    request.ik_request.pose_stamped.pose.position.x = goal.card_pos[0]
    request.ik_request.pose_stamped.pose.position.y = goal.card_pos[1]
    request.ik_request.pose_stamped.pose.position.z = goal.card_pos[2]       
    request.ik_request.pose_stamped.pose.orientation.x = 0.0
    request.ik_request.pose_stamped.pose.orientation.y = 1.0
    request.ik_request.pose_stamped.pose.orientation.z = 0.0
    request.ik_request.pose_stamped.pose.orientation.w = 0.0

    try:
        #Send the request to the service
        print("REQUEST:", request)
        response = compute_ik(request)
        
        #Print the response HERE
        print("RESPONSE:", response)
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
    # for the placing:
    print('Opening gripper...')
    rospy.sleep(1.0)
    self.left_gripper.open()
    print('Done!')
    # Do lots of awesome groundbreaking robot stuff here
    result = PlaceCardResult(1)
    self.server.set_succeeded(result)

if __name__ == '__main__':
  rospy.init_node('place_card_server')
  server = PlaceCardServer()
  rospy.spin()
