#! /usr/bin/env python
import rospy
import actionlib
from moveit_msgs.srv import GetPositionIK, GetPositionIKRequest, GetPositionIKResponse
from moveit_commander import MoveGroupCommander
import numpy as np
from numpy import linalg
from control_msgs.msg import PlaceCardAction
from geometry_msgs.msg import PoseStamped
#for the vacuum cup gripper
from baxter_interface import gripper as robot_gripper


class PlaceCardServer:
  def __init__(self):
    self.server = actionlib.SimpleActionServer('place_card', PlaceCardAction, self.execute, False)
    print("Starting PlaceCard Server")
    self.server.start()

  def execute(self, goal):
    #Construct the request
        request2 = GetPositionIKRequest()
        request2.ik_request.group_name = "left_arm"
        request2.ik_request.ik_link_name = "left_gripper"
        request2.ik_request.attempts = 20
        request2.ik_request.pose_stamped.header.frame_id = "base"

        # Move gripper to an example place, like 0.695 -0.063 -0.222
        
        #Set the desired orientation for the end effector HERE
        request2.ik_request.pose_stamped.pose.position.x = goal.x
        request2.ik_request.pose_stamped.pose.position.y = goal.y
        request2.ik_request.pose_stamped.pose.position.z = goal.z        
        request2.ik_request.pose_stamped.pose.orientation.x = 0.0
        request2.ik_request.pose_stamped.pose.orientation.y = 1.0
        request2.ik_request.pose_stamped.pose.orientation.z = 0.0
        request2.ik_request.pose_stamped.pose.orientation.w = 0.0

        try:
            #Send the request to the service
            response2 = compute_ik(request2)
            
            #Print the response HERE
            print(response2)
            group = MoveGroupCommander("left_arm")

            # Setting position and orientation target
            group.set_pose_target(request2.ik_request.pose_stamped)

            # TRY THIS
            # Setting just the position without specifying the orientation
            ###group.set_position_target([0.5, 0.5, 0.0])

            # Plan IK and execute
            print('Moving arm...')
            group.go()
            
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e
        # for the placing:
        print('Opening gripper...')
        rospy.sleep(1.0)
        left_gripper.open()
        print('Done!')
        rospy.sleep(1.0)
    # Do lots of awesome groundbreaking robot stuff here
    print("executing PlaceCard goal")

if __name__ == '__main__':
  rospy.init_node('place_card_server')
  server = PlaceCardServer()
  rospy.spin()