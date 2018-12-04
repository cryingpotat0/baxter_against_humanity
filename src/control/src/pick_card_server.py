#! /usr/bin/env python
import rospy
import actionlib
from moveit_msgs.srv import GetPositionIK, GetPositionIKRequest, GetPositionIKResponse
from moveit_commander import MoveGroupCommander
import numpy as np
from numpy import linalg
from bah_control_msgs.msg import PickCardAction, PickCardResult
from geometry_msgs.msg import PoseStamped
#for the vacuum cup gripper
from baxter_interface import gripper as robot_gripper
import baxter_interface

class PickCardServer:
  def __init__(self):
    self.server = actionlib.SimpleActionServer('pick_card', PickCardAction, self.execute, False)
    print("Starting PickCard Server")
    self.server.start()
    rospy.wait_for_service('compute_ik')
    self.compute_ik = rospy.ServiceProxy('compute_ik', GetPositionIK)
    self.right_gripper = robot_gripper.Gripper('right')

  def get_height(self):
    # returns height in mm
    dist = baxter_interface.analog_io.AnalogIO('right_hand_range').state()
    return dist

  def execute(self, goal):
    #Construct the request
    compute_ik = self.compute_ik
    request = GetPositionIKRequest()
    request.ik_request.group_name = "right_arm"
    request.ik_request.ik_link_name = "right_gripper"
    request.ik_request.attempts = 20
    request.ik_request.pose_stamped.header.frame_id = "base"

    # Move gripper to an example pick, like 0.695 -0.063 -0.222
    
    #Set the desired orientation for the end effector HERE
    request.ik_request.pose_stamped.pose.position.x = goal.card_pos[0]
    request.ik_request.pose_stamped.pose.position.y = goal.card_pos[1]
    request.ik_request.pose_stamped.pose.position.z = self.get_height()      
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
        group = MoveGroupCommander("right_arm")

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
    self.right_gripper.close()
    print('Done!')
    # Do lots of awesome groundbreaking robot stuff here
    result = PickCardResult(1)
    self.server.set_succeeded(result)

if __name__ == '__main__':
  rospy.init_node('pick_card_server')
  server = PickCardServer()
  rospy.spin()
