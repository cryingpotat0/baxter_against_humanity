#! /usr/bin/env python
import rospy
import actionlib
from moveit_msgs.srv import GetPositionIK, GetPositionIKRequest, GetPositionIKResponse
from moveit_commander import MoveGroupCommander
import numpy as np
from numpy import linalg
from geometry_msgs.msg import PoseStamped
from control_msgs.msg import PickCardAction
#for the vacuum cup gripper
from baxter_interface import gripper as robot_gripper

class PickCardServer:
  def __init__(self):
    self.server = actionlib.SimpleActionServer('pick_card', PickCardAction, self.execute, False)
    print("Starting PickCard Server")
    self.server.start()

  def execute(self, goal):
    #Wait for the IK service to become available
    rospy.wait_for_service('compute_ik')
    rospy.init_node('service_query')
    
    #Create the function used to call the service
    compute_ik = rospy.ServiceProxy('compute_ik', GetPositionIK)

    # Set up the gripper
    left_gripper = robot_gripper.Gripper('left')
    print('Calibrating the gripper...')
    left_gripper.calibrate()
    rospy.sleep(2.0)
    
    while not rospy.is_shutdown():
        raw_input('Press [ Enter ]: ')

        # open the gripper in case it wasn't open before
        print('Opening gripper...')
        left_gripper.open()
        rospy.sleep(1.0)

        #Construct the request
        request = GetPositionIKRequest()
        request.ik_request.group_name = "left_arm"
        request.ik_request.ik_link_name = "left_gripper"
        request.ik_request.attempts = 20
        request.ik_request.pose_stamped.header.frame_id = "base"
        
        #Set the desired orientation for the end effector based on the inputted goal positions.
        request.ik_request.pose_stamped.pose.position.x = goal.x
        request.ik_request.pose_stamped.pose.position.y = goal.y
        request.ik_request.pose_stamped.pose.position.z = goal.z        
        request.ik_request.pose_stamped.pose.orientation.x = 0.0
        request.ik_request.pose_stamped.pose.orientation.y = 1.0
        request.ik_request.pose_stamped.pose.orientation.z = 0.0
        request.ik_request.pose_stamped.pose.orientation.w = 0.0
        
        try:
            #Send the request to the service
            response = compute_ik(request)
            
            #Print the response HERE
            print(response)
            group = MoveGroupCommander("left_arm")

            # Setting position and orientation target
            group.set_pose_target(request.ik_request.pose_stamped)

            # TRY THIS
            # Setting just the position without specifying the orientation
            ###group.set_position_target([0.5, 0.5, 0.0])

            # Plan IK and execute
            print('Moving arm...')
            group.go()
            
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e
        # for the picking:
        rospy.sleep(1.0)
        print('closing gripper...')
        left_gripper.close()
        rospy.sleep(1.0)

    # Do lots of awesome groundbreaking robot stuff here
    print("executing PickCard goal")

if __name__ == '__main__':
  rospy.init_node('pick_card_server')
  server = PickCardServer()
  rospy.spin()