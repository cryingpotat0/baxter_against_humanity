############# LAB 5
#!/usr/bin/env python
import rospy
from moveit_msgs.srv import GetPositionIK, GetPositionIKRequest, GetPositionIKResponse
from geometry_msgs.msg import PoseStamped

def main():
    #Wait for the IK service to become available
    rospy.wait_for_service('compute_ik')
    rospy.init_node('service_query')
    
    #Create the function used to call the service
    compute_ik = rospy.ServiceProxy('compute_ik', GetPositionIK)
    
    print('\nNow ready to compute inverse kinematics solutions.')
    while not rospy.is_shutdown():
    	# get the input
        xyz_target = raw_input('Please input a space-separated list of 3 coordinates of desired end effector position, x y z: ')
        # Split the string by whitespace
        xyz_target_list = xyz_target.split()
        # convert each string into a double / float
        xyz = []
        for x in xyz_target_list:
            xyz.append(float(x))
        # create the dictionary for ease.
        xyz_target_dict = {'x': xyz[0], 'y': xyz[1], 
                        'z': xyz[2]}

        #Construct the request
        request = GetPositionIKRequest()
        request.ik_request.group_name = "left_arm"
        request.ik_request.ik_link_name = "left_gripper"
        # To check the FK from lab3, we did NOT include the gripper, just the hand,
        # so to confirm this works only check against the hand.
        #request.ik_request.ik_link_name = 'left_hand'
        request.ik_request.attempts = 20
        request.ik_request.pose_stamped.header.frame_id = "base"
        
        #Set the desired orientation for the end effector HERE
        request.ik_request.pose_stamped.pose.position.x = xyz_target_dict['x']
        request.ik_request.pose_stamped.pose.position.y = xyz_target_dict['y']
        request.ik_request.pose_stamped.pose.position.z = xyz_target_dict['z']
        
        # leave the end effector pointed down.
        request.ik_request.pose_stamped.pose.orientation.x = 0.0
        request.ik_request.pose_stamped.pose.orientation.y = 1.0
        request.ik_request.pose_stamped.pose.orientation.z = 0.0
        request.ik_request.pose_stamped.pose.orientation.w = 0.0
        
        try:
            #Send the request to the service
            response = compute_ik(request)
            
            #Print the response HERE
            print('The joint angles for baxter, in order, are:')
            #print(response.solution.joint_state.name)
            #print(response.solution.joint_state.position)

            # Loop through the responses and output the joint angles,
            # for ease of copying to the terminal to use with lab3 FK checker code.
            # The indices in the 'name' array are 1 to 7, so
            for i in range(1, 8):
            	print(response.solution.joint_state.name[i])
            print('Angles are:')
            for j in range(1, 8):
            	print(response.solution.joint_state.position[j])
        	#for i in range(1, 8):
        	#	print(response.solution.joint_state.position[i])
            #print('\n')
            
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e

#Python's syntax for a main() method
if __name__ == '__main__':
    main()

