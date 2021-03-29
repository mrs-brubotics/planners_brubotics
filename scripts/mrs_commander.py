#!/usr/bin/env python

import rospy
from moveit_msgs.msg import DisplayTrajectory
from mrs_msgs.msg import UavState
from mrs_msgs.srv import Vec4
import numpy as np



class path_subscriber:
    def __init__(self):
        self.got_it = False
        self.way_points = [] # way_points is a trajectory_msgs/JointTrajectoryPoint[] list

    def callback(self, data):
        # Points is a list
        self.got_it = True
        self.way_points = data.trajectory[0].joint_trajectory.points
        # Anytime a new massage arrives, the callback function will be trigered 

    def listener(self):
        #rospy.init_node('trajectory_listener', anonymous=False)
        rospy.Subscriber("/move_group/display_planned_path", DisplayTrajectory, self.callback)


class mrs_utils:
    def __init__(self):
        self.position_cmd_call = rospy.ServiceProxy('/uav1/control_manager/goto', Vec4)
        self.tolerable_error = 0.2

    def take_observation(self):
        data_pose = None
        
        while data_pose is None:
            try:
                data_uavstate = rospy.wait_for_message('/uav1/odometry/uav_state', UavState, timeout=1)
                data_pose = data_uavstate.pose 
            except:
                #a = 1
                rospy.loginfo("Current drone pose not ready yet, retrying for getting robot pose")
        return data_pose

    def distance(self, data_pose, reference_position):
        current_pose = [data_pose.position.x, data_pose.position.y, data_pose.position.z]
        
        err = np.subtract(current_pose, reference_position)
        #w = np.array([1, 1, 4])
        #err = np.multiply(w,err)
        dist = np.linalg.norm(err)
        return dist

    def cmd_achieve(self, command_call): 
        cmd_achieved = False
        cmd_position_3D = command_call[:-1]
        while cmd_achieved == False:
            data_pose = self.take_observation()
            dist = self.distance(data_pose, cmd_position_3D)
            if dist < self.tolerable_error:
                cmd_achieved = True
        return cmd_achieved

if __name__ == '__main__':

    rospy.init_node('mrs_commander')
    rate = rospy.Rate(5) # ROS Rate at 5Hz

    mrs = mrs_utils()
    path_sub = path_subscriber()
    path_sub.listener()

    while not rospy.is_shutdown(): 
        if path_sub.got_it == True:
            print("Got a new path!")
            path_sub.listener()
            path = []
            for i in path_sub.way_points:
                path.append(i.positions)
            #for i in range(len(path_sub.way_points)):
                #point = [points[i].transforms[0].translation.x, points[i].transforms[0].translation.y, points[i].transforms[0].translation.z]
                #path.append(point)
            path_sub.got_it = False
            #print(path)
            # Send the command
            for i in path:
                command_position = list(i)
                command_position.append(0.0)
                #print(command_position)
                #data_pose = mrs.take_observation()
            
                mrs.position_cmd_call(command_position)
                cmd_achieved = mrs.cmd_achieve(command_position)

            #print(path)
        rate.sleep()


    
    
    
    



