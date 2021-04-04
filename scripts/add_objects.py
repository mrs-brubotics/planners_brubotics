#!/usr/bin/env python

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list

def main():
    rospy.init_node('add_objects')
    scene = moveit_commander.PlanningSceneInterface()
    rospy.sleep(1)

    box_pose = geometry_msgs.msg.PoseStamped()
    box_pose.header.frame_id = "geometry_center"
    box_pose.pose.orientation.w = 1.0
    box_pose.pose.position.x = 3.0
    box_pose.pose.position.y = 0.0
    box_pose.pose.position.z = 2.5 
    box_name = "box"
    scene.add_box(box_name, box_pose, size=(0.3, 5.0, 5.0))

    current_known_objects = scene.get_known_object_names()
    timeout = 4
    start = rospy.get_time()
    while True:
        current_known_objects = scene.get_known_object_names()
        if box_name not in current_known_objects:
            print ("Wait for the object to spawn.")
            rospy.sleep(0.5)
            seconds = rospy.get_time()
            if seconds - start > timeout:
                print "Add objects failed."
                break

        else:
            #print(current_known_objects)
            print "Add objects done!"
            break

    #tutorial.remove_box()
    
    #return True

if __name__ == '__main__':
    main()