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
    box_pose.pose.position.x = 4
    box_pose.pose.position.y = 2.5
    box_pose.pose.position.z = 2.5 
    box_name = "box"
    scene.add_box(box_name, box_pose, size=(0.3, 13, 5.0))

    box_pose = geometry_msgs.msg.PoseStamped()
    box_pose.header.frame_id = "geometry_center"
    box_pose.pose.orientation.w = 1.0
    box_pose.pose.position.x = 1.0
    box_pose.pose.position.y = -4.0
    box_pose.pose.position.z = 2.5 
    box_name = "box1"
    scene.add_box(box_name, box_pose, size=(6.0, 0.3, 5.0))

    box_pose = geometry_msgs.msg.PoseStamped()
    box_pose.header.frame_id = "geometry_center"
    box_pose.pose.orientation.w = 1.0
    box_pose.pose.position.x = -2.0
    box_pose.pose.position.y = 1.5
    box_pose.pose.position.z = 2.5 
    box_name = "box2"
    #scene.add_box(box_name, box_pose, size=(0.3, 3.0, 5.0))


    box_pose = geometry_msgs.msg.PoseStamped()
    box_pose.header.frame_id = "geometry_center"
    box_pose.pose.orientation.w = 1.0
    box_pose.pose.position.x = -4.0
    box_pose.pose.position.y = 1.0
    box_pose.pose.position.z = 2.5 
    box_name = "box3"
    scene.add_box(box_name, box_pose, size=(6.0, 0.3, 5.0))

    box_pose = geometry_msgs.msg.PoseStamped()
    box_pose.header.frame_id = "geometry_center"
    box_pose.pose.orientation.w = 1.0
    box_pose.pose.position.x = 9.0
    box_pose.pose.position.y = 1.0
    box_pose.pose.position.z = 2.5 
    box_name = "box4"
    scene.add_box(box_name, box_pose, size=(0.3, 30, 5.0))


    box_pose = geometry_msgs.msg.PoseStamped()
    box_pose.header.frame_id = "geometry_center"
    box_pose.pose.orientation.w = 1.0
    box_pose.pose.position.x = -7.0
    box_pose.pose.position.y = -9.0
    box_pose.pose.position.z = 2.5 
    box_name = "box5"
    scene.add_box(box_name, box_pose, size=(0.3, 20, 5.0))

    box_pose = geometry_msgs.msg.PoseStamped()
    box_pose.header.frame_id = "geometry_center"
    box_pose.pose.orientation.w = 1.0
    box_pose.pose.position.x = -7.0
    box_pose.pose.position.y = 7.0
    box_pose.pose.position.z = 2.5 
    box_name = "box6"
    scene.add_box(box_name, box_pose, size=(22, 0.3, 5.0))

    box_pose = geometry_msgs.msg.PoseStamped()
    box_pose.header.frame_id = "geometry_center"
    box_pose.pose.orientation.w = 1.0
    box_pose.pose.position.x = 1.0
    box_pose.pose.position.y = -10.0
    box_pose.pose.position.z = 2.5 
    box_name = "box7"
    scene.add_box(box_name, box_pose, size=(40, 0.3, 5.0))

    box_pose = geometry_msgs.msg.PoseStamped()
    box_pose.header.frame_id = "geometry_center"
    box_pose.pose.orientation.w = 1.0
    box_pose.pose.position.x = -12.0
    box_pose.pose.position.y = 5.0
    box_pose.pose.position.z = 2.5 
    box_name = "box8"
    scene.add_box(box_name, box_pose, size=(0.3, 15, 5.0))

    box_pose = geometry_msgs.msg.PoseStamped()
    box_pose.header.frame_id = "geometry_center"
    box_pose.pose.orientation.w = 1.0
    box_pose.pose.position.x = -14.7
    box_pose.pose.position.y = -2.5
    box_pose.pose.position.z = 2.5 
    box_name = "box9"
    scene.add_box(box_name, box_pose, size=(6, 0.3, 5.0))
    box_pose = geometry_msgs.msg.PoseStamped()
    box_pose.header.frame_id = "geometry_center"
    box_pose.pose.orientation.w = 1.0
    box_pose.pose.position.x = 0
    box_pose.pose.position.y = 0
    box_pose.pose.position.z = 0
    box_name = "cylinder1"
    #scene.add_cylinder(box_name, box_pose, height=10.0, radius=0.5)

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
