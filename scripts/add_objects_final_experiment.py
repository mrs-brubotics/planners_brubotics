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
    box_pose.pose.position.x = 10
    box_pose.pose.position.y = 2.5
    box_pose.pose.position.z = 2.5 
    box_name = "box"
    scene.add_box(box_name, box_pose, size=(0.3, 100, 5.0))
    

    box_pose = geometry_msgs.msg.PoseStamped()
    box_pose.header.frame_id = "geometry_center"
    box_pose.pose.orientation.w = 1.0
    box_pose.pose.position.x = -10
    box_pose.pose.position.y = 2.5
    box_pose.pose.position.z = 2.5 
    box_name = "box1"
    scene.add_box(box_name, box_pose, size=(0.3, 100, 5.0))


    box_pose = geometry_msgs.msg.PoseStamped()
    box_pose.header.frame_id = "geometry_center"
    box_pose.pose.orientation.w = 1.0
    box_pose.pose.position.x = 0
    box_pose.pose.position.y = -13.0
    box_pose.pose.position.z = 2.5 
    box_name = "box2"
    scene.add_box(box_name, box_pose, size=(6.0, 0.3, 5.0))

    box_pose = geometry_msgs.msg.PoseStamped()
    box_pose.header.frame_id = "geometry_center"
    box_pose.pose.orientation.w = 1.0
    box_pose.pose.position.x = 3.0
    box_pose.pose.position.y = -3.0
    box_pose.pose.position.z = 2.5 
    box_name = "box3"
    scene.add_box(box_name, box_pose, size=(0.3, 20.0, 5.0))

    box_pose = geometry_msgs.msg.PoseStamped()
    box_pose.header.frame_id = "geometry_center"
    box_pose.pose.orientation.w = 1.0
    box_pose.pose.position.x = -3.0
    box_pose.pose.position.y = -3.0
    box_pose.pose.position.z = 2.5 
    box_name = "box4"
    scene.add_box(box_name, box_pose, size=(0.3, 20.0, 5.0))

    box_name = "boxmoving"
    box_pose = geometry_msgs.msg.PoseStamped()
    box_pose.header.frame_id = "geometry_center"
    box_pose.pose.position.x = 14
    box_pose.pose.position.y = 0
    box_pose.pose.position.z = 2.5
    scene.add_box("boxmoving", box_pose, size=(0.3, 13, 5.0))
    goLeft = False
    goRight = True
    while True:
        if goRight ==True:
            if box_pose.pose.position.x <= 6.5:
                scene.remove_world_object("boxmoving")
                box_pose.pose.position.x = box_pose.pose.position.x-0.1
                scene.add_box("boxmoving", box_pose, size=(7.0, 0.3, 5.0))
                goRight = False
                rospy.sleep(3.0)
                goLeft = True
            else:
                scene.remove_world_object("boxmoving")
                box_pose.pose.position.x = box_pose.pose.position.x-0.1
                scene.add_box("boxmoving", box_pose, size=(7.0, 0.3, 5.0))
                rospy.sleep(0.1)
        if goLeft == True:
            if box_pose.pose.position.x >= 13.5:
                scene.remove_world_object("boxmoving")
                box_pose.pose.position.x = box_pose.pose.position.x+0.1
                scene.add_box("boxmoving", box_pose, size=(7.0, 0.3, 5.0))
                goLeft = False
                rospy.sleep(3.0)
                goRight = True
            else:
                scene.remove_world_object("boxmoving")
                box_pose.pose.position.x = box_pose.pose.position.x+0.1
                scene.add_box("boxmoving", box_pose, size=(7.0, 0.3, 5.0))
                rospy.sleep(0.1)


    #tutorial.remove_box()
    
    #return True



if __name__ == '__main__':
    main()
