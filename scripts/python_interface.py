#!/usr/bin/env python

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import sensor_msgs.msg
import geometry_msgs.msg
from math import pi
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list

import numpy as np
import warnings


def all_close(goal, actual, tolerance):
  """
  Convenience method for testing if a list of values are within a tolerance of their counterparts in another list
  @param: goal       A list of floats, a Pose or a PoseStamped
  @param: actual     A list of floats, a Pose or a PoseStamped
  @param: tolerance  A float
  @returns: bool
  """
  all_equal = True
  if type(goal) is list:
    for index in range(len(goal)):
      if abs(actual[index] - goal[index]) > tolerance:
        return False

  elif type(goal) is geometry_msgs.msg.PoseStamped:
    return all_close(goal.pose, actual.pose, tolerance)

  elif type(goal) is geometry_msgs.msg.Pose:
    return all_close(pose_to_list(goal), pose_to_list(actual), tolerance)

  return True

class MoveGroupPythonInteface(object):
  """MoveGroupPythonInteface"""
  def __init__(self):
    super(MoveGroupPythonInteface, self).__init__()
    
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('move_group_python_interface_tutorial', anonymous=True)

    ## Instantiate a `RobotCommander`_ object. Provides information such as the robot's
    ## kinematic model and the robot's current joint states
    robot = moveit_commander.RobotCommander()

    ## Instantiate a `PlanningSceneInterface`_ object.  This provides a remote interface
    ## for getting, setting, and updating the robot's internal understanding of the
    ## surrounding world:
    scene = moveit_commander.PlanningSceneInterface()
    uav_name = rospy.get_param("/uav_name")
    uav_prefix = '/'+uav_name
    print('**********************')
    print(uav_prefix)
    print('**********************')
    ## Instantiate a `MoveGroupCommander`_ object.  This object is an interface
    ## to a planning group (group of joints).  In this tutorial the group is the primary
    ## arm joints in the Panda robot, so we set the group's name to "panda_arm".
    ## If you are using a different robot, change this value to the name of your robot
    ## arm planning group.
    ## This interface can be used to plan and execute motions:
    group_name = "navigation_group"
    move_group = moveit_commander.MoveGroupCommander(group_name)
    #move_group.set_planner_id("RRTConnectkConfigDefault")
    move_group.set_planner_id("RRTConnect")
    move_group.allow_replanning(True)
    ## Create a `DisplayTrajectory`_ ROS publisher which is used to display
    ## trajectories in Rviz:
    topic_name = '/move_group/display_planned_path'
    topic_name = uav_prefix + topic_name
    display_trajectory_publisher = rospy.Publisher(topic_name,
                                                   moveit_msgs.msg.DisplayTrajectory,
                                                   queue_size=20)


    planning_frame = move_group.get_planning_frame()
    print "==== Planning frame: %s" % planning_frame


    group_names = robot.get_group_names()
    print "==== Available Planning Groups:", robot.get_group_names()

    # Misc variables
    self.box_name = ''
    self.robot = robot
    self.scene = scene
    self.move_group = move_group
    self.display_trajectory_publisher = display_trajectory_publisher
    self.planning_frame = planning_frame
    self.group_names = group_names


  def go_to_joint_state(self, joint_cmd):
    move_group = self.move_group
    scene = self.scene

    joint_goal = move_group.get_current_joint_values()

    joint_goal = sensor_msgs.msg.JointState()
    joint_goal.header.frame_id = "world"
    joint_goal.name = ["direction_x", "direction_y", "direction_z"]
    joint_goal.position = joint_cmd
    execute_wait = True
    replanning_mode = False  #[USER INPUT] 
    try:
      path = move_group.plan(joint_goal)
      #execute_wait = True
      print "==== Following the planned path, please wait... "
      current_position = move_group.get_current_joint_values()
      distance = np.linalg.norm(np.subtract(current_position, joint_cmd))
      if replanning_mode == True:
        while distance >= 0.5:
          current_position = move_group.get_current_joint_values()
          distance = np.linalg.norm(np.subtract(current_position, joint_cmd))
          #scene.update()
          path = move_group.plan(joint_goal) #replan
          rospy.loginfo("Still Far Away : I replan !") #notify the replanning
          rospy.sleep(1)#Replanning rate #[USER INPUT] 
          rospy.loginfo("Woke Up!") #Woke up, start again at line 111 if goal not reached
          if distance < 0.5:
            rospy.loginfo("---------------Close enough : I stop replanning--------------")  
            move_group.clear_pose_targets()
      move_group.clear_pose_targets()    


      print "==== Goal achieved!"
    except:
      print "[WARNING] Cannot find any paths."
      execute_wait = False

    #if execute_wait == True:
      
    

    return all_close(joint_goal, joint_goal, 0.01)



def main():
  try:
    print ""
    print "---------------------------------------"
    print "  Welcome to the Moveit Path Planner"
    print "---------------------------------------"
    
    planner = MoveGroupPythonInteface()

    #print "==== Press `Enter` to execute a movement using a set-up goal position [0.0, 0.0, 2.0]."
    #raw_input()
    #planner.go_to_joint_state([0.0, 0.0, 2.0])

    print "==== Press `Enter` to execute a movement using a customized goal position [x, y, z]."
    raw_input()
    print "==== Input goal position (workspace: -50<x<50, -50<y<50, 0.5<z<10):"
    cmd_x = float(raw_input("x = "))
    cmd_y = float(raw_input("y = "))
    cmd_z = float(raw_input("z = "))
    planner.go_to_joint_state([cmd_x, cmd_y, cmd_z])
    
    while True:
      print "==== Press `Enter` to try again or type `q` to quit."
      choice = raw_input()
      if choice == "":
        print "==== Input goal position (workspace: -50<x<50, -50<y<50, 0.5<z<10):"
        cmd_x = float(raw_input("x = "))
        cmd_y = float(raw_input("y = "))
        cmd_z = float(raw_input("z = "))
        planner.go_to_joint_state([cmd_x, cmd_y, cmd_z])
      elif choice == "q":
        break
      else:
        print "==== Invalid input, please press `Enter` or `q`."
    
  except rospy.ROSInterruptException:
    return
  except KeyboardInterrupt:
    return

if __name__ == '__main__':
  main()