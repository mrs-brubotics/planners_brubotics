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

    ## Instantiate a `MoveGroupCommander`_ object.  This object is an interface
    ## to a planning group (group of joints).  In this tutorial the group is the primary
    ## arm joints in the Panda robot, so we set the group's name to "panda_arm".
    ## If you are using a different robot, change this value to the name of your robot
    ## arm planning group.
    ## This interface can be used to plan and execute motions:
    group_name = "navigation_group"
    move_group = moveit_commander.MoveGroupCommander(group_name)

    ## Create a `DisplayTrajectory`_ ROS publisher which is used to display
    ## trajectories in Rviz:
    display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                                   moveit_msgs.msg.DisplayTrajectory,
                                                   queue_size=20)

    ## BEGIN_SUB_TUTORIAL basic_info
    ##
    ## Getting Basic Information
    ## ^^^^^^^^^^^^^^^^^^^^^^^^^
    # We can get the name of the reference frame for this robot:
    planning_frame = move_group.get_planning_frame()
    print "==== Planning frame: %s" % planning_frame

    # We can also print the name of the end-effector link for this group:
    #eef_link = move_group.get_end_effector_link()
    #print "==== End effector link: %s" % eef_link

    # We can get a list of all the groups in the robot:
    group_names = robot.get_group_names()
    print "==== Available Planning Groups:", robot.get_group_names()

    # Sometimes for debugging it is useful to print the entire state of the
    # robot:
    #print "============ Printing robot state"
    #print robot.get_current_state()
    #print ""
    ## END_SUB_TUTORIAL

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

    joint_goal = move_group.get_current_joint_values()

    

    joint_goal = sensor_msgs.msg.JointState()
    joint_goal.header.frame_id = "world"
    joint_goal.name = ["direction_x", "direction_y", "direction_z"]
    #joint_goal.position = [10.0, 0.0, 2.0]
    joint_goal.position = joint_cmd
    #joint_goal.velocity = [3.0, 3.0, 3.0]
    #joint_goal.effort = [3.0, 3.0, 3.0]
    execute_wait = True
    try:
      path = move_group.plan(joint_goal)
      #execute_wait = True
    except:
      print "[WARNING] Cannot find any paths."
      execute_wait = False
      #warnings.warn("Cannot find a path.")
      #joint_cmd = move_group.get_current_joint_values()


    # Calling ``stop()`` ensures that there is no residual movement
    #move_group.stop()
    if execute_wait == True:
      print "==== Following the planned path, please wait... "
      current_position = move_group.get_current_joint_values()
      distance = np.linalg.norm(np.subtract(current_position, joint_cmd))

      while distance >= 0.1:
        current_position = move_group.get_current_joint_values()
        distance = np.linalg.norm(np.subtract(current_position, joint_cmd))
        rospy.sleep(2)

      print "==== Goal achieved!"
    

    return all_close(joint_goal, joint_goal, 0.01)



def main():
  try:
    print ""
    print "---------------------------------------"
    print "  Welcome to the Moveit Path Planner"
    print "---------------------------------------"

    planner = MoveGroupPythonInteface()

    print "==== Press `Enter` to execute a movement using a set-up goal position [0.0, 0.0, 2.0]."
    raw_input()
    planner.go_to_joint_state([0.0, 0.0, 2.0])

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