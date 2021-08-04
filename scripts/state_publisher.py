#!/usr/bin/env python
import rospy
import sys
from std_msgs.msg import String
from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry

class subscriber_class:
  def __init__(self):
    uav_name = rospy.get_param("/uav_name")
    uav_prefix = '/'+uav_name
    uav_prefix
    print('**********************')
    print(uav_prefix)
    print('**********************')    
    pos_topic_name = "/odometry/odom_gps"
    pos_topic_name = uav_prefix + pos_topic_name
    self.sub = rospy.Subscriber(pos_topic_name, Odometry,self.callback)
    self.odo_data = Odometry()
    self.uav_name = uav_prefix

  def callback(self,data):
    self.odo_data = data



def main(args):
  # Define subscriber
  odo = subscriber_class()
  joint_topic = '/joint_states'
  joint_topic = odo.uav_name + '/joint_states'
  # Define Publisher
  pub = rospy.Publisher(joint_topic, JointState, queue_size=10)

  joint_data = JointState()
  #odo_data = odo.odo_data
  rospy.init_node('mrs_connection', anonymous=True)
  while True:
    try:
        odo_data = odo.odo_data
        joint_data.header = odo_data.header
        #joint_data.header.frame_id = "geometry_center"
        joint_data.name = ["direction_x", "direction_y", "direction_z"]
        joint_data.position = [odo_data.pose.pose.position.x, odo_data.pose.pose.position.y, odo_data.pose.pose.position.z]
        rospy.loginfo('The Name is : %s',odo.uav_name)
        pub.publish(joint_data)
    
    except KeyboardInterrupt:
      print("Shutting down")

    #rospy.sleep(1)
    #rospy.spin()
  
  

if __name__ == '__main__':
    main(sys.argv)