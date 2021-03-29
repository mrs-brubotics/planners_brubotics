#!/usr/bin/env python
import rospy
import sys
from std_msgs.msg import String
from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry

class subscriber_class:
  def __init__(self):
    self.sub = rospy.Subscriber("/uav1/odometry/odom_gps", Odometry,self.callback)
    self.odo_data = Odometry()

  def callback(self,data):
    self.odo_data = data



def main(args):
  # Define subscriber
  odo = subscriber_class()

  # Define Publisher
  pub = rospy.Publisher('/joint_states', JointState, queue_size=10)

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
        pub.publish(joint_data)
    
    except KeyboardInterrupt:
      print("Shutting down")

    #rospy.sleep(1)
    #rospy.spin()
  
  

if __name__ == '__main__':
    main(sys.argv)