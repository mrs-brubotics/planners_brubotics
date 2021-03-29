// The purpose of this node is to subscribe the info from odom topic
// and publish as a tf

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <string.h>
#include <iostream>


using namespace std;
class Listener
{
public:
  std_msgs::Header header;
  std::string child_frame_id;
  geometry_msgs::Pose pose;
  void callback(const nav_msgs::Odometry::ConstPtr& odom);
};


void Listener::callback(const nav_msgs::Odometry::ConstPtr& odom)
{
  //ROS_INFO("I heard: [%s]", msg->data.c_str());
  
  Listener::header = odom->header;
  Listener::child_frame_id = odom->child_frame_id;
  Listener::pose = odom->pose.pose;

  //string s = Listener::header.frame_id;
    //ROS_INFO(listener.header.frame_id);

  //Listener::x = 0.0;
}


int main(int argc, char** argv){
  ros::init(argc, argv, "odometry_subscriber");

  ros::NodeHandle n;

  Listener listener;

  ros::Rate r(1.0);
  ros::Subscriber sub = n.subscribe("/uav1/odometry/odom_gps", 1000, &Listener::callback, &listener);
  tf::TransformBroadcaster odom_broadcaster;
  geometry_msgs::TransformStamped odom_trans;
  while(ros::ok()){
    if (listener.header.frame_id.empty()){

    }
    else {

      ros::Time current_time;
      current_time = ros::Time::now();

      //geometry_msgs::TransformStamped odom_trans;
      odom_trans.header.stamp = current_time;
      odom_trans.header.frame_id = "moveit/origin";
      odom_trans.child_frame_id = "moveit/base_link";

      odom_trans.transform.translation.x = listener.pose.position.x;
      odom_trans.transform.translation.y = listener.pose.position.y;
      odom_trans.transform.translation.z = listener.pose.position.z;
      odom_trans.transform.rotation = listener.pose.orientation;

      //send the transform
      //odom_broadcaster.sendTransform(odom_trans);
      //geometry_msgs::TransformStamped odom_trans;

    }
    ros::spinOnce();
    
    // Publish as a tf tree
    odom_broadcaster.sendTransform(odom_trans);

    


  
  r.sleep();

  }
  

  return 0;

}