#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <geometry_msgs/PointStamped.h>

tf::StampedTransform transform;
ros::Publisher transformed_pub;
bool has_transform = false;

void waypointCb(const geometry_msgs::PointStamped::ConstPtr &msg)
{
  if(has_transform)
  {
    tf::Stamped<tf::Point> point_odom;
    geometry_msgs::PointStamped point_base_link;
    tf::pointStampedMsgToTF(*msg, point_odom);
    tf::Stamped<tf::Point> point_stamped;
    point_stamped.setData(transform * point_odom);
    tf::pointStampedTFToMsg(point_stamped, point_base_link);
    transformed_pub.publish(point_base_link);
    // ROS_INFO("Published Pose %d %d %d", point_base_link.point.x, point_base_link.point.y, point_base_link.point.z);
  }
  else
    ROS_ERROR("Transform not yet initialized");
}

int main(int argc, char** argv){
  ros::init(argc, argv, "bebop_tf_listner");

  ros::NodeHandle node;
  ros::Subscriber planner_sub = node.subscribe("/clicked_point",1, waypointCb);
  tf::TransformListener listener;
  transformed_pub = node.advertise<geometry_msgs::PointStamped>("/transformed/clicked_point", 1);
  ros::Rate rate(10.0);
  while (node.ok()){

    try{
      listener.lookupTransform("odom", "base_link", 
       ros::Time(0), transform);
      ROS_INFO("Transform initialized");
      has_transform = true;
      break;
    }
    catch (tf::TransformException &ex) {
      ROS_ERROR("%s",ex.what());
      ros::Duration(1.0).sleep();
      continue;
    }

    rate.sleep();
  }
  ros::spin();  
  return 0;
};