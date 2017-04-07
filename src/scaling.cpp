#include "ros/ros.h"
#include "std_msgs/Float32.h"
#include <DenseInput.h>

ros::Publisher pub;
float scale = 1;

void convertCb(const svo_msgs::DenseInput &msg)
{
	svo_msgs::DenseInput msg_dense;
	msg_dense = msg;
	msg_dense.header.stamp = ros::Time::now();
	msg_dense.pose.position.x *= scale;
	msg_dense.pose.position.y *= scale;
	msg_dense.pose.position.z *= scale;
	msg_dense.min_depth *= scale;
	msg_dense.max_depth *= scale;
	pub.publish(msg_dense);

}

void scaleCb(const std_msgs::Float32 &msg)
{
	scale = msg.data;
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "scale_node");
	ros::NodeHandle n;

	ros::Subscriber sub = n.subscribe("/ORB/DenseInput", 10, convertCb);
	ros::Subscriber scale_sub = n.subscribe("/scaled/scale", 10, scaleCb);
	pub = n.advertise<svo_msgs::DenseInput>("/scaled/DenseInput",1);
	ros::spin();
}



