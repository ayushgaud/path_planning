#include "ros/ros.h"
#include "pcl_ros/transforms.h"
#include "sensor_msgs/PointCloud2.h"

ros::Publisher pcl_pub;
void pclCb(const sensor_msgs::PointCloud2 &msg)
{
	Eigen::Matrix4f transform;
	sensor_msgs::PointCloud2 out;
	transform << 0,0,1,0,-1,0,0,0,0,-1,0,0,0,0,0,1; //Rotate camera optical to NEU i.e. Z -pi/2 X -pi/2

	pcl_ros::transformPointCloud(transform.matrix(), msg, out);

	pcl_pub.publish(out);
}


int main(int argc, char **argv)
{
	ros::init(argc, argv, "pcl_transform");
	ros::NodeHandle n;

	pcl_pub = n.advertise<sensor_msgs::PointCloud2>("pcl_out", 100);
	ros::Subscriber pcl_sub = n.subscribe("/remode/pointcloud", 100, pclCb);

	ros::spin();

	return 0;
}
