 /*
 * Copyright 2017 Ayush Gaud 
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0

 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "ros/ros.h"
#include "std_msgs/Float32.h"
#include <nav_msgs/Odometry.h>
#include <DenseInput.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

ros::Publisher pub, scaled_pose_pub;
float scale = 1;
float min_trans = 0.5; //Minimum translation for calculating scale

geometry_msgs::Pose odom_slam, odom_sensors;
tf::Transform offset_slam, offset_sensors, offset_sensors_transformed;

bool init_slam = false;
bool init_sensors = false;
bool init_scale = false;

void transformCb(const svo_msgs::DenseInput &orb_msg)
{
	if(init_scale)
	{
		tf::Transform orb_tf;
		
		svo_msgs::DenseInput msg_dense;
		msg_dense.header.stamp = ros::Time::now();
		msg_dense.header.frame_id = "world";

		msg_dense.image = orb_msg.image;
		tf::poseMsgToTF(orb_msg.pose, orb_tf);
		orb_tf.setOrigin(scale * orb_tf.getOrigin());
		
		tf::Transform offset_slam_scaled;
		offset_slam_scaled = offset_slam;
		offset_slam_scaled.setOrigin(scale * offset_slam_scaled.getOrigin());

		tf::Transform world_to_camera;
		world_to_camera.setBasis(tf::Matrix3x3(0,-1,0,0,0,-1,1,0,0));
		
		// tf::poseMsgToTF(odom_msg->pose.pose, offset_sensors_transformed);
		offset_sensors_transformed = offset_sensors;
		offset_sensors_transformed = offset_sensors_transformed * world_to_camera.inverse();  //Transform the axis to camera convention
		offset_sensors_transformed = world_to_camera * offset_sensors_transformed;	//Transform the points
		
		static tf::TransformBroadcaster br;
		// br.sendTransform(tf::StampedTransform(orb_tf, ros::Time::now(), "world", "ORB"));
		// br.sendTransform(tf::StampedTransform(offset_slam_scaled.inverse() * orb_tf, ros::Time::now(), "world", "ORB_transformed"));
		// br.sendTransform(tf::StampedTransform(offset_sensors_transformed.inverse() * offset_slam_scaled.inverse() * orb_tf, ros::Time::now(), "world", "bebop"));
		br.sendTransform(tf::StampedTransform(world_to_camera.inverse() * offset_sensors_transformed * offset_slam_scaled.inverse() * orb_tf * world_to_camera, ros::Time::now(), "world", "scaled_bebop"));

		tf::poseTFToMsg(offset_sensors_transformed * offset_slam_scaled.inverse() * orb_tf, msg_dense.pose);
		msg_dense.min_depth = orb_msg.min_depth * scale;
		msg_dense.max_depth = orb_msg.max_depth * scale;
		pub.publish(msg_dense);
		geometry_msgs::Pose pose_msg;
		tf::poseTFToMsg(world_to_camera.inverse() * offset_sensors_transformed * offset_slam_scaled.inverse() * orb_tf * world_to_camera, pose_msg);
		scaled_pose_pub.publish(pose_msg);
	}
}

void scaleCb(const std_msgs::Float32 &msg)
{
	scale = msg.data;
	init_scale = true;
}


void callback(const nav_msgs::Odometry::ConstPtr &odom_msg, const svo_msgs::DenseInput::ConstPtr &orb_msg)
{
	// ROS_INFO("Callback");
	if(!init_sensors)
	{
		tf::poseMsgToTF(odom_msg->pose.pose, offset_sensors);
		tf::poseMsgToTF(orb_msg->pose, offset_slam);
		init_sensors = true;
	}

	if(!init_scale)
	{	
		odom_sensors = odom_msg->pose.pose;
		odom_slam = orb_msg->pose;
		float trans_odom = std::sqrt(std::pow(odom_sensors.position.x - offset_sensors.getOrigin().x(), 2) + std::pow(odom_sensors.position.y - offset_sensors.getOrigin().y(), 2) + std::pow(odom_sensors.position.z - offset_sensors.getOrigin().z(), 2));
		float trans_slam = std::sqrt(std::pow(odom_slam.position.x - offset_slam.getOrigin().x(), 2) + std::pow(odom_slam.position.y - offset_slam.getOrigin().y(), 2) + std::pow(odom_slam.position.z - offset_slam.getOrigin().z(), 2));

		if(trans_odom >= min_trans)
		{
			scale = trans_odom / trans_slam;
			init_scale = true;
			std::cout << "Initialized scale: " << scale << std::endl;
		}
	}

}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "scale_node");
	ros::NodeHandle n;
	ros::Subscriber sub = n.subscribe("/ORB/DenseInput", 10, transformCb);
	ros::Subscriber scale_sub = n.subscribe("/scaled/scale", 10, scaleCb);
	// ros::Subscriber odom_sub = n.subscribe("/bebop2/odometry_sensor1/odometry", 10, odomCb);

	message_filters::Subscriber<nav_msgs::Odometry> odom_sub(n, "/bebop2/odometry_sensor1/odometry", 1);
	message_filters::Subscriber<svo_msgs::DenseInput> orb_sub(n, "/ORB/DenseInput", 1);
	// message_filters::Subscriber<sensor_msgs::Image> img_sub(n, "/bebop/image_raw", 1);
	typedef message_filters::sync_policies::ApproximateTime<nav_msgs::Odometry, svo_msgs::DenseInput> MySyncPolicy;

	message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), odom_sub, orb_sub);

	sync.registerCallback(boost::bind(&callback, _1, _2));

	pub = n.advertise<svo_msgs::DenseInput>("/scaled/DenseInput",1);
	scaled_pose_pub = n.advertise<geometry_msgs::Pose>("/scaled_odom",1);
	ros::spin();
}