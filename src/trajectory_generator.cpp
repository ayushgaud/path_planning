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
#include <thread>
#include "mutex"
#include <Eigen/Dense>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>
#include "visualization_msgs/Marker.h"
#include <geometry_msgs/PointStamped.h>

ros::Publisher vis_pub;
ros::Publisher traj_pub;
ros::Publisher start_pub;

bool istrajpub = true;
bool visualize = true;
bool terminate_prev = false;
bool is_terminated = true;
bool init_trajpub = false;
int id = 0;

ros::Duration planner_delay(2);

Eigen::MatrixXf prev_states = Eigen::MatrixXf::Zero(4,3);
std::mutex states_mutex;

// Function to differentiate a polynomial
Eigen::MatrixXf poly_diff(int n, int k, double t)
{
	Eigen::ArrayXf T = Eigen::ArrayXf::Constant(n, 1, 1);
	Eigen::ArrayXf D = Eigen::ArrayXf::LinSpaced(n, 0, n -1);
	
	for (int j = 0; j < k; ++j)
	{
		for (int i = 0; i < n; ++i)
		{
			T(i) *= D(i);
			if(D(i) > 0)
				D(i) -= 1;
		}
	}

	for (int i = 0; i < n; ++i)
	{
		T(i) *= std::pow(t, D(i));
	}
	return T.matrix().transpose();
}

void trajCb(Eigen::MatrixXf coefficients, Eigen::MatrixXf waypoints)
{
	init_trajpub = true;
	is_terminated = false;
	size_t n = waypoints.rows() - 1;
	Eigen::VectorXf time = Eigen::VectorXf::Zero(n);
	float avg_vel = 0.3;
	// Calculate time based on average velocity for the trajetory
	for (int i = 0; i < n; ++i)
	{
		time(i) = std::sqrt(std::pow(waypoints(i + 1, 0) - waypoints(i, 0), 2) + std::pow(waypoints(i + 1, 1) - waypoints(i, 1), 2) + std::pow(waypoints(i + 1, 2) - waypoints(i, 2), 2));
		time(i) /=avg_vel;
	}

	// Trajectory publish

	trajectory_msgs::MultiDOFJointTrajectory traj_msg;
	trajectory_msgs::MultiDOFJointTrajectoryPoint point_msg;
	traj_msg.header.frame_id = "world";
	traj_msg.joint_names.clear();
	traj_msg.joint_names.push_back("Quadcopter");
	
	int idx = 0;
	int idx_replan = 0;
	geometry_msgs::PointStamped replan_pos;

	visualization_msgs::Marker marker;
	
	ros::Time time_start = ros::Time::now();
	ros::Time time_start_replan = ros::Time::now();
	double curr_time = ros::Time::now().toSec() - time_start.toSec();
	if(istrajpub)
	{	
		while(idx < n)
		{
			if(terminate_prev)
			{
				is_terminated = true;
				break;
			}
			curr_time = ros::Time::now().toSec() - time_start.toSec();

			while(true)
			{
				if(curr_time > time(idx))
				{
					idx++;
					time_start = ros::Time::now();
					break;
				}
				else
					break;
			}
			while(true)
			{
				if(ros::Time::now().toSec() - time_start_replan.toSec() + planner_delay.toSec() > time(idx_replan))
				{
					idx_replan++;
					time_start_replan = ros::Time::now();
					break;
				}
				else
					break;
			}
			if(idx_replan <= n)
			{	
				replan_pos.point.x = waypoints(n, 0);
				replan_pos.point.y = waypoints(n, 1);
				replan_pos.point.z = waypoints(n, 2);

				start_pub.publish(replan_pos);
			}
			else
			{
				Eigen::MatrixXf replan_position = poly_diff(8, 0, (ros::Time::now().toSec() - time_start_replan.toSec()  + planner_delay.toSec())/time(idx_replan)) * coefficients.block(8*idx_replan, 0, 8, 3);
				replan_pos.point.x = replan_position(0);
				replan_pos.point.y = replan_position(1);
				replan_pos.point.z = replan_position(2);

				start_pub.publish(replan_pos);
			}
			// Termination criterion
			if(idx >= n)
				break;
			curr_time = ros::Time::now().toSec() - time_start.toSec();
			traj_msg.points.clear();

			Eigen::MatrixXf position = poly_diff(8, 0, curr_time/time(idx)) * coefficients.block(8*idx, 0, 8, 3);
			// Eigen::MatrixXf velocity = poly_diff(8, 1, curr_time) * coefficients.block(8*idx, 0, 8, 3);
			// Eigen::MatrixXf acceleration = poly_diff(8, 2, curr_time) * coefficients.block(8*idx, 0, 8, 3);
			
			std::lock_guard<std::mutex> lock(states_mutex);
			// Store states for replanning
			for(int j = 0; j < 4; j++)
				prev_states.block(j, 0, 1, 3) = poly_diff(8, j, curr_time/time(idx)) * coefficients.block(8*idx, 0, 8, 3);

			traj_msg.header.stamp = ros::Time::now();
			point_msg.time_from_start.fromSec(ros::Time::now().toSec());
			point_msg.transforms.resize(1);
			// point_msg.velocities.resize(1);
			// point_msg.accelerations.resize(1);

			point_msg.transforms[0].translation.x = position(0);
			point_msg.transforms[0].translation.y = position(1);
			point_msg.transforms[0].translation.z = position(2);
			std::cout << idx << ": Trajectory: " << position << std::endl;

			traj_msg.points.push_back(point_msg);
			traj_pub.publish(traj_msg);

			// Markers for visualization
			if(visualize)
			{
				marker.header.frame_id = "world";
				marker.header.stamp = ros::Time();
				marker.ns = "Jerk";
				marker.id = id++;
				marker.type = visualization_msgs::Marker::CUBE;
				marker.action = visualization_msgs::Marker::ADD;

				marker.pose.position.x = position(0);
				marker.pose.position.y = position(1);
				marker.pose.position.z = position(2);

				marker.scale.x = 0.1;
				marker.scale.y = 0.1;
				marker.scale.z = 0.1;
				marker.color.a = 1.0;
				marker.color.r = 1.0;
				marker.color.g = 0.0;
				marker.color.b = 0.0;
				vis_pub.publish(marker);
			}
			ros::Duration(0.1).sleep(); //Publish points at 10 Hz

		}
		if(!terminate_prev)
		{
			std::cout << "Reached Goal" << std::endl;
			id = 0;
			// marker.action = visualization_msgs::Marker::DELETEALL;
			// ros::Duration(5).sleep();
			// vis_pub.publish(marker);
		}
		is_terminated = true;
	}
	
}

void plannerCb(const trajectory_msgs::MultiDOFJointTrajectory& msg)
{
	
	size_t n = msg.points.size() - 1;
	
	Eigen::MatrixXf b = Eigen::MatrixXf::Zero(8*n, 3);
	Eigen::MatrixXf A = Eigen::MatrixXf::Zero(8*n, 8*n);
	Eigen::MatrixXf waypoints = Eigen::MatrixXf::Zero(n + 1, 3);

	// Copy waypoints from message
	for (int i = 0; i <= n; ++i)
	{
		waypoints(i, 0) = msg.points[i].transforms[0].translation.x;
		waypoints(i, 1) = msg.points[i].transforms[0].translation.y;
		waypoints(i, 2) = msg.points[i].transforms[0].translation.z;
	}

	// Make b such that it's in the form Ax = b 
	for(int i = 0; i < n; i++)
	{
		b(i, 0) = waypoints(i, 0);
		b(i, 1) = waypoints(i, 1);
		b(i, 2) = waypoints(i, 2);

		b(i + n, 0) = waypoints(i + 1, 0);
		b(i + n, 1) = waypoints(i + 1, 1);
		b(i + n, 2) = waypoints(i + 1, 2);;

	}

	size_t row = 0;

	// Position constraints at t = 0
	for(int i = 0; i < n; i++)
	{
		A.block(row, 8*i, 1, 8) = poly_diff(8, 0, 0);
		row++;
	}

	// Position constraints at t =1
	for(int i = 0; i < n; i++)
	{
		A.block(row, 8*i, 1, 8) = poly_diff(8, 0, 1);
		row++;
	}

	// Velocity acceleration and jerk constraints at t = 0 for first point
	for (int i = 0; i < 3; i++)
	{
		A.block(row, 0, 1, 8) = poly_diff(8, i + 1, 0);
		if(init_trajpub)
		{
			b.block(0, 0, 1, 3) = prev_states.block(0, 0, 1, 3);
			b.block(row, 0, 1, 3) = prev_states.block(1 + i, 0, 1, 3);
		}
		row++;
	}

	// Velocity acceleration and jerk constraints at t = 1 for last point
	for (int i = 0; i < 3; i++)
	{
		A.block(row, 8*(n -1), 1, 8) = poly_diff(8, i + 1, 1);
		row++;
	}

	// Continuity constraints at intermediate points
	for (int i = 0; i < n - 1; i++)
	{
		for (int k = 0; k < 6; k++)
		{
			A.block(row, 8*i ,1, 8) = poly_diff(8, k + 1, 1);
			A.block(row, 8*i + 8, 1, 8) = -poly_diff(8, k + 1, 0);
			row++;
		}
	}

	Eigen::MatrixXf coefficients = A.colPivHouseholderQr().solve(b); //Fast and reliable
	// std::cout << "coefficients: " << coefficients << std::endl;

	// send trajectory commands
	while(!is_terminated)
	{
		terminate_prev = true;
		ros::Duration(0.05).sleep();
	}
	terminate_prev = false;
	std::thread(trajCb, coefficients, waypoints).detach();
	
	// ros::Duration(3).sleep();
	
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "trajectory_generator");
	ros::NodeHandle n;
	ros::Subscriber planner_sub = n.subscribe("/waypoints",1,plannerCb);
	vis_pub = n.advertise<visualization_msgs::Marker>( "visualization_marker_jerk", 0 );
	traj_pub = n.advertise<trajectory_msgs::MultiDOFJointTrajectory>("/bebop2/command/trajectory",10);
	start_pub = n.advertise<geometry_msgs::PointStamped>("/start/clicked_point", 10);
	ros::spin();
}