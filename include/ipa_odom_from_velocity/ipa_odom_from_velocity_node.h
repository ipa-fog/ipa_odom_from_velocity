/*
 * Copyright (c) 2018 Fraunhofer Institute for Manufacturing Engineering and Automation (IPA)
 * All rights reserved.
 * Unauthorized copying of this file, via any medium is strictly prohibited.
 * Proprietary and confidential.
 */

#ifndef IPA_ODOM_FROM_VELOCITY_NODE_H
#define IPA_ODOM_FROM_VELOCITY_NODE_H


#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_listener.h>
#include "geometry_msgs/TransformStamped.h"
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <tf2_msgs/TFMessage.h>


class OdomFromVelocityNode
{
public:
	OdomFromVelocityNode(ros::NodeHandle node_handle);

private:
	ros::NodeHandle node_;
	ros::Subscriber cmd_vel_sub_;
	ros::Subscriber tf_sub_;
	ros::Publisher odom_pub_;
	nav_msgs::Odometry odom_;
	bool run_loop_ = true;

	double x_= 0.0;
	double y_ = 0.0;
	double th_ = 0.0;
	double vx_ = 0.0;
    	double vy_ = 0.0;
    	double vth_ = 0.0;
    	geometry_msgs::TransformStamped odom_trans_;
	ros::Time current_time_, last_time_;
	tf::TransformBroadcaster odom_broadcaster_;
	geometry_msgs::Quaternion odom_quat;

 	void tf_callback(const tf2_msgs::TFMessage::ConstPtr& tf); 
	void cmd_callback(const geometry_msgs::Twist::ConstPtr& cmd);

};

#endif // IPA_ODOM_FROM_VELOCITY_NODE_H
