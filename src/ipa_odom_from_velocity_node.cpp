/*
 * Copyright (c) 2018 Fraunhofer Institute for Manufacturing Engineering and Automation (IPA)
 * All rights reserved.
 * Unauthorized copying of this file, via any medium is strictly prohibited.
 * Proprietary and confidential.
 */

#include "ipa_odom_from_velocity/ipa_odom_from_velocity_node.h"

OdomFromVelocityNode::OdomFromVelocityNode(ros::NodeHandle node_handle):
	node_(node_handle)
{
    
    cmd_vel_sub_ = node_.subscribe<geometry_msgs::Twist>("cmd_vel", 50, &OdomFromVelocityNode::cmd_callback, this);
    odom_pub_= node_.advertise<nav_msgs::Odometry>("odom", 50);
    loop_func();
}

void OdomFromVelocityNode::loop_func()
{
	ros::Rate r(50);
  	while(ros::ok())
	{
		odom_quat = tf::createQuaternionMsgFromYaw(th_);

		// publish tf
		odom_trans_.header.stamp = current_time_;
		odom_trans_.header.frame_id = "odom_combined";
		odom_trans_.child_frame_id = "base_footprint";

		odom_trans_.transform.translation.x = x_;
		odom_trans_.transform.translation.y = y_;
		odom_trans_.transform.translation.z = 0.0;
		odom_trans_.transform.rotation = odom_quat;
		ROS_INFO("x2 %f", x_);
		//send the transform
		odom_broadcaster_.sendTransform(odom_trans_);

		//next, we'll publish the odometry message over ROS
		odom_.header.stamp = current_time_;
		odom_.header.frame_id = "odom_combined";
		odom_.child_frame_id = "base_footprint";

		//set the position
		odom_.pose.pose.position.x = x_;
		odom_.pose.pose.position.y = y_;
		odom_.pose.pose.position.z = 0.0;
		odom_.pose.pose.orientation = odom_quat;

		//set the velocity
		odom_.twist.twist.linear.x = vx_;
		odom_.twist.twist.linear.y = vy_;
		odom_.twist.twist.angular.z = vth_;
		last_time_ = current_time_;

		odom_pub_.publish(odom_);

 		r.sleep();
	}

}



void OdomFromVelocityNode::cmd_callback(const geometry_msgs::Twist::ConstPtr& cmd)
{

    vx_ = cmd->linear.x;
    vy_ = cmd->linear.y;
    vth_ = cmd->angular.z;

    current_time_ = ros::Time::now();

    //compute odometry in a typical way given the velocities of the robot
    double dt = (current_time_ - last_time_).toSec();
    double delta_x = (vx_ * cos(th_) - vy_ * sin(th_)) * dt;
    double delta_y = (vx_ * sin(th_) + vy_ * cos(th_)) * dt;
    double delta_th = vth_ * dt;

    x_ += delta_x;
    y_ += delta_y;
    th_ += delta_th;

    ROS_INFO("x3 %f", x_);
}



int main(int argc, char** argv)
{
    ros::init(argc, argv, "ipa_odom_from_velocity_node");

    ros::NodeHandle node_handle;
    OdomFromVelocityNode ipa_odom_from_velocity_node(node_handle);

    ros::spin();
    return 0;
}
