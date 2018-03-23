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
    odom_pub_ = node_.advertise<nav_msgs::Odometry>("odom", 50);

}

void OdomFromVelocityNode::cmd_callback(const geometry_msgs::Twist::ConstPtr& cmd)
{

    double vx = cmd->linear.x;
    double vy = cmd->linear.y;
    double vth = cmd->angular.z;
    ROS_INFO("x %f, y %f , t %f ", vx, vy, vth);

    current_time_ = ros::Time::now();
    //compute odometry in a typical way given the velocities of the robot
    double dt = (current_time_ - last_time_).toSec();
    double delta_x = (vx * cos(th_) - vy * sin(th_)) * dt;
    double delta_y = (vx * sin(th_) + vy * cos(th_)) * dt;
    double delta_th = vth * dt;

    x_ += delta_x;
    y_ += delta_y;
    th_ += delta_th;

    //since all odometry is 6DOF we'll need a quaternion created from yaw
    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(th_);

    //first, we'll publish the transform over tf
    geometry_msgs::TransformStamped odom_trans;
    odom_trans.header.stamp = current_time_;
    odom_trans.header.frame_id = "odom_combined";
    odom_trans.child_frame_id = "base_footprint";

    odom_trans.transform.translation.x = x_;
    odom_trans.transform.translation.y = y_;
    odom_trans.transform.translation.z = 0.0;
    odom_trans.transform.rotation = odom_quat;

    //send the transform
    odom_broadcaster_.sendTransform(odom_trans);

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
    odom_.twist.twist.linear.x = vx;
    odom_.twist.twist.linear.y = vy;
    odom_.twist.twist.angular.z = vth;

    //publish the message
    odom_pub_.publish(odom_);

    last_time_ = current_time_;

}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "ipa_odom_from_velocity_node");

    ros::NodeHandle node_handle;
    OdomFromVelocityNode ipa_odom_from_velocity_node(node_handle);

    ROS_INFO("Node is spinning...");
    ros::spin();

    return 0;
}
