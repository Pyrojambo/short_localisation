#ifndef LOCALISATION_NODE_H
#define LOCALISATION_NODE_H

#include <geometry_msgs/TransformStamped.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>

#include "short_localisation/odometry_fusion.h"

class LocalisationNode {
 public:
  LocalisationNode();
  void fuseOdometry();

 private:
  void encoderOdomCb(const nav_msgs::OdometryConstPtr& odom);
  void gnssOdomCb(const nav_msgs::OdometryConstPtr& odom);
  void publish(const nav_msgs::Odometry& odom);

  ros::NodeHandle nh_;
  ros::Subscriber encoder_odom_sub_;
  ros::Subscriber gnss_odom_sub_;
  ros::Publisher fused_odom_pub_;
  tf2_ros::TransformBroadcaster tf_broadcaster_;

  OdometryFusion odom_fuser;
};

#endif  // LOCALISATION_NODE_H
