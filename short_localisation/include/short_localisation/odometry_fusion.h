#ifndef ODOMETRY_FUSION_H
#define ODOMETRY_FUSION_H

#include <nav_msgs/Odometry.h>
#include <tf2/utils.h>

#include <deque>

class OdometryFusion {
 public:
  OdometryFusion();
  void setInitialOdom(const nav_msgs::OdometryConstPtr &initial_odom);
  void forwardEncoderOdom(const nav_msgs::OdometryConstPtr &encoder_odom);
  void forwardGnssOdom(const nav_msgs::OdometryConstPtr &gnss_odom);
  nav_msgs::Odometry getFusedOdom();

 private:
  nav_msgs::Odometry encoderOdomChange();

  nav_msgs::Odometry gnss_odom_, fused_odom_;
  std::deque<nav_msgs::Odometry> encoder_odom_deque_;
  const uint encoder_queue_length_;
};

#endif  // ODOMETRY_FUSION_H
