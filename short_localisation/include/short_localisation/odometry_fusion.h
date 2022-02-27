#ifndef ODOMETRY_FUSION_H
#define ODOMETRY_FUSION_H

#include <nav_msgs/Odometry.h>

class OdometryFusion {
 public:
  OdometryFusion();
  void forwardEncoderOdom(const nav_msgs::OdometryConstPtr &encoder_odom);
  void forwardGnssOdom(const nav_msgs::OdometryConstPtr &gnss_odom);
  nav_msgs::Odometry getFusedOdom();

 private:
  nav_msgs::Odometry encoder_odom_, gnss_odom_, fused_odom_;
};

#endif  // ODOMETRY_FUSION_H
