#include "short_localisation/odometry_fusion.h"

OdometryFusion::OdometryFusion() {}

void OdometryFusion::forwardEncoderOdom(
    const nav_msgs::OdometryConstPtr &encoder_odom) {
  encoder_odom_ = *encoder_odom;
}

void OdometryFusion::forwardGnssOdom(
    const nav_msgs::OdometryConstPtr &gnss_odom) {
  gnss_odom_ = *gnss_odom;
}

nav_msgs::Odometry OdometryFusion::getFusedOdom() {
  fused_odom_ = encoder_odom_;

  fused_odom_.pose.pose.position.x =
      (encoder_odom_.pose.pose.position.x + gnss_odom_.pose.pose.position.x) /
      2.;
  fused_odom_.pose.pose.position.y =
      (encoder_odom_.pose.pose.position.y + gnss_odom_.pose.pose.position.y) /
      2.;

  return fused_odom_;
}
