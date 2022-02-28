#include "short_localisation/odometry_fusion.h"

OdometryFusion::OdometryFusion() : encoder_queue_length_{50} {}

void OdometryFusion::setInitialOdom(
    const nav_msgs::OdometryConstPtr &initial_odom) {
  fused_odom_ = *initial_odom;

  std::cout << "Initial odom: (" << fused_odom_.pose.pose.position.x << ", "
            << fused_odom_.pose.pose.position.y << ", "
            << tf2::getYaw(fused_odom_.pose.pose.orientation) << ")"
            << std::endl;
}

void OdometryFusion::forwardEncoderOdom(
    const nav_msgs::OdometryConstPtr &encoder_odom) {
  encoder_odom_deque_.push_back(*encoder_odom);

  if (encoder_odom_deque_.size() > encoder_queue_length_) {
    encoder_odom_deque_.pop_front();
  }
}

void OdometryFusion::forwardGnssOdom(
    const nav_msgs::OdometryConstPtr &gnss_odom) {
  gnss_odom_ = *gnss_odom;
}

nav_msgs::Odometry OdometryFusion::getFusedOdom() {
  fused_odom_ = encoderOdomChange();

  fused_odom_.pose.pose.position.x += gnss_odom_.pose.pose.position.x;
  fused_odom_.pose.pose.position.x /= 2.;
  fused_odom_.pose.pose.position.y += gnss_odom_.pose.pose.position.y;
  fused_odom_.pose.pose.position.y /= 2.;

  return fused_odom_;
}

nav_msgs::Odometry OdometryFusion::encoderOdomChange() {
  nav_msgs::Odometry encoderChange{fused_odom_};

  if (!encoder_odom_deque_.empty()) {
    double last_time{encoderChange.header.stamp.toSec()};
    double current_yaw{tf2::getYaw(encoderChange.pose.pose.orientation)};

    for (auto encoder_odom : encoder_odom_deque_) {
      const double delta_t{encoder_odom.header.stamp.toSec() - last_time};
      last_time = encoder_odom.header.stamp.toSec();

      const double displacement{encoder_odom.twist.twist.linear.x * delta_t};
      // polar to cartesian coordinates
      const double delta_x{displacement * cos(current_yaw)};
      const double delta_y{displacement * sin(current_yaw)};
      encoderChange.pose.pose.position.x += delta_x;
      encoderChange.pose.pose.position.y += delta_y;

      const double delta_yaw{encoder_odom.twist.twist.angular.z * delta_t};
      current_yaw += delta_yaw;

      //      std::cout << "Encoder step change: (" << delta_x << ", " <<
      //      delta_y
      //                << ", " << delta_yaw << ") for " << delta_t << "s" <<
      //                std::endl;
    }
    encoder_odom_deque_.clear();

    tf2::Quaternion end_orientation;
    end_orientation.setEuler(0, 0, current_yaw);
    encoderChange.pose.pose.orientation = tf2::toMsg(end_orientation);

    encoderChange.header.stamp = ros::Time(last_time);

    //    std::cout << "Encoder change: ("
    //              << (encoderChange.pose.pose.position.x -
    //                  fused_odom_.pose.pose.position.x)
    //              << ", "
    //              << (encoderChange.pose.pose.position.y -
    //                  fused_odom_.pose.pose.position.y)
    //              << ", "
    //              << (current_yaw -
    //              tf2::getYaw(fused_odom_.pose.pose.orientation))
    //              << ")" << std::endl;
  }

  return encoderChange;
}
