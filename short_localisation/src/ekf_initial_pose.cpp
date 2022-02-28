#include "short_localisation/ekf_initial_pose.h"

EkfInitialPose::EkfInitialPose() {
  initial_pose_client =
      nh_.serviceClient<robot_localization::SetPose>("/set_pose");
}

void EkfInitialPose::setInitialPose() {
  if (!initial_pose_client.waitForExistence(ros::Duration{10})) {
    ROS_ERROR("Initial ekf pose client does not exist");
    return;
  }

  nav_msgs::OdometryConstPtr initial_odom_ptr =
      ros::topic::waitForMessage<nav_msgs::Odometry>("sensors/gnss/odom",
                                                     ros::Duration{10});
  if (initial_odom_ptr == nullptr) {
    ROS_ERROR("Failed to get odometry to set initial ekf pose");
    return;
  }

  robot_localization::SetPose initial_pose;
  initial_pose.request.pose.header = initial_odom_ptr->header;
  initial_pose.request.pose.pose = initial_odom_ptr->pose;

  if (!initial_pose_client.call(initial_pose)) {
    ROS_ERROR("Failed to set initial ekf pose");
  }
}

int main(int argc, char *argv[]) {
  ros::init(argc, argv, "ekf_initial_pose");
  EkfInitialPose ekf_initial_pose;
  ekf_initial_pose.setInitialPose();
  return 0;
}
