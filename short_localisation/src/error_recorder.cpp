#include "short_localisation/error_recorder.h"

ErrorRecorder::ErrorRecorder() {
  truth_odom_sub_ = nh_.subscribe("sensors/odom/ground_truth", 1,
                                  &ErrorRecorder::truthOdomCb, this);
  estimate_odom_sub_ =
      nh_.subscribe("fused_odom", 1, &ErrorRecorder::estimateOdomCb, this);
}

void ErrorRecorder::recorderError() {
  std::string file_path{ros::package::getPath("short_localisation") +
                        "/data/error.csv"};

  if (!getInitialValues()) {
    return;
  }

  ros::Rate loop_rate{10};
  while (nh_.ok()) {
    ros::spinOnce();
    calculateError();
    loop_rate.sleep();
  }
  csv_helper.writeFile(file_path);
}

void ErrorRecorder::truthOdomCb(const nav_msgs::OdometryConstPtr &odom) {
  truth_odom_ = *odom;
}

void ErrorRecorder::estimateOdomCb(const nav_msgs::OdometryConstPtr &odom) {
  estimate_odom_ = *odom;
}

bool ErrorRecorder::getInitialValues() {
  nav_msgs::OdometryConstPtr initial_ground_truth_ptr =
      ros::topic::waitForMessage<nav_msgs::Odometry>(truth_odom_sub_.getTopic(),
                                                     ros::Duration{10});
  if (initial_ground_truth_ptr == nullptr) {
    ROS_ERROR("Failed to get initial odometry from %s. Aborting",
              truth_odom_sub_.getTopic().c_str());
    return false;
  }

  nav_msgs::OdometryConstPtr initial_estimate_ptr =
      ros::topic::waitForMessage<nav_msgs::Odometry>(
          estimate_odom_sub_.getTopic(), ros::Duration{10});
  if (initial_estimate_ptr == nullptr) {
    ROS_ERROR("Failed to get initial odometry from %s. Aborting",
              estimate_odom_sub_.getTopic().c_str());
    return false;
  }
  return true;
}

void ErrorRecorder::calculateError() {
  if (fabs(truth_odom_.header.stamp.toSec() -
           estimate_odom_.header.stamp.toSec()) > 0.1) {
    // time is too far apart to calculate error
    return;
  }

  const double x_error{estimate_odom_.pose.pose.position.x -
                       truth_odom_.pose.pose.position.x};
  const double y_error{estimate_odom_.pose.pose.position.y -
                       truth_odom_.pose.pose.position.y};
  double yaw_error{tf2::getYaw(estimate_odom_.pose.pose.orientation) -
                   tf2::getYaw(truth_odom_.pose.pose.orientation)};

  // put yaw error in range of [-pi, pi)
  yaw_error = fmod(yaw_error + M_PI, 2 * M_PI);
  if (yaw_error < 0) {
    yaw_error += 2 * M_PI;
  }
  yaw_error -= M_PI;

  std::vector<std::string> csv_row;
  csv_row.push_back(std::to_string(x_error));
  csv_row.push_back(std::to_string(y_error));
  csv_row.push_back(std::to_string(yaw_error));

  csv_helper.setNextRow(csv_row, ",");
}

int main(int argc, char *argv[]) {
  ros::init(argc, argv, "error_recorder");

  ErrorRecorder error_recorder;
  error_recorder.recorderError();

  return 0;
}
