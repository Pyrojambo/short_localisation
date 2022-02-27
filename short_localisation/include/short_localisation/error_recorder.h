#ifndef ERROR_RECORDER_H
#define ERROR_RECORDER_H

#include <nav_msgs/Odometry.h>
#include <ros/package.h>
#include <ros/ros.h>
#include <tf2/utils.h>

#include "short_localisation/csv_helper.h"

class ErrorRecorder {
 public:
  ErrorRecorder();
  void recorderError();

 private:
  void truthOdomCb(const nav_msgs::OdometryConstPtr& odom);
  void estimateOdomCb(const nav_msgs::OdometryConstPtr& odom);
  bool getInitialValues();
  void calculateError();

  ros::NodeHandle nh_;
  ros::Subscriber truth_odom_sub_;
  ros::Subscriber estimate_odom_sub_;

  nav_msgs::Odometry estimate_odom_, truth_odom_;

  CsvHelper csv_helper;
};
#endif  // ERROR_RECORDER_H
