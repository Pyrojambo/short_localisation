#ifndef EKF_INITIAL_POSE_H
#define EKF_INITIAL_POSE_H

#include <nav_msgs/Odometry.h>
#include <robot_localization/SetPose.h>
#include <ros/ros.h>

class EkfInitialPose
{
 public:
  EkfInitialPose();
  void setInitialPose();
  
 private:
  ros::NodeHandle nh_;
  ros::ServiceClient initial_pose_client;
};

#endif  // EKF_INITIAL_POSE_H
