#include "short_localisation/localisation_node.h"

LocalisationNode::LocalisationNode() {
  encoder_odom_sub_ =
      nh_.subscribe("sensors/odom", 1, &LocalisationNode::encoderOdomCb, this);
  gnss_odom_sub_ = nh_.subscribe("sensors/gnss/odom", 1,
                                 &LocalisationNode::gnssOdomCb, this);

  fused_odom_pub_ = nh_.advertise<nav_msgs::Odometry>("fused_odom", 1);
}

void LocalisationNode::fuseOdometry() {
  ros::Rate loop_rate{50};

  while (nh_.ok()) {
    ros::spinOnce();
    loop_rate.sleep();
  }
}

void LocalisationNode::publish(const nav_msgs::Odometry &odom) {
  tf2::Quaternion odom_quaternion{
      odom.pose.pose.orientation.x, odom.pose.pose.orientation.y,
      odom.pose.pose.orientation.z, odom.pose.pose.orientation.w};
  if (fabs(odom_quaternion.length() - 1) > 1e-6) {
    // if length != 1 then it is not a normalised quaternion and as such can't
    // be used
    ROS_WARN_THROTTLE(
        5,
        "Invalid odometry quaternion (%.3f, %.3f, %.3f, %.3f). Not "
        "publishing anything",
        odom_quaternion.x(), odom_quaternion.y(), odom_quaternion.z(),
        odom_quaternion.w());
    return;
  }

  fused_odom_pub_.publish(odom);

  geometry_msgs::TransformStamped odom_to_base_tf;
  odom_to_base_tf.header.stamp = odom.header.stamp;
  odom_to_base_tf.header.frame_id = "odom";
  odom_to_base_tf.child_frame_id = "base_link";
  odom_to_base_tf.transform.translation.x = odom.pose.pose.position.x;
  odom_to_base_tf.transform.translation.y = odom.pose.pose.position.y;
  odom_to_base_tf.transform.translation.z = 0;
  odom_to_base_tf.transform.rotation = odom.pose.pose.orientation;

  tf_broadcaster_.sendTransform(odom_to_base_tf);
}

void LocalisationNode::encoderOdomCb(const nav_msgs::OdometryConstPtr &odom) {
  odom_fuser.forwardEncoderOdom(odom);
}

void LocalisationNode::gnssOdomCb(const nav_msgs::OdometryConstPtr &odom) {
  odom_fuser.forwardGnssOdom(odom);
  
  // gnss is the slower input source so use that as the output trigger
  nav_msgs::Odometry fused_odom{odom_fuser.getFusedOdom()};
  publish(fused_odom);
}

int main(int argc, char *argv[]) {
  ros::init(argc, argv, "custom_localisation");
  LocalisationNode localisation_node;
  localisation_node.fuseOdometry();
  return 0;
}
