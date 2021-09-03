#ifndef DRIVE_BASE_H
#define DRIVE_BASE_H

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose2D.h>
#include <tf2_ros/transform_listener.h>
#include "laser_line_extraction/line_extraction_ros.h"
// need this for the function tf2::getYaw
#include <tf2/utils.h>
#include <tf/transform_listener.h>
#include "laser_line_extraction/modulation.h"

namespace line_extraction
{

class RobotDriver
{

public:

  RobotDriver(ros::NodeHandle &nh);

  bool driveKeyboard();

  bool driveForwardOdom(double distance);


private:
  // node passed from outside (shared with main application)
  ros::NodeHandle nh_;

  // where we publish the Twist
  ros::Publisher cmd_vel_pub_;

  //tf transforms
  tf::TransformListener listener_;

};

} // namespace line_extraction

#endif // DRIVE_BASE_H
