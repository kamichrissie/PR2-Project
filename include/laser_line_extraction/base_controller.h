#ifndef BASE_CONTROLLER_H
#define BASE_CONTROLLER_H

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose2D.h>
#include <tf2_ros/transform_listener.h>
#include "laser_line_extraction/line_extraction_ros.h"

// need this for the function tf2::getYaw
#include <tf2/utils.h>
#include "laser_line_extraction/modulation.h"

namespace line_extraction
{

class BaseController
{

public:
  enum State{
    GOAL_REACHED,
    INITIAL_ROTATION,
    MOVE_STRAIGHT,
    FINAL_ROTATION,
    IDLE
  };

  BaseController(ros::NodeHandle* nh, Modulation* modulation = nullptr);

  State loop();

  void setGoal(const geometry_msgs::Pose2D& goal);
  //void setGoal(double goalx, double goaly, double goalalpha);

private:
  // node passed from outside (shared with main application)
  ros::NodeHandle* nh_;

  // where we publish the Twist
  ros::Publisher cmd_vel_pub_;

  // the tf listener will listen to the transform map -> odom
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  geometry_msgs::Pose2D target_;


  bool valid_target_;

  // proportional controller for linear motion
  double P_linear_;
  // proportional controller for rotation
  double P_angle_;

  geometry_msgs::Pose2D getCurrentPosition();

  State current_state_;

  Modulation* modulation_;

};

} // namespace line_extraction

#endif
