#include "laser_line_extraction/base_controller.h"
#include "laser_line_extraction/utilities.h"


namespace line_extraction {

BaseController::BaseController(ros::NodeHandle *nh, Modulation* modulation):
  nh_(nh),
  tf_listener_(tf_buffer_),
  valid_target_(false),
  current_state_(IDLE),
  modulation_(modulation)
{
  cmd_vel_pub_ = nh_->advertise<geometry_msgs::Twist>("/base_controller/command", 10);

  ros::NodeHandle nh_local("~");

  nh_local.param<double>("p_gain_linear", P_linear_, 2);
  nh_local.param<double>("p_gain_rotation", P_angle_, 2);
}

inline double Clamp(double value, double limit)
{
  limit = std::abs(limit);
  value = std::max( value , -limit );
  value = std::min( value ,  limit );
  return value;
}

BaseController::State BaseController::loop()
{
  if( !valid_target_ )
  {
    return IDLE;
  }

  const double epsilon_dist = 0.02;
  const double epsilon_angle = 0.01;
  const double MAX_LINEAR_VELOCITY = 0.8;
  const double MAX_ANGULAR_VELOCITY = 1.5;

  auto actual_pos = getCurrentPosition();

  double dist_x = target_.x - actual_pos.x;
  double dist_y = target_.y - actual_pos.y;
  double distance = std::sqrt( dist_x*dist_x + dist_y*dist_y);

  double angle_error_to_goal = atan2( dist_y, dist_x) - actual_pos.theta;
  angle_error_to_goal = pi_to_pi(angle_error_to_goal);

  double angle_error_heading = target_.theta - actual_pos.theta;
  angle_error_heading = pi_to_pi(angle_error_heading);

  geometry_msgs::Twist desired_twist;

  // Super simple state machine

  if( current_state_ == INITIAL_ROTATION )
  {
    if( std::abs(angle_error_to_goal) < epsilon_angle )
    {
      // switch to next state
      current_state_ = MOVE_STRAIGHT;
    }
    else {
      double vel = P_angle_ * angle_error_to_goal;
      desired_twist.angular.z = Clamp(vel, MAX_ANGULAR_VELOCITY);
    }
  }

  if( current_state_ == MOVE_STRAIGHT)
  {
    if( distance < epsilon_dist )
    {
      // switch to next state
      current_state_ = FINAL_ROTATION;
    }
    else {
      double lin_vel = P_linear_ * distance;
      desired_twist.linear.x = Clamp(lin_vel, MAX_LINEAR_VELOCITY);

      double ang_vel = P_angle_ * angle_error_to_goal;
      desired_twist.angular.z = Clamp(ang_vel, MAX_ANGULAR_VELOCITY);
    }
  }

  if( current_state_ == FINAL_ROTATION)
  {
    if( std::abs(angle_error_heading) < epsilon_dist )
    {
      // switch to next state
      current_state_ = GOAL_REACHED;
    }
    else {
      double vel =  P_angle_ * angle_error_heading;
      desired_twist.angular.z = Clamp(vel, MAX_ANGULAR_VELOCITY);
    }
  }

  /*Eigen::Vector3d& curr_pos;
  Eigen::VectorXf& curr_speed;

  line_extraction::Modulation _mod(actual_pos, desired_twist_2d);
  _mod.justatestfunction();
*/

  ROS_INFO("State[%d]: Linear error: %f Angular error: %f  Twist linear: %f Twist Angular: %f",
           current_state_, distance, angle_error_to_goal, desired_twist.linear.x, desired_twist.angular.z);

  // TODO: apply modulation here
  // command_twist = modulator.apply(desired_twist);
  //if( modulation_ == nullptr){
    cmd_vel_pub_.publish(desired_twist);
  //}
  //else {
    // Umformen!    Eigen::Vector2f desired_twist_2d << desired_twist.linear, ... ;
    //auto command_twist_2d = modulator_->apply(actual_pos, desired_twist_2d);
   // cmd_vel_pub_.publish( Convert(command_twist_2d) );
  //}

  return current_state_;
}

void BaseController::setGoal(const geometry_msgs::Pose2D &goal)
{
  ROS_INFO("setGoal [x,y,theta]: %f %f %f", goal.x, goal.y, goal.theta);
  auto current_pose = getCurrentPosition();
  ROS_INFO("Current [x,y,theta]: %f %f %f", current_pose.x, current_pose.y, current_pose.theta);
  target_ = goal;
  valid_target_ = true;
  current_state_ = INITIAL_ROTATION;
}

/*
void BaseController::setGoal(double goalx, double goaly, double goalalpha)
{
  auto current_pose = getCurrentPosition();
  /*geometry_msgs::Pose2D &goal;
  goal.x = goalx;
  goal.y = goaly;
  goal.theta = goaltheta;
  target_.x = goalx;
  target_.y = goaly;
  target_.theta = goalalpha;
  valid_target_ = true;
  current_state_ = INITIAL_ROTATION;
}
*/

geometry_msgs::Pose2D BaseController::getCurrentPosition()
{
  geometry_msgs::Pose2D current_pose;

  try{
    geometry_msgs::TransformStamped transform;
    ros::Time now = ros::Time::now();
    tf_buffer_.canTransform("map", "base_link", now, ros::Duration(3.0));
    transform = tf_buffer_.lookupTransform("map", "base_link", now);


    current_pose.x = transform.transform.translation.x;
    current_pose.y = transform.transform.translation.y;
    current_pose.theta = tf2::getYaw(transform.transform.rotation);
  }
  catch (tf2::TransformException ex){
    ROS_ERROR("%s",ex.what());
  }
  return current_pose;
}

} // end namespace
