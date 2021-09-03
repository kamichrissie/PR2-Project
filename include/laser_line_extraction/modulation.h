#ifndef MODULATION_H
#define MODULATION_H

#include <vector>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/Twist.h>
#include <laser_line_extraction/ellipse.h>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>
#include <cmath>
#include <ros/ros.h>
#include <fstream>
#include <sstream>

#include <tf/tf.h>
#include "tf/transform_datatypes.h"
#include <tf_conversions/tf_eigen.h>
#include <eigen_conversions/eigen_msg.h>
#include <boost/bind.hpp>


#include <boost/shared_ptr.hpp>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/planning_scene/planning_scene.h>
#include <boost/shared_ptr.hpp>
#include <boost/function.hpp>
#include <boost/asio.hpp>
#include <boost/system/error_code.hpp>


struct Twist2D
{
  double linear_speed;
  double angular_speed;
};

inline geometry_msgs::Twist Convert(const Twist2D& in)
{
  geometry_msgs::Twist out;
  out.linear.x = in.linear_speed;
  out.angular.z = in.angular_speed;
  return out;
}

inline Twist2D Convert(const geometry_msgs::Twist& in)
{
  Twist2D out;
  out.linear_speed = in.linear.x;
  out.angular_speed = in.angular.z;
  return out;
}


namespace line_extraction
{

  class Ellipse;

  class Modulation
  {
  private:
    std::vector<line_extraction::Ellipse> ellipses_;
    Eigen::Vector3d position_;
    //Eigen::VectorXd gripper_position_;
    //Eigen::Vector2d virtual_atractor_;
    //double gripper_yaw_speed_;
    Eigen::VectorXf speed_;
    //bool collision_ = false;
    //bool do_ir_modulation_ = true;
    int first_ellipse_ = 0;

    std::vector<double> lambda_;
    std::vector<double> gamma_;
    std::vector<double> real_gamma_;
    double gamma_alpha_;
    std::vector<std::vector<double> > xi_wave_;

    void computeXiWave();
    void computeGamma();
    void computeGammaAlpha(int ellipseNr);
    double computeWeight(int k);
    std::vector<double> computeEigenvalue(int k);
    std::vector<double>  computeHyperplane(int k);
    std::vector<std::vector<double> > computeEBase(int k, std::vector<double> normal);
    Eigen::MatrixXf assembleD_k(int k);
    Eigen::MatrixXf assembleE_k(int k);



  public:

    std::vector<line_extraction::Ellipse>& getEllipses();
    void setEllipses(const std::vector<Ellipse> ellipses );
    void justatestfunction();

    // The purpose of this method is to use return a NEW and modified Twist2D
    virtual Twist2D apply(const geometry_msgs::Pose2D& current_state,
                          const Twist2D& desired) = 0;
  };

} //namespace line_extraction


#endif // MODULATION_H
