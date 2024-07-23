/**
 * @file foot_trajectory_ellipse.cpp
 * @author Gennaro Raiola
 * @date 12 June, 2019
 * @brief This file contains the ellipse implementation of the foot trajectory
 */

#include <wolf_controller_core/wpg/foot_trajectory_ellipse.h>

using namespace wolf_controller;

Ellipse::Ellipse()
{
  xyz_ = xyz_dot_ = Eigen::Vector3d::Zero();
}

const Eigen::Vector3d& Ellipse::trajectoryFunction(const double& time)
{
  xyz_(0) = length_/2 * (1 - std::cos(M_PI * (swing_frequency_ * time)));
  xyz_(1) = 0.0;
  xyz_(2) = height_ * std::sin(M_PI * (swing_frequency_ * time));

  return xyz_;
}

const Eigen::Vector3d& Ellipse::trajectoryFunctionDot(const double& time)
{

  xyz_dot_(0) = M_PI * swing_frequency_ * length_/2 * std::sin(M_PI * (swing_frequency_ * time));
  xyz_dot_(1) = 0.0;
  xyz_dot_(2) = M_PI * swing_frequency_ * height_ * std::cos(M_PI * (swing_frequency_ * time));

  return xyz_dot_;
}
