/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) Gennaro Raiola
 */

#ifndef WPG_FOOT_TRAJECTORY_ELLIPSE_H
#define WPG_FOOT_TRAJECTORY_ELLIPSE_H

#include <wolf_controller_core/wpg/foot_trajectory_interface.h>

namespace wolf_controller
{

class Ellipse : public TrajectoryInterface
{

public:

  const std::string CLASS_NAME = "Ellipse";

  Ellipse();

protected:

  virtual const Eigen::Vector3d& trajectoryFunction(const double& time);

  virtual const Eigen::Vector3d& trajectoryFunctionDot(const double& time);

private:

  Eigen::Vector3d xyz_;
  Eigen::Vector3d xyz_dot_;

};

} // namespace

#endif
