/**
WoLF: WoLF: Whole-body Locomotion Framework for quadruped robots (c) by Gennaro Raiola

WoLF is licensed under a license Creative Commons Attribution-NonCommercial-NoDerivatives 4.0 International License.

You should have received a copy of the license along with this
work. If not, see <http://creativecommons.org/licenses/by-nc-nd/4.0/>.
**/

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
