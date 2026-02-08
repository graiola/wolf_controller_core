/**
WoLF: WoLF: Whole-body Locomotion Framework for quadruped robots (c) by Gennaro Raiola

WoLF is licensed under a license Creative Commons Attribution-NonCommercial-NoDerivatives 4.0 International License.

You should have received a copy of the license along with this
work. If not, see <http://creativecommons.org/licenses/by-nc-nd/4.0/>.
**/

#ifndef WPG_COM_PLANNER_H
#define WPG_COM_PLANNER_H

#include <memory>
#include <Eigen/Core>
#include <wolf_controller_utils/filters.h>
#include <wolf_controller_core/state_estimator.h>
#include <wolf_controller_core/terrain_estimator.h>
#include <wolf_controller_core/wpg/gait_generator.h>
#include <wolf_wbid/core/quadruped_robot.h>

namespace wolf_controller
{

class ComPlanner
{

public:

  const std::string CLASS_NAME = "ComPlanner";

  /**
   * @brief Shared pointer to ComPlanner
   */
  typedef std::shared_ptr<ComPlanner> Ptr;

  /**
   * @brief Shared pointer to const ComPlanner
   */
  typedef std::shared_ptr<const ComPlanner> ConstPtr;

  ComPlanner(wolf_wbid::QuadrupedRobot::Ptr robot_model, GaitGenerator::Ptr gait_generator, TerrainEstimator::Ptr terrain_estimator);

  void update(const double& dt, const Eigen::Vector3d& base_velocity, const double& base_height);

  const Eigen::Vector3d& getComVelocity() const;

  const Eigen::Vector3d& getComPosition() const;

  const Eigen::Vector2d& getCapturePoint() const;

  void reset();

  void resetVelocity();

  void resetPosition();

private:

  void computeSupportPolygonCenter();
  void computeComPositionReference(const double& dt, const double& base_height);
  void computeComVelocityReference(const double& dt, const Eigen::Vector3d& base_velocity);

  wolf_wbid::QuadrupedRobot::Ptr robot_model_;
  GaitGenerator::Ptr gait_generator_;
  TerrainEstimator::Ptr terrain_estimator_;
  Eigen::Vector3d com_velocity_ref_;
  Eigen::Vector3d com_position_ref_;
  Eigen::Vector3d base_velocity_;
  Eigen::Vector3d support_polygon_center_;
  std::vector<Eigen::Vector3d> support_polygon_edges_;

  wolf_controller_utils::SecondOrderFilter<Eigen::Vector3d> com_position_ref_filter_;
  wolf_controller_utils::SecondOrderFilter<Eigen::Vector3d> com_velocity_ref_filter_;

  Eigen::Vector2d dcm_target_;
  Eigen::Vector2d dcm_nominal_;
  Eigen::Vector3d com_position_;
  Eigen::Vector3d com_velocity_;

};


} // namespace

#endif
