/**
 * @file com_planner.cpp
 * @brief DCM-based COM reference planner for quadruped locomotion
 */

#include <wolf_controller_core/wpg/com_planner.h>

using namespace wolf_wbid;

namespace wolf_controller {

ComPlanner::ComPlanner(QuadrupedRobot::Ptr robot_model,
                       FootholdsPlanner::Ptr foothold_planner,
                       TerrainEstimator::Ptr terrain_estimator)
{
  robot_model_ = robot_model;
  foothold_planner_ = foothold_planner;
  terrain_estimator_ = terrain_estimator;

  com_velocity_ref_.setZero();
  com_position_ref_.setZero();

  support_polygon_edges_.resize(N_LEGS);
  update_ = true;

  com_position_ref_filter_.setTimeStep(_period);
  com_velocity_ref_filter_.setTimeStep(_period);
  com_position_ref_filter_.setOmega(2.0 * M_PI * 1.0/foothold_planner_->getCycleTime());
  com_velocity_ref_filter_.setOmega(2.0 * M_PI * 1.0/foothold_planner_->getCycleTime());

  computeComPositionReference(_period);
}

void ComPlanner::computeSupportPolygonCenter()
{
  auto foot_positions = robot_model_->getFeetPositionInWorld();
  auto foot_names = foothold_planner_->getFootNames();
  support_polygon_center_.setZero();
  for (unsigned int i = 0; i < foot_names.size(); i++)
  {
    support_polygon_center_ += foot_positions[foot_names[i]];
    support_polygon_edges_[i] = foot_positions[foot_names[i]];
  }
  support_polygon_center_ /= N_LEGS;
}

void ComPlanner::computeComVelocityReference(const double &dt)
{
  Eigen::Vector3d com_pos, com_vel;
  robot_model_->getCOM(com_pos);
  robot_model_->getCOMVelocity(com_vel);

  constexpr double min_height = 0.05;
  const double z = std::max(com_pos.z(), min_height);
  const double omega = std::sqrt(GRAVITY / z);

  // 1. Get current ICP target (from push recovery)
  Eigen::Vector2d dcm_target = foothold_planner_->getPushRecovery()->getCapturePoint();

  // 2. Compute nominal forward DCM based on velocity command
  Eigen::Vector2d v_cmd_xy;
  v_cmd_xy.x() = foothold_planner_->getBaseLinearVelocityCmdX();
  v_cmd_xy.y() = foothold_planner_->getBaseLinearVelocityCmdY();

  Eigen::Vector2d dcm_nominal = com_pos.head<2>() + v_cmd_xy / omega;

  // 3. Blend ICP and nominal
  double alpha = 0.5; // blending factor between push recovery and feedforward
  dcm_target = (1.0 - alpha) * dcm_target + alpha * dcm_nominal;

  // 4. Compute velocity reference from DCM target
  Eigen::Vector2d com_vel_ref_xy = omega * (dcm_target - com_pos.head<2>());
  com_velocity_ref_.head<2>() = com_vel_ref_xy;
  com_velocity_ref_.z() = 0.0;
}

void ComPlanner::computeComPositionReference(const double &dt)
{
  com_position_ref_.head<2>() += com_velocity_ref_.head<2>() * dt;
  com_position_ref_.z() = foothold_planner_->getBaseHeight();
}

void ComPlanner::update(const double &dt)
{
  com_position_ref_filter_.setTimeStep(dt);
  com_velocity_ref_filter_.setTimeStep(dt);
  com_position_ref_filter_.setOmega(2.0 * M_PI * 1.0/foothold_planner_->getCycleTime());
  com_velocity_ref_filter_.setOmega(2.0 * M_PI * 1.0/foothold_planner_->getCycleTime());

  computeComVelocityReference(dt);
  computeComPositionReference(dt);

  com_position_ref_ = com_position_ref_filter_.process(com_position_ref_);
  com_velocity_ref_ = com_velocity_ref_filter_.process(com_velocity_ref_);
}

const Eigen::Vector3d &ComPlanner::getComVelocity() const
{
  return com_velocity_ref_;
}

const Eigen::Vector3d &ComPlanner::getComPosition() const
{
  return com_position_ref_;
}

void ComPlanner::reset()
{
  resetPosition();
  resetVelocity();
}

void ComPlanner::resetPosition()
{
  robot_model_->getCOM(com_position_ref_);
  com_position_ref_filter_.reset(com_position_ref_);
}

void ComPlanner::resetVelocity()
{
  com_velocity_ref_.setZero();
  com_velocity_ref_filter_.reset(com_velocity_ref_);
}

} // namespace wolf_controller
