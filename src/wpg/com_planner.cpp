/**
 * @file com_planner.cpp
 * @author Gennaro Raiola
 * @date 1 November, 2021
 * @brief This file contains the ComPlanner used to generate the com position and velocity references
 */

#include <wolf_controller_core/wpg/com_planner.h>

using namespace rt_logger;
using namespace wolf_wbid;

namespace wolf_controller {

ComPlanner::ComPlanner(QuadrupedRobot::Ptr robot_model, FootholdsPlanner::Ptr foothold_planner, TerrainEstimator::Ptr terrain_estimator)
{

  robot_model_ = robot_model;
  foothold_planner_ = foothold_planner;
  terrain_estimator_ = terrain_estimator;

  com_velocity_ref_.setZero();
  com_position_ref_.setZero();

  support_polygon_edges_.resize(N_LEGS);

  update_ = true;

  // Initialize filters
  filters_cutoff_freq_ = 10.0; // hz
  com_position_ref_filter_.setTimeStep(_period);
  com_velocity_ref_filter_.setTimeStep(_period);
  com_position_ref_filter_.setOmega(2.0*M_PI*filters_cutoff_freq_);
  com_velocity_ref_filter_.setOmega(2.0*M_PI*filters_cutoff_freq_);

  computeComPositionReference(_period);
}

void ComPlanner::computeSupportPolygonCenter()
{
  // Note: the com position reference has to be defined wrt world because
  // the com task is wrt world
  auto foot_positions = robot_model_->getFeetPositionInWorld();
  auto foot_names = foothold_planner_->getFootNames();
  support_polygon_center_.setZero();
  for(unsigned int i = 0; i<foot_names.size(); i++)
  {
    support_polygon_center_ = support_polygon_center_ + foot_positions[foot_names[i]];
    support_polygon_edges_[i] = foot_positions[foot_names[i]];
  }

  support_polygon_center_ = support_polygon_center_/N_LEGS;
}

void ComPlanner::computeComPositionReference(const double& dt)
{
  // Update the support polygon everytime there is a touchdown
  // or if the robot is standing up or down (because if we
  // are using the estimated_z the com reference is calculated wrt the base which is
  // moving up/down)
  //if(foothold_planner_->isAnyFootInTouchDown()
  //   || robot_model_->getState() == QuadrupedRobot::STANDING_UP
  //   || robot_model_->getState() == QuadrupedRobot::STANDING_DOWN
  //   )
  //  update_ = true;

  // If all feet in stance then update the support polygon center
  if (foothold_planner_->areAllFeetInStance())
  {
    //if(update_)
    //{
      computeSupportPolygonCenter();
     // update_ = false;
    //}
  }

  com_position_ref_ << support_polygon_center_(0), support_polygon_center_(1), foothold_planner_->getBaseHeight();
}

void ComPlanner::computeComVelocityReference(const double& dt)
{

  //https://kodlab.seas.upenn.edu/uploads/Kod/Schwind95.pdf
  //vcom = 2*X_piede/T

  base_velocity_ = foothold_planner_->getBaseLinearVelocityReference();

  com_velocity_ref_.setZero();
  com_velocity_ref_ = terrain_estimator_->getTerrainOrientationWorld().transpose() * base_velocity_; // getPose(): world_T_terrain
  //com_velocity_ref_.z() = 0.0; // No height reference, only damping

  com_velocity_ref_ = 1.0/foothold_planner_->getVelocityFactor() * com_velocity_ref_;
}

void ComPlanner::update(const double& dt)
{
  // Update filters 
  com_position_ref_filter_.setTimeStep(dt);
  com_velocity_ref_filter_.setTimeStep(dt);
  com_position_ref_filter_.setOmega(2.0*M_PI*filters_cutoff_freq_);
  com_velocity_ref_filter_.setOmega(2.0*M_PI*filters_cutoff_freq_);

  // Update com pos and vel
  computeComVelocityReference(dt);
  computeComPositionReference(dt);

  // Process filters
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

void ComPlanner::setFiltersCutoffFreq(const double &hz)
{
  if(filters_cutoff_freq_> 0.0)
    filters_cutoff_freq_ = hz;
}

}; // namespace
