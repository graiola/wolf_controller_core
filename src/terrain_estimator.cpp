/**
 * @file terrain_estimator.cpp
 * @author Gennaro Raiola, Michele Focchi
 * @date 12 June, 2019
 * @brief Terrain estimator based on foot position fitting
 */

#include <wolf_controller_core/terrain_estimator.h>
#include <wolf_controller_utils/filters.h>
#include <wolf_controller_utils/geometry.h>
#include <Eigen/QR>

using namespace wolf_controller_utils;
using namespace wolf_wbid;

namespace wolf_controller {

TerrainEstimator::TerrainEstimator(StateEstimator::Ptr state_estimator,
                                   QuadrupedRobot::Ptr robot_model)
{
  assert(state_estimator);
  state_estimator_ = state_estimator;

  assert(robot_model);
  robot_model_ = robot_model;

  A_.resize(N_LEGS, 3);
  b_.resize(N_LEGS);

  rpy_filter_.setOmega(2.0 * M_PI * 3.0); // 3 Hz cutoff by default

  terrain_rotation_.setIdentity();
  x_axis_.setZero();
  y_axis_.setZero();
  z_axis_.setZero();
  rpy_.setZero();
  terrain_rpy_.setZero();
  avg_.setZero();
  terrain_params_.setZero();
  filtered_rpy_.setZero();

  reset();
}

bool TerrainEstimator::computeTerrainEstimation(const double& dt)
{
  if (!state_estimator_->areAllFeetInContact())
    return false;

  updateFootMatrix();

  // Solve for terrain normal using QR decomposition
  terrain_params_ = A_.colPivHouseholderQr().solve(b_);
  terrain_normal_ << terrain_params_(0), terrain_params_(1), 1.0;
  terrain_normal_.normalize();

  // Convert normal to roll/pitch (reusing pre-allocated variables)
  z_axis_ = terrain_normal_;
  x_axis_ = (Eigen::Vector3d::UnitX() - z_axis_ * (z_axis_.dot(Eigen::Vector3d::UnitX()))).normalized();
  y_axis_ = z_axis_.cross(x_axis_);

  terrain_rotation_.col(0) = x_axis_;
  terrain_rotation_.col(1) = y_axis_;
  terrain_rotation_.col(2) = z_axis_;

  rotToRpy(terrain_rotation_, rpy_);
  estimated_roll_ = rpy_(0);
  estimated_pitch_ = rpy_(1);

  rpy_filter_.setTimeStep(dt);
  filtered_rpy_ = rpy_filter_.process({estimated_roll_, estimated_pitch_, 0.0});
  roll_ = filtered_rpy_.x();
  pitch_ = filtered_rpy_.y();

  if (std::abs(roll_) > max_roll_ || std::abs(pitch_) > max_pitch_)
    return false;

  roll_out_world_ = roll_;
  pitch_out_world_ = pitch_;

  // Update terrain pose
  terrain_rpy_ << roll_, pitch_, 0.0;
  rpyToRotTranspose(terrain_rpy_, world_R_terrain_);
  world_T_terrain_.translation() = world_X_terrain_;
  world_T_terrain_.linear() = world_R_terrain_;

  hf_T_terrain_ = robot_model_->getHfRotationInWorld().transpose() * world_T_terrain_;
  hf_X_terrain_ = hf_T_terrain_.translation();
  hf_R_terrain_ = hf_T_terrain_.linear();

  rotTransposeToRpy(hf_R_terrain_, terrain_rpy_);
  roll_out_hf_ = terrain_rpy_(0);
  pitch_out_hf_ = terrain_rpy_(1);

  // Posture adjustment
  double terrain_h_base = state_estimator_->getFloatingBasePosition()(2);
  posture_adjustment_ = terrain_h_base * std::tan(pitch_out_hf_);
  posture_adjustment_dot_ = (posture_adjustment_ - posture_adjustment_prev_) / dt;
  posture_adjustment_prev_ = posture_adjustment_;
  posture_adjustment_dot_base_ << posture_adjustment_dot_, 0.0, 0.0;
  posture_adjustment_dot_world_ = robot_model_->getBaseRotationInWorld() * posture_adjustment_dot_base_;

  state_estimator_->setTerrainNormal(terrain_normal_);

  return true;
}

void TerrainEstimator::updateFootMatrix()
{
  auto foot_names = robot_model_->getFootNames();
  auto foot_positions = robot_model_->getFeetPositionInWorld();

  avg_.setZero();

  for (size_t i = 0; i < foot_names.size(); ++i)
  {
    const Eigen::Vector3d& pos = foot_positions[foot_names[i]];
    A_.row(i) << pos.x(), pos.y(), 1.0;
    b_(i) = -pos.z();
    avg_ += pos;
  }

  world_X_terrain_ = avg_ / static_cast<double>(foot_names.size());
}

void TerrainEstimator::reset()
{
  roll_ = pitch_ = roll_out_world_ = pitch_out_world_ = 0.0;
  estimated_roll_ = estimated_pitch_ = 0.0;

  terrain_normal_ << 0.0, 0.0, 1.0;
  world_X_terrain_ = hf_X_terrain_ = Eigen::Vector3d::Zero();
  world_T_terrain_ = hf_T_terrain_ = Eigen::Affine3d::Identity();
  world_R_terrain_ = hf_R_terrain_ = Eigen::Matrix3d::Identity();

  posture_adjustment_ = posture_adjustment_prev_ = posture_adjustment_dot_ = 0.0;
  posture_adjustment_dot_world_ = Eigen::Vector3d::Zero();

  rpy_filter_.reset(Eigen::Vector3d::Zero());
}

void TerrainEstimator::setMinRoll(const double min) { min_roll_ = min; }
void TerrainEstimator::setMinPitch(const double min) { min_pitch_ = min; }
void TerrainEstimator::setMaxRoll(const double max) { max_roll_ = max; }
void TerrainEstimator::setMaxPitch(const double max) { max_pitch_ = max; }

const double& TerrainEstimator::getRollInWorld() const { return roll_out_world_; }
const double& TerrainEstimator::getPitchInWorld() const { return pitch_out_world_; }
const double& TerrainEstimator::getRollInHf() const { return roll_out_hf_; }
const double& TerrainEstimator::getPitchInHf() const { return pitch_out_hf_; }
const Eigen::Vector3d& TerrainEstimator::getPostureAdjustmentDot() const { return posture_adjustment_dot_world_; }
const Eigen::Vector3d& TerrainEstimator::getTerrainNormal() const { return terrain_normal_; }
const Eigen::Vector3d& TerrainEstimator::getTerrainPositionWorld() const { return world_X_terrain_; }
const Eigen::Vector3d& TerrainEstimator::getTerrainPositionHf() const { return hf_X_terrain_; }
const Eigen::Matrix3d& TerrainEstimator::getTerrainOrientationWorld() const { return world_R_terrain_; }
const Eigen::Matrix3d& TerrainEstimator::getTerrainOrientationHf() const { return hf_R_terrain_; }
const Eigen::Affine3d& TerrainEstimator::getTerrainPoseWorld() const { return world_T_terrain_; }
const Eigen::Affine3d& TerrainEstimator::getTerrainPoseHf() const { return hf_T_terrain_; }

bool TerrainEstimator::isOnFlatTerrain()
{
  return std::abs(roll_out_world_) <= 0.01 && std::abs(pitch_out_world_) <= 0.01;
}

} // namespace
