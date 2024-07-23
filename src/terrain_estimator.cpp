/**
 * @file terrain_estimator.cpp
 * @author Gennaro Raiola, Michele Focchi
 * @date 12 June, 2019
 * @brief Terrain estimator based on foot position fitting
 */

#include <wolf_controller_core/terrain_estimator.h>
#include <wolf_controller_core/common.h>
#include <wolf_controller_utils/filters.h>
#include <wolf_controller_utils/geometry.h>

using namespace rt_logger;
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

  A_.resize(N_LEGS,3);
  Ai_.resize(3,N_LEGS);
  b_.resize(N_LEGS);

  reset();
}

bool TerrainEstimator::computeTerrainEstimation(const double& dt)
{

  posture_adjustment_dot_world_.setZero();

  // 0 - Update the terrain estimation everytime there is a new contact (touchdown)
  //auto foot_names = robot_model_->getFootNames();
  //for(unsigned int i=0; i<foot_names.size(); i++)
  //  update_ = update_ || touchdown_[foot_names[i]].update(state_estimator_->getContact(foot_names[i]));

  // 1 - Check if the feet are all in stance
  if(state_estimator_->areAllFeetInContact())
  {

    // NOTE: I commented out this line because it was causing huge steps in the terrain height estimation.
    // This can be a real problem when going up stairs.
    //if(update_ == true)
    //{

      // 2 - Update A and b with the feet position
      update();

      tmp_matrix3d_.noalias() = A_.transpose()*A_;

      if(tmp_matrix3d_.determinant()!=0.0)
      {
        Ai_.noalias() = tmp_matrix3d_.inverse() * A_.transpose();
        terrain_normal_ = Ai_ * b_;
      }
      else
      {
        ROS_WARN_STREAM_NAMED(CLASS_NAME,"Can not solve the problem!");
        return false;
      }

      // 3 - Normalize the terrain normal
      terrain_normal_(2) = 1.0;
      terrain_normal_ = terrain_normal_ / terrain_normal_.norm();

      // 4 - Extract the estimated values for roll and pitch
      estimated_pitch_  = std::atan(terrain_normal_(0)/terrain_normal_(2));
      estimated_roll_ = std::atan(-terrain_normal_(1)*std::sin(estimated_pitch_)/terrain_normal_(0));

      // Perform only one update per touch down
      //update_ = false;
    //}
  }

  // 5 - Filter
  roll_  = secondOrderFilter(roll_,roll_filt_,estimated_roll_,1.0);
  pitch_ = secondOrderFilter(pitch_,pitch_filt_,estimated_pitch_,1.0);

  // 6 - Check output limits
  if((roll_  > min_roll_ ) && (roll_  < max_roll_) &&
     (pitch_ > min_pitch_) && (pitch_ < max_pitch_))
  {
    // 7 - Update the roll and pitch output values
    roll_out_world_  = roll_;
    pitch_out_world_ = pitch_;
  }
  else
  {
    ROS_WARN_STREAM_NAMED(CLASS_NAME,"Angles beyond limits!");
    return false;
  }

  // 7 - Update the resulting Transformation
  // 7.1 World transformations
  tmp_vector3d_ << roll_out_world_,pitch_out_world_,0.0;
  rpyToRotTranspose(tmp_vector3d_,world_R_terrain_);
  world_T_terrain_.translation() = world_X_terrain_;
  world_T_terrain_.linear() = world_R_terrain_;
  // 7.2 Horizontal frame transformations
  hf_T_terrain_ = robot_model_->getHfRotationInWorld().transpose() * world_T_terrain_;
  hf_X_terrain_ = hf_T_terrain_.translation();
  hf_R_terrain_ = hf_T_terrain_.linear();
  rotTransposeToRpy(hf_R_terrain_,tmp_vector3d_);
  roll_out_hf_ = tmp_vector3d_(0);
  pitch_out_hf_ = tmp_vector3d_(1);

  // 8 - Update the state estimator to align the contact forces with the
  // terrain and the base height and rotation references
  state_estimator_->setTerrainNormal(terrain_normal_);

  // 9 - Posture adjustment, compute the offsets to help adapting the posture w.r.t
  // terrain, we compute only an adjustment along the x axis of the base
  double terrain_h_base = state_estimator_->getFloatingBasePosition()(2);
  posture_adjustment_ = terrain_h_base * std::tan(pitch_out_hf_);
  posture_adjustment_dot_ = (posture_adjustment_ - posture_adjustment_prev_)/dt;
  posture_adjustment_prev_ = posture_adjustment_;
  posture_adjustment_dot_base_ << posture_adjustment_dot_, 0.0, 0.0;
  posture_adjustment_dot_world_ = robot_model_->getBaseRotationInWorld() * posture_adjustment_dot_base_;  // w.r.t world
  //kin_->setDesiredPostureAdjustmentDot(tmp_vector3d_); // This is ok but I need to be sure that state_estimator_->getFloatingHfPosition()(2) is adapted wrt terrain

  // 10 - Adjust the references for the desired Hf height and orientation (roll and pitch)
  // and updates the foot trajectories with the terrain rotation
  //foot_holds_planner_->setTerrainTransform(world_T_terrain_); Now it is done in updateWpg()

  return true;
}

void TerrainEstimator::reset()
{
  update_ = true; // true to have a first update

  auto foot_names = robot_model_->getFootNames();
  for(unsigned int i=0; i<foot_names.size(); i++)
    touchdown_[foot_names[i]].update(false);

  roll_ = roll_filt_ = roll_out_world_ = roll_out_hf_ = estimated_roll_ = 0.0;
  pitch_ = pitch_filt_ = pitch_out_world_ = pitch_out_hf_ = estimated_pitch_ = 0.0;

  terrain_normal_ << 0.0, 0.0, 1.0;
  world_X_terrain_ = hf_X_terrain_ = Eigen::Vector3d::Zero();
  world_T_terrain_ = hf_T_terrain_ = Eigen::Affine3d::Identity();
  world_R_terrain_ = hf_R_terrain_ = Eigen::Matrix3d::Identity();

  posture_adjustment_ = posture_adjustment_prev_ = posture_adjustment_dot_ = 0.0;
  posture_adjustment_dot_world_ = Eigen::Vector3d::Zero();
}

void TerrainEstimator::update()
{
  auto foot_names = robot_model_->getFootNames();
  auto foot_positions = robot_model_->getFeetPositionInWorld();

  A_(0,0) = foot_positions[foot_names[0]](0);
  A_(0,1) = foot_positions[foot_names[0]](1);
  A_(0,2) = 1.0;
  b_(0)   = -foot_positions[foot_names[0]](2);

  A_(1,0) = foot_positions[foot_names[1]](0);
  A_(1,1) = foot_positions[foot_names[1]](1);
  A_(1,2) = 1.0;
  b_(1)   = -foot_positions[foot_names[1]](2);

  A_(2,0) = foot_positions[foot_names[2]](0);
  A_(2,1) = foot_positions[foot_names[2]](1);
  A_(2,2) = 1.0;
  b_(2)   = -foot_positions[foot_names[2]](2);

  A_(3,0) = foot_positions[foot_names[3]](0);
  A_(3,1) = foot_positions[foot_names[3]](1);
  A_(3,2) = 1.0;
  b_(3)   = -foot_positions[foot_names[3]](2);

  double avg_x = 0.0;
  double avg_y = 0.0;
  double avg_z = 0.0;
  for(unsigned int i = 0; i<foot_names.size(); i++)
  {
    avg_x = avg_x + foot_positions[foot_names[i]](0);
    avg_y = avg_y + foot_positions[foot_names[i]](1);
    avg_z = avg_z + foot_positions[foot_names[i]](2);
  }

  avg_x = avg_x/N_LEGS;
  avg_y = avg_y/N_LEGS;
  avg_z = avg_z/N_LEGS;

  world_X_terrain_ << avg_x, avg_y, avg_z;

}

void TerrainEstimator::setMinRoll(const double min)
{
  min_roll_ = min;
}

void TerrainEstimator::setMinPitch(const double min)
{
  min_pitch_ = min;
}

void TerrainEstimator::setMaxRoll(const double max)
{
  max_roll_ = max;
}

void TerrainEstimator::setMaxPitch(const double max)
{
  max_pitch_ = max;
}

const double& TerrainEstimator::getRollInWorld() const
{
  return roll_out_world_;
}

const double& TerrainEstimator::getPitchInWorld() const
{
  return pitch_out_world_;
}

const double& TerrainEstimator::getRollInHf() const
{
  return roll_out_hf_;
}

const double& TerrainEstimator::getPitchInHf() const
{
  return pitch_out_hf_;
}

const Eigen::Vector3d &TerrainEstimator::getPostureAdjustmentDot() const
{
  return posture_adjustment_dot_world_;
}

const Eigen::Vector3d& TerrainEstimator::getTerrainNormal() const
{
  return terrain_normal_;
}

const Eigen::Vector3d &TerrainEstimator::getTerrainPositionWorld() const
{
  return world_X_terrain_;
}

const Eigen::Vector3d &TerrainEstimator::getTerrainPositionHf() const
{
  return hf_X_terrain_;
}

const Eigen::Matrix3d &TerrainEstimator::getTerrainOrientationWorld() const
{
  return world_R_terrain_;
}

const Eigen::Matrix3d &TerrainEstimator::getTerrainOrientationHf() const
{
  return hf_R_terrain_;
}

const Eigen::Affine3d &TerrainEstimator::getTerrainPoseWorld() const
{
  return world_T_terrain_;
}

const Eigen::Affine3d &TerrainEstimator::getTerrainPoseHf() const
{
  return hf_T_terrain_;
}

bool TerrainEstimator::isOnFlatTerrain()
{
 if(std::abs(roll_out_world_) <= 0.01 && std::abs(pitch_out_world_) <= 0.01)
   return true;
 else
   return false;
}

} // namespace

