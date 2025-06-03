/**
 * @file footholds_planner.cpp
 * @author Gennaro Raiola, Michele Focchi
 * @date 12 June, 2019
 * @brief This file contains the implementation of the walking pattern generator and push
 * recovery
 */

#include <wolf_controller_core/wpg/footholds_planner.h>
#include <wolf_controller_utils/filters.h>
#include <OpenSoT/utils/cartesian_utils.h>

// RT LOGGER
#ifdef RT_LOGGER
#include <rt_logger/rt_logger.h>
using namespace rt_logger;
#endif

using namespace wolf_controller_utils;
using namespace wolf_wbid;

namespace wolf_controller {

#define GAIN 0.05

FootholdsPlanner::FootholdsPlanner(StateMachine::Ptr state_machine, ComPlanner::Ptr com_planner, GaitGenerator::Ptr gait_generator, QuadrupedRobot::Ptr robot_model, double step_length_max, double step_height_max)
{
  assert(state_machine);
  state_machine_ = state_machine;
  assert(com_planner);
  com_planner_ = com_planner;
  assert(gait_generator);
  gait_generator_ = gait_generator;
  assert(robot_model);
  robot_model_ = robot_model;
  assert(step_length_max>=0.0);
  step_length_max_ = step_length_max;
  assert(step_height_max>=0.0);
  step_height_max_ = step_height_max;

  hf_X_initial_footholds_.resize(4);
  hf_X_initial_hips_.resize(4); // \f$X_hip(i)\f$ with i corresponding to the leg number
  for(unsigned int i=0; i<4; i++)
  {
    hf_X_initial_hips_[i].setZero();
    hf_X_initial_footholds_[i].setZero();
  }

  offsets_applied_ = false;

  world_T_terrain_ = world_T_terrain_init_ = Eigen::Affine3d::Identity();

  push_recovery_ = std::make_shared<PushRecovery>(this);
  push_detected_ = false;
  push_recovery_active_ = false;
  use_com_planner_references_ = true;

  step_height_ = 0.0; // [m]
  step_length_ = 0.0; // [m]

  base_linear_velocity_cmd_x_ = 0.0; // [m/s]
  base_linear_velocity_cmd_y_ = 0.0; // [m/s]
  base_linear_velocity_cmd_z_ = 0.0; // [m/s]
  base_angular_velocity_cmd_roll_  = 0.0; // [rad/s]
  base_angular_velocity_cmd_pitch_ = 0.0; // [rad/s]
  base_angular_velocity_cmd_yaw_   = 0.0; // [rad/s]

  double omega = 2.0 * M_PI * 3; // 3Hz
  linear_velocity_filter_.setOmega(omega);
  angular_velocity_filter_.setOmega(omega);
  reset();
#ifdef RT_LOGGER
  RtLogger::getLogger().addPublisher(TOPIC(des_base_height),base_position_(2));
#endif
}

void FootholdsPlanner::reset()
{

  const std::vector<std::string>& foot_names = gait_generator_->getFootNames();
  for(unsigned int i=0;i<foot_names.size();i++)
  {
    steps_length_[foot_names[i]] = step_length_;
    steps_heading_[foot_names[i]] = 0.0;
    steps_heading_rate_[foot_names[i]] = 0.0;
    steps_height_[foot_names[i]] = step_height_;

    robot_model_->getPose(foot_names[i],robot_model_->getBaseLinkName(),tmp_affine3d_); // base_T_foot
    tmp_matrix3d_ = robot_model_->getBaseRotationInHf(); // hf_R_base
    desired_foothold_[foot_names[i]]    = tmp_affine3d_.translation();
    virtual_foothold_[foot_names[i]]    = tmp_affine3d_.translation();
    current_foothold_hf_[foot_names[i]] = tmp_matrix3d_ * tmp_affine3d_.translation();
    current_foothold_[foot_names[i]]    = tmp_affine3d_.translation();
    capture_point_delta_[foot_names[i]]  = Eigen::Vector2d::Zero();
  }

  resetVelocyScales();

  robot_model_->getFloatingBasePose(tmp_affine3d_);
  base_rotation_reference_ = tmp_affine3d_.linear();
  rotToRpy(base_rotation_reference_,base_orientation_);
  base_position_ = tmp_affine3d_.translation();

  gait_generator_->reset();

  terrain_transform_init_ = false;

  cmd_ = cmd_t::HOLD;
}

void FootholdsPlanner::update(const double& period, const Eigen::Vector3d& base_position) // OpenLoop Orientation
{
  update(period,base_position,base_orientation_);
}

void FootholdsPlanner::update(const double& period) // OpenLoop
{
  update(period,base_position_,base_orientation_);
}

void FootholdsPlanner::initializeFootPosition(const std::string& foot_name)
{
  robot_model_->getPose(foot_name,tmp_affine3d_); // world_T_foot
  gait_generator_->setInitialPose(foot_name,tmp_affine3d_);
}

void FootholdsPlanner::initializeFeetPosition()
{
  const std::vector<std::string>& foot_names = gait_generator_->getFootNames();

  for(unsigned int i=0; i<foot_names.size(); i++)
    initializeFootPosition(foot_names[i]);
}

void FootholdsPlanner::update(const double& period, const Eigen::Vector3d& base_position, const Eigen::Vector3d& base_orientation) // ClosedLoop
{
  unsigned int cmd = cmd_;

  world_R_hf_ = robot_model_->getHfRotationInWorld();
  hf_R_base_  = robot_model_->getBaseRotationInHf();

  setInitialOffsets();

  switch(cmd)
  {

  case cmd_t::HOLD:
    resetBaseVelocities();
    resetVelocyScales();
    calculateBasePosition(period,base_position);
    calculateBaseOrientation(period,base_orientation);
    break;

  case cmd_t::LINEAR:
    calculateBasePosition(period,base_position);
    resetBaseAngularVelocity();
    break;

  case cmd_t::ANGULAR:
    calculateBaseOrientation(period,base_orientation);
    resetBaseLinearVelocity();
    break;

  case cmd_t::LINEAR_AND_ANGULAR:
    calculateBasePosition(period,base_position);
    calculateBaseOrientation(period,base_orientation);
    break;

  case cmd_t::BASE_ONLY:
    calculateBasePosition(period,base_position);
    calculateBaseOrientation(period,base_orientation);
    break;

  case cmd_t::RESET_BASE:
    resetBaseVelocities();
    resetBasePosition();
    resetBaseOrientation();
    break;
  };

  if(push_recovery_active_ && push_recovery_->update(period))
  {
    push_detected_ = true;
    //ROS_DEBUG_NAMED(CLASS_NAME,"Push detected!");
  }
  else
    push_detected_ = false;

  const std::vector<std::string>& foot_names = gait_generator_->getFootNames();
  for(unsigned int i=0; i<foot_names.size(); i++)
    // Set the initial pose for the next swing
    if(gait_generator_->isLiftOff(foot_names[i]))
      initializeFootPosition(foot_names[i]);

  // Update the com references
  com_planner_->update(period,base_linear_velocity_reference_,base_position_reference_(2));

  calculateFootSteps();

  for(unsigned int i=0; i<foot_names.size(); i++)
  {
    gait_generator_->setStepLength(foot_names[i], steps_length_[foot_names[i]]);
    gait_generator_->setStepHeading(foot_names[i], steps_heading_[foot_names[i]]);
    gait_generator_->setStepHeight(foot_names[i], steps_height_[foot_names[i]]);
    gait_generator_->setStepHeadingRate(foot_names[i], steps_heading_rate_[foot_names[i]]);
  }
  gait_generator_->setTerrainRotation(world_T_terrain_.linear());

  if(state_machine_->getCurrentState() == StateMachine::ACTIVE)
  {
    if(cmd == cmd_t::LINEAR_AND_ANGULAR || push_detected_)
    {
      if(push_detected_ && gait_generator_->getGaitType() == Gait::gait_t::CRAWL)
        gait_generator_->setGaitType(Gait::gait_t::TROT);
      //robot_model_->setState(QuadrupedRobot::WALKING);
      gait_generator_->activateSwing();
    }
    else
    {
      gait_generator_->deactivateSwing();
    }
  }

  // Update the gait_generator
  gait_generator_->update(period);
}

void FootholdsPlanner::calculateFootSteps()
{
  const std::vector<std::string>& foot_names = robot_model_->getFootNames();

  for(unsigned int i=0; i<foot_names.size(); i++)
  {

    //ROS_DEBUG_STREAM_NAMED(CLASS_NAME,"*********");
    //ROS_DEBUG_STREAM_NAMED(CLASS_NAME,"CalculateFootSteps for foot "<<foot_names[i]);

    if(gait_generator_->isLiftOff(foot_names[i]))
    {

      capture_point_delta_[foot_names[i]] = push_recovery_->getDelta(foot_names[i]);

      // Overwrite the linear velocity references with the com planner ones
      if(use_com_planner_references_)
      {
        hf_base_linear_velocity_.setZero();
        hf_base_linear_velocity_ = world_R_hf_.transpose() * com_planner_->getComVelocity();
      }

      // 1) Compute the displacement of the foot produced by the linear velocity command
      hf_delta_hip_.setZero(); // \f$\deltaL_{x,y,0}\f$
      hf_delta_hip_(0) = hf_base_linear_velocity_(0)*1.0/gait_generator_->getSwingFrequency(foot_names[i]);
      hf_delta_hip_(1) = hf_base_linear_velocity_(1)*1.0/gait_generator_->getSwingFrequency(foot_names[i]);
      //ROS_DEBUG_STREAM_NAMED(CLASS_NAME,"hf_delta_hip_ (Linear part): "<<hf_delta_hip_.transpose());

      // 2) Compute the displacement of the foot produced by the angular velocity command
      hf_delta_heading_.setZero(); // \f$\deltaL_{h,0}\f$
      hf_delta_heading_(2) = hf_base_angular_velocity_(2)*1.0/gait_generator_->getSwingFrequency(foot_names[i]);
      hf_delta_heading_ = hf_delta_heading_.cross(hf_X_initial_hips_[i]);
      //ROS_DEBUG_STREAM_NAMED(CLASS_NAME,"hf_delta_heading_ (Angular part): "<<hf_delta_heading_.transpose());

      // 3) Combine the two displacements
      hf_delta_hip_(0)+= hf_delta_heading_(0);
      hf_delta_hip_(1)+= hf_delta_heading_(1);
      //ROS_DEBUG_STREAM_NAMED(CLASS_NAME,"hf_delta_hip_ (Combined): "<<hf_delta_hip_.transpose());

    }
    else if(gait_generator_->isSwinging(foot_names[i]))
    {

      // 4) Calculate the foothold offset based on the initial feet position (virtual foothold offset)
      robot_model_->getPose(foot_names[i],robot_model_->getBaseLinkName(),base_T_foot_);
      // current foot position in the horizontal frame
      hf_X_current_foothold_ = hf_R_base_ * base_T_foot_.translation();
      //ROS_DEBUG_STREAM_NAMED(CLASS_NAME,"hf_X_current_foothold_: "<<hf_X_current_foothold_.transpose());

      // 5) Sum everything to obtain the new foothold displacement w.r.t hf
      hf_delta_foot_.setZero();
      hf_delta_foot_.head(2) =  hf_delta_hip_.head(2) + (hf_X_initial_footholds_[i] - hf_X_current_foothold_).head(2);

      //6) Sum the delta for the push recovery
      hf_delta_foot_.head(2) =  hf_delta_foot_.head(2) + capture_point_delta_[foot_names[i]];
      //ROS_DEBUG_STREAM_NAMED(CLASS_NAME,"hf_delta_foot_: "<<hf_delta_foot_.transpose());

      // 6a) Add stabilizing COM-based shift - This shift helps pull the swing leg toward the COM reference projection in the horizontal frame
      /*if (use_com_planner_references_)
      {
        Eigen::Vector3d com_pos_ref = com_planner_->getComPosition();      // Global COM ref
        Eigen::Vector3d hf_com_pos_ref = hf_R_base_ * com_pos_ref;         // Project into HF

        Eigen::Vector2d stabilizing_shift = stability_gain_ * (hf_com_pos_ref.head<2>() - hf_X_current_foothold_.head<2>());

        hf_delta_foot_.head(2) += stabilizing_shift;

        // Optional: debug
        // ROS_DEBUG_STREAM("Stabilizing shift for " << foot_names[i] << ": " << stabilizing_shift.transpose());
      }*/

      // 6) Sum everything to obtain the new foothold displacement w.r.t world
      //world_delta_foot_.setZero();
      //world_delta_foot_.head(2) =  world_R_hf_ * hf_delta_foot_;
      ////ROS_DEBUG_STREAM_NAMED(CLASS_NAME,"world_delta_foot_: "<<world_delta_foot_.transpose());

      // 7) Get the step length and heading
      step_length_ = std::sqrt(hf_delta_foot_(0)*hf_delta_foot_(0) + hf_delta_foot_(1)*hf_delta_foot_(1));

      if(step_length_ > step_length_max_)
      {
        step_length_ = step_length_max_;
        //ROS_WARN_STREAM_THROTTLE_NAMED(THROTTLE_SEC,CLASS_NAME,"Step length is greater than: "<<step_length_max_);
      }

      steps_length_[foot_names[i]]         = step_length_;
      steps_heading_[foot_names[i]]        = std::atan2(hf_delta_foot_(1),hf_delta_foot_(0)) + robot_model_->getBaseYawInWorld();
      steps_height_[foot_names[i]]         = step_height_;
      steps_heading_rate_[foot_names[i]]   = hf_base_angular_velocity_(2);
    }
    else if(gait_generator_->isInStance(foot_names[i]))
    {
      steps_length_[foot_names[i]]         = 0.0;
      steps_heading_[foot_names[i]]        = 0.0;
      steps_height_[foot_names[i]]         = 0.0;
      steps_heading_rate_[foot_names[i]]   = 0.0;
      capture_point_delta_[foot_names[i]]  = Eigen::Vector2d::Zero();
    }

    virtual_foothold_[foot_names[i]]        = hf_X_initial_footholds_[i];
    current_foothold_[foot_names[i]]        = base_T_foot_.translation();
    current_foothold_hf_[foot_names[i]]     = hf_X_current_foothold_;
    desired_foothold_[foot_names[i]].x()    = current_foothold_[foot_names[i]].x() + steps_length_[foot_names[i]] * std::cos(steps_heading_[foot_names[i]]);
    desired_foothold_[foot_names[i]].y()    = current_foothold_[foot_names[i]].y() + steps_length_[foot_names[i]] * std::sin(steps_heading_[foot_names[i]]);

    //ROS_DEBUG_STREAM_NAMED(CLASS_NAME,"steps_length["<<foot_names[i]<<"]: "<<steps_length_[foot_names[i]]);
    //ROS_DEBUG_STREAM_NAMED(CLASS_NAME,"steps_heading["<<foot_names[i]<<"]: "<<steps_heading_[foot_names[i]]);
    //ROS_DEBUG_STREAM_NAMED(CLASS_NAME,"steps_height["<<foot_names[i]<<"]: "<<steps_height_[foot_names[i]]);
    //ROS_DEBUG_STREAM_NAMED(CLASS_NAME,"steps_heading_rate["<<foot_names[i]<<"]: "<<steps_heading_rate_[foot_names[i]]);
  }
}

void FootholdsPlanner::resetFeetStep()
{
  const std::vector<std::string>& foot_names = gait_generator_->getFootNames();
  for(unsigned int i=0; i<foot_names.size(); i++)
  {
    steps_length_[foot_names[i]]         = 0.0;
    steps_heading_[foot_names[i]]        = 0.0;
    steps_height_[foot_names[i]]         = 0.0;
    steps_heading_rate_[foot_names[i]]   = 0.0;
    capture_point_delta_[foot_names[i]]  = Eigen::Vector2d::Zero();
  }
}

void FootholdsPlanner::resetBaseAngularVelocity()
{
  hf_base_angular_velocity_.setZero();
  hf_base_angular_velocity_ref_.setZero();
}

void FootholdsPlanner::resetBaseLinearVelocity()
{
  hf_base_linear_velocity_.setZero();
  hf_base_linear_velocity_ref_.setZero();
}

void FootholdsPlanner::resetBaseVelocities()
{
  resetBaseAngularVelocity();
  resetBaseLinearVelocity();
}

void FootholdsPlanner::resetBasePosition()
{
  for(unsigned int i=0;i<3;i++)
    base_position_reference_(i) = secondOrderFilter(base_position_(i),base_position_filt_(i),default_base_position_(i),1.0);
}

void FootholdsPlanner::resetBaseOrientation()
{
  default_base_orientation_(2) = base_orientation_(2); // Keep the same yaw

  for(unsigned int i=0;i<3;i++)
    base_orientation_(i) = secondOrderFilter(base_orientation_(i),base_orientation_filt_(i),default_base_orientation_(i),1.0); //FIXME hardcoded gain, it should be based on the sampling time

  rpyToRot(base_orientation_,base_rotation_reference_);

  // This is the base rotation reference computed w.r.t terrain
  base_rotation_reference_ = world_T_terrain_.linear().transpose() * base_rotation_reference_;
}

void FootholdsPlanner::resetVelocyScales()
{
  base_linear_velocity_scale_x_ = 0.0;
  base_linear_velocity_scale_y_ = 0.0;
  base_linear_velocity_scale_z_ = 0.0;

  base_angular_velocity_scale_roll_ = 0.0;
  base_angular_velocity_scale_pitch_ = 0.0;
  base_angular_velocity_scale_yaw_ = 0.0;
}

void FootholdsPlanner::calculateBasePosition(const double& period, const Eigen::Vector3d& base_position)
{
  base_position_ = base_position;

  double f = gait_generator_->getVelocityFactor();

  hf_base_linear_velocity_ref_(0) = f * base_linear_velocity_cmd_x_ * base_linear_velocity_scale_x_;
  hf_base_linear_velocity_ref_(1) = f * base_linear_velocity_cmd_y_ * base_linear_velocity_scale_y_;
  hf_base_linear_velocity_ref_(2) = base_linear_velocity_cmd_z_ * base_linear_velocity_scale_z_;

  //for(unsigned int i=0;i<3;i++)
  //  hf_base_linear_velocity_(i) = secondOrderFilter(hf_base_linear_velocity_(i),hf_base_linear_velocity_filt_(i),hf_base_linear_velocity_ref_(i),GAIN); //FIXME hardcoded gain, it should be based on the sampling time

  linear_velocity_filter_.setTimeStep(period);
  hf_base_linear_velocity_ = linear_velocity_filter_.process(hf_base_linear_velocity_ref_);
  
  base_linear_velocity_reference_ = world_R_hf_ * hf_base_linear_velocity_;

  base_position_ = base_linear_velocity_reference_ * period + base_position_;

  // This is the base height reference computed w.r.t terrain ( reference = reference_world - reference_terrain )
  base_position_reference_.head(2) = base_position_.head(2);

  double terrain_delta = (world_T_terrain_.translation().z() - world_T_terrain_init_.translation().z());
  double base_height_max_wrt_terrain = base_height_max_ + world_T_terrain_.translation().z();
  double base_height_min_wrt_terrain = world_T_terrain_.translation().z();

  base_position_reference_(2) = base_position_(2) + terrain_delta;

  // Clamp the z value
  if(base_position_reference_(2) > base_height_max_wrt_terrain)
  {
    base_position_reference_(2) = base_height_max_wrt_terrain;
    base_linear_velocity_reference_(2) = hf_base_linear_velocity_ref_(2) = 0.0;
    //ROS_WARN_STREAM_THROTTLE_NAMED(THROTTLE_SEC,CLASS_NAME,"Desired base height limit reached: "<<base_height_max_);
  }
  else if (base_position_reference_(2) < base_height_min_wrt_terrain)
  {
    base_position_reference_(2) = base_height_min_wrt_terrain;
    base_linear_velocity_reference_(2) = hf_base_linear_velocity_ref_(2) = 0.0;
    //ROS_WARN_STREAM_THROTTLE_NAMED(THROTTLE_SEC,CLASS_NAME,"Desired base height limit reached: "<<0.0);
  }
}

void FootholdsPlanner::calculateBaseOrientation(const double& period, const Eigen::Vector3d& base_orientation)
{
  base_orientation_ = base_orientation;

  hf_base_angular_velocity_ref_(0) = base_angular_velocity_cmd_roll_ * base_angular_velocity_scale_roll_;
  hf_base_angular_velocity_ref_(1) = base_angular_velocity_cmd_pitch_ * base_angular_velocity_scale_pitch_;
  hf_base_angular_velocity_ref_(2) = base_angular_velocity_cmd_yaw_ * base_angular_velocity_scale_yaw_;

  //for(unsigned int i=0;i<3;i++)
  //  hf_base_angular_velocity_(i) = secondOrderFilter(hf_base_angular_velocity_(i),hf_base_angular_velocity_filt_(i),hf_base_angular_velocity_ref_(i),GAIN);

  angular_velocity_filter_.setTimeStep(period);
  hf_base_angular_velocity_ = angular_velocity_filter_.process(hf_base_angular_velocity_ref_);

  base_angular_velocity_reference_ = hf_base_angular_velocity_;

  base_orientation_ = base_angular_velocity_reference_ * period + base_orientation_;

  // Clamp the roll and pitch values
  if(base_orientation_(0) > base_roll_max_)
  {
    base_orientation_(0) = base_roll_max_;
    base_angular_velocity_reference_(0) = hf_base_angular_velocity_ref_(0) = 0.0;
    //ROS_WARN_STREAM_THROTTLE_NAMED(THROTTLE_SEC,CLASS_NAME,"Desired base roll rotation max reached: "<<base_roll_max_);
  }
  else if (base_orientation_(0) < base_roll_min_)
  {
    base_orientation_(0) = base_roll_min_;
    base_angular_velocity_reference_(0) = hf_base_angular_velocity_ref_(0) = 0.0;
    //ROS_WARN_STREAM_THROTTLE_NAMED(THROTTLE_SEC,CLASS_NAME,"Desired base roll rotation min reached: "<<base_roll_min_);
  }
  if(base_orientation_(1) > base_roll_max_)
  {
    base_orientation_(1) = base_pitch_max_;
    base_angular_velocity_reference_(1) = hf_base_angular_velocity_ref_(1) = 0.0;
    //ROS_WARN_STREAM_THROTTLE_NAMED(THROTTLE_SEC,CLASS_NAME,"Desired base pitch rotation max reached: "<<base_pitch_max_);
  }
  else if (base_orientation_(1) < base_pitch_min_)
  {
    base_orientation_(1) = base_pitch_min_;
    base_angular_velocity_reference_(1) = hf_base_angular_velocity_ref_(1) = 0.0;
    //ROS_WARN_STREAM_THROTTLE_NAMED(THROTTLE_SEC,CLASS_NAME,"Desired base pitch rotation min reached: "<<base_pitch_min_);
  }

  rpyToRot(base_orientation_,base_rotation_reference_);
  // This is the base rotation reference computed w.r.t terrain
  base_rotation_reference_ = world_T_terrain_.linear().transpose() * base_rotation_reference_;
}

void FootholdsPlanner::setInitialOffsets()
{
  if(!offsets_applied_)
  {
    const std::vector<std::string>& hips_names = robot_model_->getHipNames();
    for(unsigned int i=0; i<hips_names.size(); i++)
    {
      robot_model_->getPose(robot_model_->getStandUpJointPostion(),gait_generator_->getFootNames()[i],robot_model_->getBaseLinkName(),tmp_affine3d_1_); // base_T_foot
      robot_model_->getPose(robot_model_->getStandUpJointPostion(),hips_names[i],robot_model_->getBaseLinkName(),tmp_affine3d_); // base_T_hip
      tmp_matrix3d_ = robot_model_->getBaseRotationInHf(); // hf_R_base_
      // initial hip positions, we assume the base starts horizontal (TODO)
      tmp_matrix3d_.setIdentity();
      // initial feet offsets in the horizontal frame
      hf_X_initial_footholds_[i] = tmp_matrix3d_ * tmp_affine3d_1_.translation(); // base_X_foot
      // initial hip positions, we assume the base starts horizontal (TODO)
      hf_X_initial_hips_[i] = tmp_affine3d_.translation(); //base_X_hip
    }

    offsets_applied_ = true;
  }
}

void FootholdsPlanner::startPushRecovery(bool start)
{
  push_recovery_active_ = start;
  if(push_recovery_active_)
    push_recovery_->activateComputeDeltas();
  else
    push_recovery_->deactivateComputeDeltas();
}

void FootholdsPlanner::togglePushRecovery()
{
  push_recovery_active_ = !push_recovery_active_;
  if(push_recovery_active_)
    push_recovery_->activateComputeDeltas();
  else
    push_recovery_->deactivateComputeDeltas();
}

bool FootholdsPlanner::isPushRecoveryActive() const
{
  return push_recovery_active_;
}

// Sets
void FootholdsPlanner::setCmd(const unsigned int cmd)
{
  cmd_ = cmd;
}

void FootholdsPlanner::setBasePosition(const Eigen::Vector3d& position)
{
  base_position_ = position;
}

void FootholdsPlanner::setBaseOrientation(const Eigen::Vector3d& orientation)
{
  base_orientation_ = orientation;
}

void FootholdsPlanner::setDefaultBaseOrientation(const Eigen::Vector3d& orientation)
{
  default_base_orientation_ = orientation;
}

void FootholdsPlanner::setDefaultBasePosition(const Eigen::Vector3d& position)
{
  default_base_position_ = position;
}

void FootholdsPlanner::setBaseVelocityScaleX(const double& scale)
{
  base_linear_velocity_scale_x_ = scale;
}

void FootholdsPlanner::setBaseVelocityScaleY(const double& scale)
{
  base_linear_velocity_scale_y_ = scale;
}

void FootholdsPlanner::setBaseVelocityScaleZ(const double& scale)
{
  base_linear_velocity_scale_z_ = scale;
}

void FootholdsPlanner::setBaseVelocityScaleRoll(const double& scale)
{
  base_angular_velocity_scale_roll_ = scale;
}

void FootholdsPlanner::setBaseVelocityScalePitch(const double& scale)
{
  base_angular_velocity_scale_pitch_ = scale;
}

void FootholdsPlanner::setBaseVelocityScaleYaw(const double& scale)
{
  base_angular_velocity_scale_yaw_ = scale;
}

void FootholdsPlanner::increaseStepHeight()
{
  setStepHeight(step_height_ + 0.01); // Increase step height
}

void FootholdsPlanner::decreaseStepHeight()
{
  setStepHeight(step_height_ - 0.01); // Decrease step height
}

void FootholdsPlanner::setTerrainTransform(const Eigen::Affine3d &world_T_terrain)
{
  if(!terrain_transform_init_)
  {
    world_T_terrain_init_ = world_T_terrain;
    terrain_transform_init_ = true;
  }

  world_T_terrain_ = world_T_terrain;
}

void FootholdsPlanner::setPushRecoverySensibility(const double& v)
{
  if(v>=0.0 && v<= 1.0)
  {
    push_recovery_->setScaleValue(1.0 - v);
    PRINT_INFO_NAMED(CLASS_NAME,"Set push recovery sensibility to "<< v);
  }
  else
    PRINT_WARN_NAMED(CLASS_NAME,"Push recovery sensibility has to be defined between 0 and 1!");
}

void FootholdsPlanner::setBaseLinearVelocityCmd(const double& linear)
{
  base_linear_velocity_cmd_x_ = linear;
  base_linear_velocity_cmd_y_ = linear;
  base_linear_velocity_cmd_z_ = linear;
  PRINT_INFO_NAMED(CLASS_NAME,"Set base linear velocity to "<< linear);
}

void FootholdsPlanner::setBaseAngularVelocityCmd(const double& angular)
{
  base_angular_velocity_cmd_roll_  = angular;
  base_angular_velocity_cmd_pitch_ = angular;
  base_angular_velocity_cmd_yaw_   = angular;
  PRINT_INFO_NAMED(CLASS_NAME,"Set base angular velocity to "<< angular);
}

void FootholdsPlanner::setBaseLinearVelocityCmd(const double& x, const double& y, const double& z, bool verbose)
{
  base_linear_velocity_cmd_x_ = x;
  base_linear_velocity_cmd_y_ = y;
  base_linear_velocity_cmd_z_ = z;
  if(verbose)
    PRINT_INFO_NAMED(CLASS_NAME,"Set base linear velocity to "<<" "<<x<<" "<<y<<" "<<z);
}

void FootholdsPlanner::setBaseAngularVelocityCmd(const double& roll, const double& pitch, const double& yaw, bool verbose)
{
  base_angular_velocity_cmd_roll_  = roll;
  base_angular_velocity_cmd_pitch_ = pitch;
  base_angular_velocity_cmd_yaw_   = yaw;
  if(verbose)
    PRINT_INFO_NAMED(CLASS_NAME,"Set base angular velocity to "<<" "<<roll<<" "<<pitch<<" "<<yaw);
}

void FootholdsPlanner::setBaseLinearVelocityCmdX(const double &v)
{
  base_linear_velocity_cmd_x_ = v;
}

void FootholdsPlanner::setBaseLinearVelocityCmdY(const double &v)
{
  base_linear_velocity_cmd_y_ = v;
}

void FootholdsPlanner::setBaseLinearVelocityCmdZ(const double &v)
{
  base_linear_velocity_cmd_z_ = v;
}

void FootholdsPlanner::setBaseAngularVelocityCmdRoll(const double &v)
{
  base_angular_velocity_cmd_roll_  = v;
}

void FootholdsPlanner::setBaseAngularVelocityCmdPitch(const double &v)
{
  base_angular_velocity_cmd_pitch_  = v;
}

void FootholdsPlanner::setBaseAngularVelocityCmdYaw(const double &v)
{
  base_angular_velocity_cmd_yaw_  = v;
}

void FootholdsPlanner::setStepHeight(const double& height)
{
  if(height > step_height_max_) // Check if it is ok
  {
    double height_max = step_height_max_;
    step_height_ = height_max;
    PRINT_WARN_NAMED(CLASS_NAME,"Step height is greater than: "<<height_max);
  }
  else if(height <= 0.0)
  {
    step_height_ = 0.0;
    PRINT_WARN_NAMED(CLASS_NAME,"Step height is less equal than: 0.0");
  }
  else
  {
    step_height_ = height;
    PRINT_INFO_NAMED(CLASS_NAME,"Set step height to: "<<height);
  }
}

void FootholdsPlanner::setMaxStepHeight(const double& max)
{
  if(max >= 0.0) // Check if it is ok
  {
    step_height_max_ = max;
  }
  else
    PRINT_WARN_NAMED(CLASS_NAME,"Max step height is less equal than: 0.0");
}

void FootholdsPlanner::setMaxBaseHeight(const double& max)
{
  if(max >= 0.0) // Check if it is ok
  {
    base_height_max_ = max;
  }
  else
    PRINT_WARN_NAMED(CLASS_NAME,"Max base height is less equal than: 0.0");
}

void FootholdsPlanner::setMaxBaseRoll(const double &max)
{
  base_roll_max_ = max;
}

void FootholdsPlanner::setMaxBasePitch(const double &max)
{
  base_pitch_max_ = max;
}

void FootholdsPlanner::setMinBaseRoll(const double &min)
{
  base_roll_min_ = min;
}

void FootholdsPlanner::setMinBasePitch(const double &min)
{
  base_pitch_min_ = min;
}

void FootholdsPlanner::setMaxStepLength(const double& max)
{
  if(max >= 0.0) // Check if it is ok
  {
    step_length_max_ = max;
    push_recovery_->setMaxDelta(max);
  }
  else
    PRINT_WARN_NAMED(CLASS_NAME,"Max step length is less equal than: 0.0");
}

// Gets
unsigned int FootholdsPlanner::getCmd()
{
  return cmd_;
}

const Eigen::Vector3d& FootholdsPlanner::getBasePositionReference() const
{
  return base_position_reference_;
}

const Eigen::Vector3d& FootholdsPlanner::getBaseLinearVelocityReference() const
{
  return base_linear_velocity_reference_;
}

const Eigen::Vector3d& FootholdsPlanner::getBaseAngularVelocityReference() const
{
  return base_angular_velocity_reference_;
}

const Eigen::Vector3d& FootholdsPlanner::getBaseLinearVelocityReferenceHF() const
{
  return hf_base_linear_velocity_ref_;
}

const Eigen::Vector3d& FootholdsPlanner::getBaseAngularVelocityReferenceHF() const
{
  return hf_base_angular_velocity_ref_;
}

const Eigen::Matrix3d& FootholdsPlanner::getBaseRotationReference() const
{
  return base_rotation_reference_;
}

const double& FootholdsPlanner::getStepLength(const std::string& foot_name)
{
  return steps_length_[foot_name];
}

const double& FootholdsPlanner::getStepHeading(const std::string& foot_name)
{
  return steps_heading_[foot_name];
}

const double& FootholdsPlanner::getStepHeight(const std::string& foot_name)
{
  return steps_height_[foot_name];
}

const double& FootholdsPlanner::getStepHeadingRate(const std::string& foot_name)
{
  return steps_heading_rate_[foot_name];
}

const double& FootholdsPlanner::getBaseHeight() const
{
  return base_position_reference_(2);
}

double FootholdsPlanner::getBaseLinearVelocityCmdX() const
{
  return base_linear_velocity_cmd_x_;
}

double FootholdsPlanner::getBaseLinearVelocityCmdY() const
{
  return base_linear_velocity_cmd_y_;
}

double FootholdsPlanner::getBaseLinearVelocityCmdZ() const
{
  return base_linear_velocity_cmd_z_;
}

double FootholdsPlanner::getBaseAngularVelocityCmdRoll() const
{
  return base_angular_velocity_cmd_roll_;
}

double FootholdsPlanner::getBaseAngularVelocityCmdPitch() const
{
  return base_angular_velocity_cmd_pitch_;
}

double FootholdsPlanner::getBaseAngularVelocityCmdYaw() const
{
  return base_angular_velocity_cmd_yaw_;
}

double FootholdsPlanner::getStepHeight() const
{
  return step_height_;
}

double FootholdsPlanner::getStepLength() const
{
  return step_length_;
}

Gait::gait_t FootholdsPlanner::getGaitType() const
{
  return gait_generator_->getGaitType();
}

bool FootholdsPlanner::isAnyFootInTouchDown()
{
  return gait_generator_->isAnyFootInTouchDown();
}

bool FootholdsPlanner::isAnyFootInSwing()
{
  return gait_generator_->isAnyFootInSwing();
}

bool FootholdsPlanner::isGaitCycleEnded()
{
  return gait_generator_->isGaitCycleEnded();
}

PushRecovery* FootholdsPlanner::getPushRecovery() const
{
  return push_recovery_.get();
}

GaitGenerator* FootholdsPlanner::getGaitGenerator() const
{
  return gait_generator_.get();
}

bool FootholdsPlanner::areAllFeetInStance()
{
  return gait_generator_->areAllFeetInStance();
}

const std::vector<std::string>& FootholdsPlanner::getFootNames() const
{
  return gait_generator_->getFootNames();
}

double FootholdsPlanner::getSwingFrequency()
{
  return gait_generator_->getAvgSwingFrequency();
}

double FootholdsPlanner::getCycleTime()
{
  return gait_generator_->getAvgCycleTime();
}

double FootholdsPlanner::getVelocityFactor()
{
  return gait_generator_->getVelocityFactor();
}

double FootholdsPlanner::getPushRecoverySensibility()
{
  return (1.0 - push_recovery_->getScaleValue());
}

Eigen::Vector3d &FootholdsPlanner::getCurrentFoothold(const std::string &foot_name)
{
  return current_foothold_[foot_name];
}

Eigen::Vector3d &FootholdsPlanner::getCurrentFootholdHF(const std::string &foot_name)
{
  return current_foothold_hf_[foot_name];
}

Eigen::Vector3d &FootholdsPlanner::getVirtualFoothold(const std::string& foot_name)
{
  return virtual_foothold_[foot_name];
}

Eigen::Vector3d &FootholdsPlanner::getDesiredFoothold(const std::string& foot_name)
{
  return desired_foothold_[foot_name];
}

PushRecovery::PushRecovery(FootholdsPlanner* const footholds_planner_ptr)
{
  assert(footholds_planner_ptr);
  footholds_planner_ptr_ = footholds_planner_ptr;

  const std::vector<std::string>& foot_names = footholds_planner_ptr_->robot_model_->getFootNames();
  for(unsigned int i=0;i<foot_names.size();i++)
    deltas_[foot_names[i]].setZero();

  max_delta_ = (footholds_planner_ptr_->step_length_max_) / 1.5; //  x ~ L/sqrt(2)

  compute_deltas_    = true;
  push_detected_     = false;
  scale_ = 1.0;

  vertx_.resize(N_LEGS);
  verty_.resize(N_LEGS);
  support_polygon_edges_.resize(N_LEGS);
  footholds_planner_ptr_->robot_model_->getCOM(com_pos_);

  // Order the foot names such that they are in a consecutive order, this is needed by pnpoly
  feet_pos_ = footholds_planner_ptr_->robot_model_->getFeetPositionInWorld();
  ordered_foot_names_ = sortByLegPrefix(footholds_planner_ptr_->robot_model_->getFootNames(),{"lf","rf","rh","lh"});

  for(unsigned int i=0;i<ordered_foot_names_.size();i++)
  {
    vertx_[i] = 0.0;
    verty_[i] = 0.0;
    support_polygon_edges_[i] = Eigen::Vector2d::Zero();
  }

#ifdef DEBUG
  //RtLogger::getLogger().addPublisher(TOPIC(capture_point),capture_point_);
  //RtLogger::getLogger().addPublisher(TOPIC(com_vel),com_vel_);
  //RtLogger::getLogger().addPublisher(TOPIC(com_pos),com_pos_);
  for(unsigned int i=0;i<foot_names.size();i++)
    RtLogger::getLogger().addPublisher(_robot_name+"/wolf_controller/delta_"+foot_names[i],deltas_[foot_names[i]]);
#endif
}

bool PushRecovery::update(const double& period)
{

  // Get COM
  footholds_planner_ptr_->robot_model_->getCOMVelocity(com_vel_);
  footholds_planner_ptr_->robot_model_->getCOM(tmp_vector3d_);

  // Filter COM velocity around the cycle frequency because when lifting off the robot kind of shakes generating
  // unwanted velocities
  com_vel_filt_.setOmega(2.0*M_PI*1.0/footholds_planner_ptr_->gait_generator_->getAvgCycleTime());
  com_vel_filt_.setTimeStep(period);
  com_vel_ = com_vel_filt_.process(com_vel_);

  // Get COM position
  com_pos_(2) = tmp_vector3d_(2); // Take Z
  com_pos_.head(2) = com_pos_.head(2) + com_vel_.head(2) * period; // Integrate COM
  //com_pos_ = tmp_vector3d_; // Do not integrate
  com_pos_xy_ = com_pos_.head(2); // To export only

  // Get the foot positions wrt world
  feet_pos_ = footholds_planner_ptr_->robot_model_->getFeetPositionInWorld();

  // Calculate the center of the support polygon wrt world to compute the scaling
  tmp_vector3d_.setZero();
  for(unsigned int i=0; i <feet_pos_.size(); i++)
    tmp_vector3d_ = feet_pos_[ordered_foot_names_[i]] + tmp_vector3d_;
  tmp_vector3d_ = tmp_vector3d_/feet_pos_.size();

  // Update the support polygon
  if(footholds_planner_ptr_->gait_generator_->areAllFeetInStance())
  {
    for(unsigned int i=0;i<ordered_foot_names_.size();i++)
    {
      support_polygon_edges_[i] = tmp_vector3d_.head(2) + scale_ * (feet_pos_[ordered_foot_names_[i]].head(2) - tmp_vector3d_.head(2));
      vertx_[i] = static_cast<float>(support_polygon_edges_[i].x());
      verty_[i] = static_cast<float>(support_polygon_edges_[i].y());
    }
  }

  capture_point_ = footholds_planner_ptr_->com_planner_->getCapturePoint();

  // Reset COM integration if the gait cycle is ended
  if(gait_cycle_ended_.update(footholds_planner_ptr_->gait_generator_->isGaitCycleEnded()))
    com_pos_.head(2) = tmp_vector3d_.head(2);

  float testx = static_cast<float>(capture_point_.x());
  float testy = static_cast<float>(capture_point_.y());

  // Check if the capture point is outside the support polygon
  if(cartesian_utils::pnpoly(N_LEGS,vertx_.data(),verty_.data(),testx,testy)==0)
    push_detected_ = true;
  else
    push_detected_ = false;

  // Compute deltas
  if(compute_deltas_)
  {
    const std::vector<std::string>& foot_names = footholds_planner_ptr_->robot_model_->getFootNames();
    for(unsigned int i=0;i<foot_names.size();i++)
    {

      deltas_[foot_names[i]].x() = capture_point_(0) - tmp_vector3d_.x();
      deltas_[foot_names[i]].y() = capture_point_(1) - tmp_vector3d_.y();

      if(deltas_[foot_names[i]].x() > max_delta_)
        deltas_[foot_names[i]].x() = max_delta_;
      if(deltas_[foot_names[i]].x() < -max_delta_)
        deltas_[foot_names[i]].x() = -max_delta_;

      if(deltas_[foot_names[i]].y() > max_delta_)
        deltas_[foot_names[i]].y() = max_delta_;
      if(deltas_[foot_names[i]].y() < -max_delta_)
        deltas_[foot_names[i]].y() = -max_delta_;

      // Rotate to align with HF
      deltas_[foot_names[i]] = footholds_planner_ptr_->world_R_hf_.block(0,0,2,2).transpose() * deltas_[foot_names[i]];
    }
  }

  return push_detected_;
}

const Eigen::Vector2d &PushRecovery::getDelta(const std::string &foot_name)
{
  return deltas_[foot_name];
}

const std::vector<Eigen::Vector2d> &PushRecovery::getSupportPolygonEdges()
{
  return support_polygon_edges_;
}

const Eigen::Vector2d &PushRecovery::getCapturePoint()
{
  return capture_point_;
}

const Eigen::Vector2d &PushRecovery::getComPositionXY()
{
  return com_pos_xy_;
}

void PushRecovery::setMaxDelta(const double& max)
{
  if(max > 0)
    max_delta_ = max;
  else
    PRINT_WARN_NAMED(CLASS_NAME,"max delta must be positive!");
}

void PushRecovery::activateComputeDeltas()
{
  compute_deltas_ = true;
  PRINT_INFO_NAMED(CLASS_NAME,"Push recovery activated!");
}

void PushRecovery::deactivateComputeDeltas()
{
  compute_deltas_ = false;
  PRINT_INFO_NAMED(CLASS_NAME,"Push recovery de-activated!");
}

void PushRecovery::setScaleValue(double scale)
{
  scale_ = scale;
}

double PushRecovery::getScaleValue()
{
  return scale_;
}

const std::vector<std::string> &PushRecovery::getOrderedFootNames()
{
  return ordered_foot_names_;
}

void FootholdsPlanner::setVelocityFilterCutoffFrequency(double freq_hz)
{
  double omega = 2.0 * M_PI * freq_hz;
  linear_velocity_filter_.setOmega(omega);
  angular_velocity_filter_.setOmega(omega);
}

}; // namespace
