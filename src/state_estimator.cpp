/**
 * @file state_estimator.cpp
 * @author Gennaro Raiola
 * @date 12 June, 2019
 * @brief State estimator
 */

#include <wolf_controller_core/state_estimator.h>
#include <wolf_controller_core/common.h>
#include <wolf_controller_utils/geometry.h>

#include <unordered_map>

#include <wolf_estimation/estimation/kf_estimation_pinocchio.h>
#include <wolf_estimation/estimation/kf_estimation_rbdl.h>

// RT LOGGER
#ifdef RT_LOGGER
#include <rt_logger/rt_logger.h>
using namespace rt_logger;
#endif

using namespace wolf_controller_utils;
using namespace wolf_estimation;
using namespace wolf_wbid;

namespace wolf_controller {

std::string enumToString(const StateEstimator::estimation_t& estimation)
{
  std::string ret = "none";
  switch (estimation)
  {

  case StateEstimator::estimation_t::NONE:
    ret = "none";
    break;

  case StateEstimator::estimation_t::GROUND_TRUTH:
    ret = "ground_truth";
    break;

  case StateEstimator::estimation_t::IMU_GYROSCOPE:
    ret = "imu_gyroscope";
    break;

  case StateEstimator::estimation_t::IMU_MAGNETOMETER:
    ret = "imu_magnetometer";
    break;

  case StateEstimator::estimation_t::KALMAN_FILTER:
    ret = "kalman_filter";
    break;
  };

  return ret;
}

StateEstimator::estimation_t stringToEnum(const std::string& estimation)
{
  StateEstimator::estimation_t ret = StateEstimator::estimation_t::NONE;

  if (estimation == "none")
    ret = StateEstimator::estimation_t::NONE;
  else if(estimation == "ground_truth")
    ret = StateEstimator::estimation_t::GROUND_TRUTH;
  else if(estimation == "imu_gyroscope")
    ret = StateEstimator::estimation_t::IMU_GYROSCOPE;
  else if(estimation == "imu_magnetometer")
    ret = StateEstimator::estimation_t::IMU_MAGNETOMETER;
  else if(estimation == "kalman_filter")
    ret = StateEstimator::estimation_t::KALMAN_FILTER;
  else
    throw std::runtime_error("Wrong estimation type!");

  return ret;
}

StateEstimator::StateEstimator(StateMachine::Ptr state_machine, QuadrupedRobot::Ptr robot_model)
{

  assert(state_machine);
  state_machine_ = state_machine;

  assert(robot_model);
  robot_model_ = robot_model;

  const std::vector<std::string>& contact_names = robot_model_->getContactNames();
  const std::vector<std::string>& limb_names = robot_model_->getLimbNames();
  const std::vector<std::string>& foot_names = robot_model_->getFootNames();
  const std::vector<std::string>& leg_names = robot_model_->getLegNames();
  const std::vector<std::string>& ee_names = robot_model_->getEndEffectorNames();
  const std::vector<std::string>& arm_names = robot_model_->getArmNames();

  // KF estimator
  //kf_estimation_ = std::make_shared<KalmanFilterEstimatorPinocchio>(robot_model_->getUrdfString(),
  //                                                                 robot_model_->getSrdfString(),
  //                                                                 wolf_controller::_period);
  kf_estimation_ = std::make_shared<KalmanFilterEstimatorRbdl>(robot_model_->getUrdfString(),
                                                              robot_model_->getSrdfString(),
                                                              wolf_controller::_period);

  int n_dofs = robot_model_->getJointNum();
  joint_positions_.resize(static_cast<Eigen::Index>(n_dofs));
  joint_velocities_.resize(static_cast<Eigen::Index>(n_dofs));
  joint_efforts_.resize(static_cast<Eigen::Index>(n_dofs));
  floating_base_rpy_ = Eigen::Vector3d::Zero();
  floating_base_position_ = Eigen::Vector3d::Zero();
  floating_base_velocity_ = Eigen::Vector6d::Zero();
  floating_base_pose_ = Eigen::Affine3d::Identity();
  gt_position_ = Eigen::Vector3d::Zero();
  gt_orientation_ = Eigen::Quaterniond::Identity();
  gt_linear_velocity_ = Eigen::Vector3d::Zero();
  gt_angular_velocity_= Eigen::Vector3d::Zero();
  gt_linear_acceleration_ = Eigen::Vector3d::Zero();
  terrain_normal_ << 0,0,1;
  imu_orientation_.normalize();
  imu_gyroscope_.setZero();
  imu_accelerometer_.setZero();
  contact_computation_active_ = false;
  estimated_z_ = 0.0;

  for(unsigned int i=0;i<contact_names.size();i++)
  {
    contact_states_[contact_names[i]]   = true;
    contact_forces_[contact_names[i]]  = Eigen::Vector3d::Zero();
    world_X_contact_[contact_names[i]] = Eigen::Vector3d::Zero();
    base_X_contact_[contact_names[i]]  = Eigen::Vector3d::Zero();
  }

  map_rpy_derivatives_to_omega_ = Eigen::Matrix3d::Identity();
  world_R_base_ = Eigen::Matrix3d::Identity();
  raw_world_R_base_ = Eigen::Matrix3d::Identity();
  raw_base_rpy_ = Eigen::Vector3d::Zero();

  estimation_orientation_ = estimation_t::KALMAN_FILTER;
  estimation_position_ = estimation_t::KALMAN_FILTER;

  reset_gyro_integration_done_ = false;

  contact_force_th_ = 0.0; // [N]

  use_external_contact_forces_ = false;
  use_external_contact_states_ = false;

  // Contact force estimation reset
  force_estimation_ = std::make_shared<ForceEstimatorMomentumBased>(robot_model_,1.0/_period);
  //force_estimation_ = std::make_shared<XBot::Cartesian::Utils::ForceEstimation>(robot_model_->getXBotModel());

  // Contact estimation reset
  std::vector<int> dofs = {0,1,2}; // x y z
  std::vector<std::string> chain(1);

  // Build contact->limb mapping (feet->legs, ee->arms). Fallback to index if aligned.
  std::unordered_map<std::string, std::string> contact_to_limb;
  const size_t n_legs = std::min(foot_names.size(), leg_names.size());
  for(size_t i = 0; i < n_legs; ++i) {
    contact_to_limb[foot_names[i]] = leg_names[i];
  }
  const size_t n_arms = std::min(ee_names.size(), arm_names.size());
  for(size_t i = 0; i < n_arms; ++i) {
    contact_to_limb[ee_names[i]] = arm_names[i];
  }

  for(size_t i = 0; i < contact_names.size(); ++i)
  {
    auto it = contact_to_limb.find(contact_names[i]);
    if(it != contact_to_limb.end()) {
      chain[0] = it->second;
      force_torque_sensors_[contact_names[i]] = force_estimation_->add_link(contact_names[i],dofs,chain);
      continue;
    }

    // Fallback: use index alignment if provided by SRDF
    if(i < limb_names.size()) {
      chain[0] = limb_names[i];
      force_torque_sensors_[contact_names[i]] = force_estimation_->add_link(contact_names[i],dofs,chain);
      continue;
    }

    PRINT_WARN_NAMED(CLASS_NAME,
                     "No limb mapping for contact '" << contact_names[i]
                     << "'. Skipping force estimation for this contact.");
  }
#ifdef RT_LOGGER
  RtLogger::getLogger().addPublisher(TOPIC(floating_base_position)   ,floating_base_position_);
  RtLogger::getLogger().addPublisher(TOPIC(floating_base_rpy     )   ,floating_base_rpy_);
  RtLogger::getLogger().addPublisher(TOPIC(floating_base_velocity)   ,floating_base_velocity_);
  RtLogger::getLogger().addPublisher(TOPIC(base_height           )   ,floating_base_position_(2));
#endif
}

void StateEstimator::reset()
{
  floating_base_rpy_ = Eigen::Vector3d::Zero();
  floating_base_position_ = Eigen::Vector3d::Zero();
  floating_base_velocity_ = Eigen::Vector6d::Zero();
  floating_base_pose_ = Eigen::Affine3d::Identity();
  gt_position_ = Eigen::Vector3d::Zero();
  gt_orientation_ = Eigen::Quaterniond::Identity();
  gt_linear_velocity_ = Eigen::Vector3d::Zero();
  gt_angular_velocity_= Eigen::Vector3d::Zero();
  gt_linear_acceleration_ = Eigen::Vector3d::Zero();
  imu_orientation_.normalize();
  imu_gyroscope_.setZero();
  imu_accelerometer_.setZero();
  estimated_z_ = 0.0;

  kf_estimation_->reset();
}

void StateEstimator::setEstimationType(const std::string& position_t, const std::string& orientation_t)
{
  setPositionEstimationType(position_t);
  setOrientationEstimationType(orientation_t);
}

void StateEstimator::setPositionEstimationType(const std::string& position_t)
{
  setPositionEstimationType(stringToEnum(position_t));
}

void StateEstimator::setOrientationEstimationType(const std::string& orientation_t)
{
  setOrientationEstimationType(stringToEnum(orientation_t));
}

void StateEstimator::setEstimationType(estimation_t position_t, estimation_t orientation_t)
{
  estimation_orientation_ = orientation_t;
  estimation_position_ = position_t;
}

void StateEstimator::setPositionEstimationType(estimation_t position_t)
{
  estimation_position_ = position_t;
}

void StateEstimator::setOrientationEstimationType(estimation_t orientation_t)
{
  estimation_orientation_ = orientation_t;
}

void StateEstimator::setJointPosition(const Eigen::VectorXd& joint_positions)
{
  joint_positions_ = joint_positions;
}

void StateEstimator::setJointVelocity(const Eigen::VectorXd& joint_velocities)
{
  joint_velocities_ = joint_velocities;
}

void StateEstimator::setJointEffort(const Eigen::VectorXd& joint_efforts)
{
  joint_efforts_ = joint_efforts;
}

void StateEstimator::setTerrainNormal(const Eigen::Vector3d &terrain_normal)
{
  terrain_normal_ = terrain_normal;
}

void StateEstimator::setImuOrientation(const Eigen::Quaterniond& imu_orientation)
{
  imu_orientation_ = imu_orientation.normalized();
}

void StateEstimator::setImuGyroscope(const Eigen::Vector3d& imu_gyroscope)
{
  imu_gyroscope_ = imu_gyroscope;
}

void StateEstimator::setImuAccelerometer(const Eigen::Vector3d& imu_accelerometer)
{
  imu_accelerometer_ = imu_accelerometer;
}

void StateEstimator::setGroundTruthBasePosition(const Eigen::Vector3d& gt_position)
{
  gt_position_ = gt_position;
}

void StateEstimator::setGroundTruthBaseOrientation(const Eigen::Quaterniond& gt_orientation)
{
  gt_orientation_ = gt_orientation;
}

void StateEstimator::setGroundTruthBaseLinearVelocity(const Eigen::Vector3d& gt_linear_velocity)
{
  gt_linear_velocity_ = gt_linear_velocity;
}

void StateEstimator::setGroundTruthBaseAngularVelocity(const Eigen::Vector3d& gt_angular_velocity)
{
  gt_angular_velocity_ = gt_angular_velocity;
}

void StateEstimator::setGroundTruthBaseLinearAcceleration(const Eigen::Vector3d& gt_linear_acceleration)
{
  gt_linear_acceleration_ = gt_linear_acceleration;
}

void StateEstimator::setContactState(const std::string &name, const bool &state)
{
  if(use_external_contact_states_ == false)
    use_external_contact_states_ = true;

  if(contact_states_.count(name)> 0)
  {
    contact_states_[name] = state;
  }
  else
  {
    PRINT_WARN_NAMED(CLASS_NAME,"Wrong contact name!");
  }
}

void StateEstimator::setContactThreshold(const double& th)
{
  contact_force_th_ = th;
}

double StateEstimator::getContactThreshold()
{
  return contact_force_th_;
}

std::string StateEstimator::getPositionEstimationType()
{
  return enumToString(estimation_position_);
}

std::string StateEstimator::getOrientationEstimationType()
{
  return enumToString(estimation_orientation_);
}

void StateEstimator::setContactForce(const std::string& name, const Eigen::Vector3d& force)
{
  if(use_external_contact_forces_ == false)
    use_external_contact_forces_ = true;

  if(contact_forces_.count(name)> 0)
  {
    contact_forces_[name] = force;
  }
  else
  {
    PRINT_WARN_NAMED(CLASS_NAME,"Wrong contact name!");
  }
}

void StateEstimator::setDesiredContactState(const std::string &name, const bool &state)
{
  des_contact_states_[name] = state;
}

void StateEstimator::setDesiredContactForce(const std::string &name, const Eigen::Vector3d &force)
{
  des_contact_forces_[name] = force;
}

const Eigen::Affine3d& StateEstimator::getFloatingBasePose() const
{
  return floating_base_pose_;
}

const Eigen::Vector3d& StateEstimator::getFloatingBasePosition() const
{
  return floating_base_position_;
}

const Eigen::Vector3d& StateEstimator::getFloatingBaseOrientationRPY() const
{
  return floating_base_rpy_;
}

const Eigen::Vector6d &StateEstimator::getFloatingBaseTwist() const
{
  return floating_base_velocity_;
}

const std::map<std::string,Eigen::Vector3d>& StateEstimator::getContactForces() const
{
  return contact_forces_;
}

const std::map<std::string,bool>& StateEstimator::getContacts() const
{
  return contact_states_;
}

Eigen::Vector3d& StateEstimator::getContactForce(const std::string& contact_name)
{
  return contact_forces_[contact_name];
}

bool StateEstimator::getContact(const std::string& contact_name)
{
  return contact_states_[contact_name];
}

const std::map<std::string,Eigen::Vector3d>& StateEstimator::getContactPositionInWorld() const
{
  return world_X_contact_;
}

const std::map<std::string,Eigen::Vector3d>& StateEstimator::getContactPositionInBase() const
{
  return base_X_contact_;
}

const Eigen::Vector3d& StateEstimator::getGroundTruthBasePosition() const
{
  return gt_position_;
}

const Eigen::Quaterniond& StateEstimator::getGroundTruthBaseOrientation() const
{
  return gt_orientation_;
}

const Eigen::Vector3d& StateEstimator::getGroundTruthBaseLinearVelocity() const
{
  return gt_linear_velocity_;
}

const Eigen::Vector3d& StateEstimator::getGroundTruthBaseAngularVelocity() const
{
  return gt_angular_velocity_;
}

const double& StateEstimator::getBaseHeightInBasefoot() const
{
  return estimated_z_;
}

const double& StateEstimator::getBaseHeightInWorld() const
{
  return floating_base_position_(2);
}

void StateEstimator::startContactComputation()
{
  contact_computation_active_ = true;

  //ROS_DEBUG_NAMED(CLASS_NAME,"Start contact computation");
}

void StateEstimator::resetGyroscopeIntegration()
{
  reset_gyro_integration_done_ = false;
}

bool StateEstimator::isAnyFootInContact()
{
  const std::vector<std::string>& foot_names = robot_model_->getFootNames();
  bool result = false;
  for(unsigned int i=0; i<foot_names.size(); i++)
    result = result || contact_states_[foot_names[i]];
  return result;
}

bool StateEstimator::areAllFeetInContact()
{
  const std::vector<std::string>& foot_names = robot_model_->getFootNames();
  bool result = true;
  for(unsigned int i=0; i<foot_names.size(); i++)
    result = result && contact_states_[foot_names[i]];
  return result;
}

void StateEstimator::stopContactComputation()
{
  contact_computation_active_ = false;

  //ROS_DEBUG_NAMED(CLASS_NAME,"Stop contact computation: the contact states are forced to TRUE");
}

void StateEstimator::update(const double& period)
{
  const std::vector<std::string>& foot_names = robot_model_->getFootNames();
  const std::vector<std::string>& ee_names = robot_model_->getEndEffectorNames();

  for(unsigned int i=0; i<foot_names.size(); i++)
  {
    world_X_contact_[foot_names[i]] = robot_model_->getFootPositionInWorld(foot_names[i]);
    base_X_contact_[foot_names[i]] = robot_model_->getFootPositionInBase(foot_names[i]);
  }

  for(unsigned int i=0; i<ee_names.size(); i++)
  {
    world_X_contact_[ee_names[i]] = robot_model_->getEndEffectorPositionInWorld(ee_names[i]);
    base_X_contact_[ee_names[i]] = robot_model_->getEndEffectorPositionInBase(ee_names[i]);
  }

  updateContactState();

  updateFloatingBase(period);
}

void StateEstimator::updateContactState()
{
  force_estimation_->update();

  // Update contact state for the feet
  const std::vector<std::string>& foot_names = robot_model_->getFootNames();
  for(unsigned int i=0; i<foot_names.size(); i++)
  {
    tmp_vector3d_.setZero(); // contact_force_foot
    auto it = force_torque_sensors_.find(foot_names[i]);
    if(it == force_torque_sensors_.end() || !it->second)
    {
      contact_states_[foot_names[i]] = false;
      contact_forces_[foot_names[i]].setZero();
      continue;
    }
    // If we don't have a measurement of the contact forces, estimate them
    if(!use_external_contact_forces_)
      it->second->getForce(tmp_vector3d_);
    else
      tmp_vector3d_ = contact_forces_[foot_names[i]];

    // Transform the local foot force in world
    contact_forces_[foot_names[i]] = robot_model_->getFootPoseInWorld(foot_names[i]) * tmp_vector3d_; // contact_force_world = world_T_foot * contact_force_foot

    if(contact_computation_active_)
    {
      // If we don't have a measurement of the contact states, estimate them with a threshold
      if(!use_external_contact_states_)
        contact_states_[foot_names[i]] = (contact_forces_[foot_names[i]].dot(terrain_normal_) >= contact_force_th_ ? true : false);
    }
    else
    {
      contact_states_[foot_names[i]] = true;
    }

  }

  // Update contact state for the arm end-effectors
  const std::vector<std::string>& ee_names = robot_model_->getEndEffectorNames();
  for(unsigned int i=0; i<ee_names.size(); i++)
  {

    tmp_vector3d_.setZero();
    auto it = force_torque_sensors_.find(ee_names[i]);
    if(it == force_torque_sensors_.end() || !it->second)
    {
      contact_states_[ee_names[i]] = false;
      contact_forces_[ee_names[i]].setZero();
      continue;
    }
    it->second->getForce(tmp_vector3d_); // tmp_vector3d_ = contact_force_arm

    tmp_vector3d_ = robot_model_->getEndEffectorPoseInWorld(ee_names[i]) * tmp_vector3d_; // contact_force_world = world_T_ee * contact_force_ee

    if(contact_computation_active_)
      contact_states_[ee_names[i]] = (tmp_vector3d_.norm() >= contact_force_th_ ? true : false);
    else
      contact_states_[ee_names[i]] = false;

    contact_forces_[ee_names[i]] = tmp_vector3d_;
  }

}

double StateEstimator::estimateZ()
{
  // Estimate z using the legs position
  const std::vector<std::string>& foot_names = robot_model_->getFootNames();
  double estimated_z = 0.0;
  int feet_in_stance = 0;
  tmp_affine3d_.setIdentity();
  for(unsigned int i = 0; i<foot_names.size(); i++)
  {
    if(contact_states_[foot_names[i]])
    {
      feet_in_stance++;
      tmp_affine3d_.translation() = floating_base_pose_.linear() *  robot_model_->getFootPositionInBase(foot_names[i]);
      estimated_z +=  tmp_affine3d_.translation().z();
    }
  }
  if(feet_in_stance!=0)
    estimated_z /= feet_in_stance;
  return estimated_z;
}

void StateEstimator::updateFloatingBase(const double& period)
{
  estimation_t estimation_orientation = estimation_orientation_;
  estimation_t estimation_position = estimation_position_;

  // Update the joints information of the virtual model
  robot_model_->setJointVelocity(joint_velocities_);
  robot_model_->setJointEffort(joint_efforts_);
  robot_model_->setJointPosition(joint_positions_);

  // Update the KF
  if(estimation_position == KALMAN_FILTER || estimation_orientation == KALMAN_FILTER)
  {
    if(!kf_estimation_->isInitialized())
    {
      kf_estimation_->init(joint_positions_,joint_velocities_,floating_base_pose_);
    }
    kf_estimation_->updateJoints(joint_positions_,joint_velocities_);
    kf_estimation_->updateImu(imu_orientation_,imu_gyroscope_,imu_accelerometer_,false);

    for(unsigned int i = 0; i<robot_model_->getFootNames().size(); i++)
    {
      kf_estimation_->updateContact(i,contact_states_[robot_model_->getFootNames()[i]]);
      kf_estimation_->updateContactForce(i,contact_forces_[robot_model_->getFootNames()[i]]);
    }

    kf_estimation_->update();
  }

  // Note: we assume that the IMU is orientated as the base/waist of the robot
  // if this is not the case, it is necessary to add a transfomation from the IMU frame to the
  // base/trunk frame

  switch(estimation_orientation)
  {
  case estimation_t::NONE:
    // The base does not rotate
    floating_base_pose_.linear() = Eigen::Matrix3d::Identity();
    floating_base_velocity_.segment(3,3) << 0.0, 0.0, 0.0;
    rotToRpy(floating_base_pose_.linear(),floating_base_rpy_);
    break;
  case estimation_t::KALMAN_FILTER:
    floating_base_pose_.linear() = kf_estimation_->getOrientation().toRotationMatrix();
    floating_base_velocity_.segment(3,3) << kf_estimation_->getAngularVelocity();
    rotToRpy(floating_base_pose_.linear(),floating_base_rpy_);
    break;
  case estimation_t::IMU_MAGNETOMETER: // Use directly the orientation information from the IMU
    //use this with the real robot only if the magnetometer is not drifting
    quatToRot(imu_orientation_.normalized(),world_R_base_);
    rotToRpy(world_R_base_,floating_base_rpy_);
    floating_base_pose_.linear() = world_R_base_;
#ifdef ANGULAR_VELOCITIES_WRT_BASE
    floating_base_velocity_.segment(3,3) = floating_base_pose_.linear() * imu_gyroscope_;
#else
    floating_base_velocity_.segment(3,3) = imu_gyroscope_;
#endif
    break;
  case estimation_t::IMU_GYROSCOPE: // Intergate the gyroscope, useful if the magnetometer measure has interferences
    if(!reset_gyro_integration_done_) // Initialize the integration with the measured orientation
    {
      // Initialization for the integration
      quatToRot(imu_orientation_.normalized(),world_R_base_);
      rotToRpy(world_R_base_,floating_base_rpy_);
      reset_gyro_integration_done_ = true;
    }
#ifdef ANGULAR_VELOCITIES_WRT_BASE
    rpyToEarBase(floating_base_rpy_,map_rpy_derivatives_to_omega_);
#else
    rpyToEarWorld(base_rpy_,mapRPYderivativesToOmega_);
#endif
    // Map the omegas in the base into rpy derivatives and integrate
    floating_base_rpy_ += (map_rpy_derivatives_to_omega_.inverse() * imu_gyroscope_) * period;
    // Overwrite measures if one of them is more noisy (e.g. only yaw is noisy)
    //quatToRotMat(imu_orientation_.normalized(),raw_base_R_world_);
    //rotTorpy(raw_base_R_world_,raw_base_rpy_);
    //base_rpy_.head(2) = raw_base_rpy_.head(2);
    //set the affine transformation for angular position
    rpyToRot(floating_base_rpy_, world_R_base_);
    floating_base_pose_.linear() = world_R_base_;
    // Set the affine transformation for angular velocity
#ifdef ANGULAR_VELOCITIES_WRT_BASE
    floating_base_velocity_.segment(3,3) = world_R_base_ * imu_gyroscope_;
#else
    floating_base_velocity_.segment(3,3) = imu_gyroscope_;
#endif
    break;
  case estimation_t::GROUND_TRUTH:
    quatToRot(gt_orientation_.normalized(),world_R_base_);
    rotToRpy(world_R_base_,floating_base_rpy_);
    floating_base_pose_.linear() = world_R_base_;
    floating_base_velocity_.segment(3,3) = gt_angular_velocity_;
    break;
  default:
    // The base does not rotate
    floating_base_pose_.linear() = Eigen::Matrix3d::Identity();
    floating_base_velocity_.segment(3,3) << 0.0, 0.0, 0.0;
    break;
  };

  //// Reset the imu one time so that the world and the base are aligned
  ////tmp_matrix3d_ = world_R_imu_init_.transpose() * tmp_matrix3d_; // imu_R_world = R_imu0' * R_imu
  ////quatToRpy(floating_base_orientation_.normalized(),floating_base_orientation_rpy_);//take rpy measures

  // Update the orientation part of the floating base
  robot_model_->setFloatingBaseOrientation(floating_base_pose_.linear());
  robot_model_->setFloatingBaseAngularVelocity(floating_base_velocity_.segment(3,3));
  robot_model_->update();

  estimated_z_ = -estimateZ();

  switch(estimation_position)
  {
  case estimation_t::NONE:
    // The base does not move
    floating_base_velocity_.segment(0,3) << 0.0,0.0,0.0;
    floating_base_position_ << 0.0,0.0,0.0;
    break;
  case estimation_t::KALMAN_FILTER:
    floating_base_position_ = kf_estimation_->getPosition();
    floating_base_velocity_.segment(0,3) = kf_estimation_->getLinearVelocity();
    break;
  case estimation_t::GROUND_TRUTH:
    floating_base_velocity_.segment(0,3) << gt_linear_velocity_;
    floating_base_position_ = gt_position_;
    break;
  default:
    // The base does not move
    floating_base_velocity_.segment(0,3) << 0.0,0.0,0.0;
    floating_base_position_ << 0.0,0.0,0.0;
    break;
  };

  // Finally update the floating base with the full pose and velocities
  floating_base_pose_.translation() = floating_base_position_;
  robot_model_->setFloatingBaseState(floating_base_pose_,floating_base_velocity_); // This should trigger the update of the model
  robot_model_->update();

}

}
