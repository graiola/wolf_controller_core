/**
 * @file controller.cpp
 * @author Gennaro Raiola
 * @date 12 June, 2019
 * @brief This file contains the constructor, destructor, init, stopping and other facilities for the
 * WoLF controller.
 */

// WoLF
#include <wolf_controller_core/controller_core.h>
#include <wolf_controller_core/state_machine.h>

// WoLF controller utils
#include <wolf_controller_utils/tools.h>

// RT GUI
#ifdef RT_GUI
#include <rt_gui_ros/rt_gui_client.h>
using namespace rt_gui;
#endif

using namespace XBot;
using namespace wolf_controller_utils;
using namespace wolf_wbid;

namespace wolf_controller {

std::vector<std::string> _dof_names = {}; // To be loaded from the robot model
std::vector<std::string> _cartesian_names = {"x","y","z","roll","pitch","yaw"}; // This is our standard cartesian dofs order
std::vector<std::string> _xyz = {"x","y","z"};
std::vector<std::string> _rpy = {"roll","pitch","yaw"};
std::vector<std::string> _joints_prefix = {"haa","hfe","kfe"};
std::vector<std::string> _legs_prefix = {"lf","lh","rf","rh"};
double _period = 0.001;
std::string _robot_name         = "";
std::string _robot_model_name   = "";
std::string _tf_prefix          = "";
std::string _rt_gui_group       = "";

std::string enumToString(ControllerCore::mode_t mode)
{
  std::string ret = "WPG";
  switch (mode)
  {
  case ControllerCore::mode_t::WPG:
    ret = "WPG";
    break;

  case ControllerCore::mode_t::EXT:
    ret = "EXT";
    break;

  case ControllerCore::mode_t::MPC:
    ret = "MPC";
    break;

  case ControllerCore::mode_t::RESET:
    ret = "RESET";
    break;
  };
  return ret;
}

ControllerCore::ControllerCore()
  :current_mode_(WPG)
  ,requested_mode_(WPG)
  ,previous_mode_(WPG)
  ,posture_(DOWN)
{
  XBot::Logger::SetVerbosityLevel(XBot::Logger::Severity::HIGH);
}

ControllerCore::~ControllerCore()
{
}

bool ControllerCore::init(const double& period, const std::string& urdf, const std::string& srdf, const std::string& robot_name)
{
  PRINT_INFO_NAMED(CLASS_NAME,"Initialize");

  _period = period_ = period;

  robot_name_ = robot_name;

  _robot_name = robot_name_;
  if(_robot_name.empty())
    _rt_gui_group = "controller";
  else
    _rt_gui_group = "controller/"+_robot_name;

  // Create the robot model
  robot_model_ = std::make_shared<wolf_wbid::QuadrupedRobot>(robot_name,urdf,srdf);
  n_joints_    = robot_model_->getJointNum();
  joint_names_ = robot_model_->getJointNames();

  PRINT_INFO_NAMED(CLASS_NAME,"Number of joints: "<<n_joints_);

  _robot_model_name = robot_model_->getRobotModelName();

  // Resize the variables
  joint_positions_.resize(static_cast<Eigen::Index>(n_joints_));
  joint_positions_init_.resize(static_cast<Eigen::Index>(n_joints_));
  joint_velocities_.resize(static_cast<Eigen::Index>(n_joints_));
  joint_velocities_filt_.resize(static_cast<Eigen::Index>(n_joints_));
  joint_accellerations_.resize(static_cast<Eigen::Index>(n_joints_));
  joint_efforts_.resize(static_cast<Eigen::Index>(n_joints_));
  des_joint_positions_.resize(static_cast<Eigen::Index>(n_joints_));
  des_joint_velocities_.resize(static_cast<Eigen::Index>(n_joints_));
  des_joint_efforts_.resize(static_cast<Eigen::Index>(n_joints_));
  des_joint_efforts_solver_.resize(static_cast<Eigen::Index>(n_joints_));
  des_joint_efforts_impedance_.resize(static_cast<Eigen::Index>(n_joints_));
  des_contact_forces_.resize(robot_model_->getContactNames().size(),Eigen::Vector6d::Zero());
  des_contact_states_.resize(robot_model_->getContactNames().size(),false);

  // Initializations
  joint_positions_.fill(0.0);
  joint_velocities_.fill(0.0);
  joint_velocities_filt_.fill(0.0);
  joint_accellerations_.fill(0.0);
  joint_efforts_.fill(0.0);
  des_joint_positions_.fill(0.0);
  des_joint_velocities_.fill(0.0);
  des_joint_efforts_.fill(0.0);
  des_joint_efforts_impedance_.fill(0.0);
  des_joint_efforts_solver_.fill(0.0);
  imu_orientation_.normalize();
  imu_gyroscope_.fill(0.0);
  imu_accelerometer_.fill(0.0);
  imu_gyroscope_filt_.fill(0.0);
  imu_accelerometer_filt_.fill(0.0);

  state_machine_ = std::make_shared<StateMachine>(this);

  state_estimator_   = std::make_shared<StateEstimator>(state_machine_,robot_model_);

  terrain_estimator_ = std::make_shared<TerrainEstimator>(state_estimator_,robot_model_);
  terrain_estimator_->setMaxRoll(M_PI);
  terrain_estimator_->setMinRoll(-M_PI);
  terrain_estimator_->setMaxPitch(M_PI);
  terrain_estimator_->setMinPitch(-M_PI);

  gait_generator_ = std::make_shared<GaitGenerator>(robot_model_->getFootNames(),Gait::TROT);
  com_planner_ = std::make_shared<ComPlanner>(robot_model_,gait_generator_,terrain_estimator_);
  foot_holds_planner_ = std::make_shared<FootholdsPlanner>(state_machine_,com_planner_,gait_generator_,robot_model_);


  impedance_     = std::make_shared<Impedance>(state_estimator_,robot_model_);
  impedance_->startInertiaCompensation(false);

  PRINT_INFO_NAMED(CLASS_NAME,"Create ID Problem...");
  id_prob_ = std::make_unique<IDProblem>(robot_model_);
  PRINT_INFO_NAMED(CLASS_NAME,"...done!");

  solver_failures_cnt_   = std::make_shared<Counter>(static_cast<int>(std::ceil(0.5 / period_)));
  contact_failures_cnt_  = std::make_shared<Counter>(static_cast<int>(std::ceil(0.5 / period_)));
  for(unsigned int i=0;i<joint_velocities_.size();i++)
    velocity_lims_failures_cnt_.push_back(std::make_shared<Counter>(static_cast<int>(std::ceil(0.5 / period_))));

#ifdef RT_GUI
  // create interface
  RtGuiClient::getIstance().init("wolf_rviz","wolf_controller_gui");
  RtGuiClient::getIstance().addLabel(std::string(wolf_controller::_rt_gui_group),std::string("Control mode"),&mode_string_);
  RtGuiClient::getIstance().addTrigger(std::string(wolf_controller::_rt_gui_group),std::string("Stand up"),std::bind(&wolf_controller::ControllerCore::standUp,this,true));
  RtGuiClient::getIstance().addTrigger(std::string(wolf_controller::_rt_gui_group),std::string("Stand down"),std::bind(&wolf_controller::ControllerCore::standUp,this,false));
  RtGuiClient::getIstance().addTrigger(std::string(wolf_controller::_rt_gui_group),std::string("Emergency stop"),std::bind(&wolf_controller::ControllerCore::emergencyStop,this));
#endif

  PRINT_INFO_NAMED(CLASS_NAME,"Initialization done!");

  return true;
}

bool ControllerCore::setFrictionConesMu(const double& mu)
{
  if(id_prob_)
  {
    if(mu>=0.0 && mu<=1.0)
    {
      id_prob_->setFrictionConesMu(mu);
      PRINT_INFO_NAMED(CLASS_NAME,"Set mu to: "<<mu);
    }
    else
    {
      PRINT_WARN_NAMED(CLASS_NAME,"Mu has to be between 0 and 1!");
      return false;
    }
  }
  else
  {
    PRINT_WARN_NAMED(CLASS_NAME,"Did you press start?");
    return false;
  }
  return true;
}


void ControllerCore::setCutoffFreqQdot(const double &hz)
{
  if(hz >= 0.0)
  {
    qdot_filter_.setOmega(2.0*M_PI*hz);
    PRINT_INFO_NAMED(CLASS_NAME,"Set cutoff frequency for qdot filter at "<< hz);
  }
  else
    PRINT_WARN_NAMED(CLASS_NAME,"Cutoff frequency has to be >= 0.0 [Hz]!");
}

void ControllerCore::setCutoffFreqGyroscope(const double &hz)
{
  if(hz >= 0.0)
  {
    imu_gyroscope_filter_.setOmega(2.0*M_PI*hz);
    PRINT_INFO_NAMED(CLASS_NAME,"Set cutoff frequency for imu gyroscope filter at "<< hz);
  }
  else
    PRINT_WARN_NAMED(CLASS_NAME,"Cutoff frequency has to be >= 0.0 [Hz]!");
}

void ControllerCore::setCutoffFreqAccelerometer(const double &hz)
{
  if(hz >= 0.0)
  {
    imu_accelerometer_filter_.setOmega(2.0*M_PI*hz);
    PRINT_INFO_NAMED(CLASS_NAME,"Set cutoff frequency for imu accelerometer filter at "<< hz);
  }
  else
    PRINT_WARN_NAMED(CLASS_NAME,"Cutoff frequency has to be >= 0.0 [Hz]!");
}

bool ControllerCore::setSwingFrequency(const double& swing_frequency)
{
  const std::vector<std::string>& foot_names = robot_model_->getFootNames();
  if(gait_generator_)
  {
    for(unsigned int i=0; i < foot_names.size(); i++)
      gait_generator_->setSwingFrequency(foot_names[i],swing_frequency);
    PRINT_INFO_NAMED(CLASS_NAME,"Set swing frequency to "<< swing_frequency);
  }
  else
  {
    PRINT_WARN_NAMED(CLASS_NAME,"gait_generator not initialized yet.");
    return false;
  }
  return true;
}

bool ControllerCore::setStepHeight(const double& step_height)
{
  if(foot_holds_planner_)
    foot_holds_planner_->setStepHeight(step_height);
  else
  {
    PRINT_WARN_NAMED(CLASS_NAME,"foot_holds_planner not initialized yet.");
    return false;
  }
  return true;
}

bool ControllerCore::selectControlMode(const std::string& mode)
{
  if(mode == "WPG")
    requested_mode_ = ControllerCore::mode_t::WPG;
  else if(mode == "EXT")
    requested_mode_ = ControllerCore::mode_t::EXT;
  else if(mode == "MPC")
    requested_mode_ = ControllerCore::mode_t::MPC;
  else if(mode == "RESET")
    requested_mode_ = ControllerCore::mode_t::RESET;
  else
  {
    PRINT_ERROR_NAMED(CLASS_NAME,"Wrong control mode!");
    return false;
  }
  PRINT_INFO_NAMED(CLASS_NAME,"Selected control mode "<< mode);

  return true;
}

unsigned int ControllerCore::getControlMode()
{
  return current_mode_;
}

void ControllerCore::switchControlMode()
{
  if(requested_mode_ == ControllerCore::mode_t::WPG)
    requested_mode_ = ControllerCore::mode_t::EXT;
  else
    requested_mode_ = ControllerCore::mode_t::WPG;
}

bool ControllerCore::selectPosture(const std::string& posture)
{
  if(posture == "UP")
  {
    posture_ = ControllerCore::posture_t::UP;
  }
  else if(posture == "DOWN")
    posture_ = ControllerCore::posture_t::DOWN;
  else
  {
    PRINT_ERROR_NAMED(CLASS_NAME,"Wrong posture!");
    return false;
  }
  PRINT_INFO_NAMED(CLASS_NAME,"Selected posture "<< posture);

  return true;
}

ControllerCore::posture_t ControllerCore::getPosture()
{
  return posture_;
}

void ControllerCore::switchPosture()
{
  if(posture_ == ControllerCore::posture_t::UP)
    posture_ = ControllerCore::posture_t::DOWN;
  else
    posture_ = ControllerCore::posture_t::UP;
}

void ControllerCore::standUp(bool stand_up)
{
  if(stand_up)
    posture_ = ControllerCore::posture_t::UP;
  else
    posture_ = ControllerCore::posture_t::DOWN;
}

void ControllerCore::resetBase()
{
  requested_mode_ = ControllerCore::mode_t::RESET;
}

void ControllerCore::emergencyStop()
{
  state_machine_->setCurrentState(StateMachine::ANOMALY);
}

bool ControllerCore::selectGait(const string& gait)
{
  if(gait_generator_)
  {
    if(gait == "TROT")
      gait_generator_->setGaitType(Gait::gait_t::TROT);
    else if(gait == "CRAWL")
      gait_generator_->setGaitType(Gait::gait_t::CRAWL);
    else
    {
      PRINT_ERROR_NAMED(CLASS_NAME,"Wrong gait!");
      return false;
    }
    PRINT_INFO_NAMED(CLASS_NAME,"Selected gait "<< gait);
  }
  return true;
}

void ControllerCore::switchGait()
{
  if(gait_generator_)
  {
    gait_generator_->switchGait();
  }
  else
    PRINT_WARN_NAMED(CLASS_NAME,"GaitGenerator not ready yet!");
}

bool ControllerCore::setDutyFactor(const double& duty_factor)
{
  if(duty_factor>=0.0 && duty_factor<=1.0 && gait_generator_)
  {
    gait_generator_->setDutyFactor(duty_factor);
    PRINT_INFO_NAMED(CLASS_NAME,"setDutyFactor: set the duty factor to "<<duty_factor);
    return true;
  }
  else
  {
    PRINT_WARN_NAMED(CLASS_NAME,"setDutyFactor: duty factor has to be between 0 and 1!");
    return false;
  }
}

void ControllerCore::setJointState(const Eigen::VectorXd& pos, const Eigen::VectorXd& vel, const Eigen::VectorXd& acc, const Eigen::VectorXd& effort)
{
  //joint_positions_.setZero();
  //joint_velocities_.setZero();
  //joint_velocities_filt_.setZero();
  //joint_accellerations_.setZero();
  //joint_efforts_.setZero();
  for (int i = 0; i < n_joints_; i++)
  {
    joint_positions_(i+FLOATING_BASE_DOFS) = pos(i);
    joint_velocities_(i+FLOATING_BASE_DOFS) = vel(i);
    joint_accellerations_(i+FLOATING_BASE_DOFS) = acc(i);
    joint_efforts_(i+FLOATING_BASE_DOFS) = effort(i);
  }
}

void ControllerCore::setJointPosition(const unsigned int &i, const double &value)
{
  joint_positions_(i) = value;
}

void ControllerCore::setJointVelocity(const unsigned int &i, const double &value)
{
  joint_velocities_(i) = value;
}

void ControllerCore::setJointAcceleration(const unsigned int &i, const double &value)
{
  joint_accellerations_(i) = value;
}

void ControllerCore::setJointEffort(const unsigned int &i, const double &value)
{
  joint_efforts_(i) = value;
}

void ControllerCore::setImu(const Eigen::Quaterniond& q, const Eigen::Vector3d& gyro, const Eigen::Vector3d& acc)
{
  imu_accelerometer_ = acc;
  imu_gyroscope_     = gyro;
  imu_orientation_   = q;
}

void ControllerCore::setImuOrientation(const Eigen::Quaterniond& q)
{
  imu_orientation_ = q;
}

void ControllerCore::setImuGyroscope(const Eigen::Vector3d& gyro)
{
  imu_gyroscope_ = gyro;
}

void ControllerCore::setImuAccelerometer(const Eigen::Vector3d& acc)
{
  imu_accelerometer_ = acc;
}

void ControllerCore::setExtEstimatedState(const Eigen::Vector3d& lin_pos,
                                          const Eigen::Vector3d& lin_vel,
                                          const Eigen::Vector3d& lin_acc,
                                          const Eigen::Quaterniond& q,
                                          const Eigen::Vector3d& ang_vel)
{
  // Linear
  state_estimator_->setGroundTruthBasePosition(lin_pos);
  state_estimator_->setGroundTruthBaseLinearVelocity(lin_vel);
  state_estimator_->setGroundTruthBaseLinearAcceleration(lin_acc);
  // Orientation
  state_estimator_->setGroundTruthBaseOrientation(q);
  state_estimator_->setGroundTruthBaseAngularVelocity(ang_vel);
}

void ControllerCore::setExtEstimatedContactStates(const std::map<std::string,std::pair<bool,Eigen::Vector3d> >& states)
{
  for(const auto& tmp : states)
  {
    state_estimator_->setContactForce(tmp.first,tmp.second.second);
    state_estimator_->setContactState(tmp.first,tmp.second.first);
  }
}

void ControllerCore::setExtEstimatedContactState(const std::string& contact_name, const bool& state, const Eigen::Vector3d& force)
{
  state_estimator_->setContactForce(contact_name,force);
  state_estimator_->setContactState(contact_name,state);
}

void ControllerCore::updateStateEstimator(const double& dt)
{
  state_estimator_->setJointPosition(joint_positions_);
  state_estimator_->setJointVelocity(joint_velocities_filt_);
  state_estimator_->setJointEffort(joint_efforts_);
  state_estimator_->setImuOrientation(imu_orientation_);
  state_estimator_->setImuGyroscope(imu_gyroscope_filt_);
  state_estimator_->setImuAccelerometer(imu_accelerometer_filt_);

  const std::vector<std::string>& foot_names = robot_model_->getFootNames();
  for(unsigned int i = 0; i<foot_names.size(); i++)
  {
    state_estimator_->setDesiredContactForce(foot_names[i],des_contact_forces_[i].head(3));
    state_estimator_->setDesiredContactState(foot_names[i],des_contact_states_[i]);
  }
  state_estimator_->update(dt);
}

void ControllerCore::updateTerrainEstimator(const double &dt)
{
  // Update the terrain estimator
  terrain_estimator_->computeTerrainEstimation(dt);
}

void ControllerCore::updateStateMachine(const double &dt)
{
  state_machine_->updateStateMachine(dt);
}

void ControllerCore::reset()
{
  // State estimator
  // Be sure to start the solver and the contact estimation when the robot is grounded.
  state_estimator_->resetGyroscopeIntegration();
  state_estimator_->startContactComputation();
  // Terrain Estimator
  terrain_estimator_->reset();
  // Footholds planner with gait generator
  foot_holds_planner_->reset();
  foot_holds_planner_->setBasePosition(state_estimator_->getFloatingBasePosition());
  foot_holds_planner_->setDefaultBasePosition(Eigen::Vector3d(0.0,0.0,robot_model_->getStandUpHeight()));
  foot_holds_planner_->setBaseOrientation(state_estimator_->getFloatingBaseOrientationRPY());
  foot_holds_planner_->setDefaultBaseOrientation(Eigen::Vector3d(0.0,0.0,0.0));
  foot_holds_planner_->initializeFeetPosition();
  // CoM planner
  com_planner_->reset();
  // Filters
  imu_gyroscope_filter_.setTimeStep(period_);
  imu_accelerometer_filter_.setTimeStep(period_);
  qdot_filter_.setTimeStep(period_);
  // Counters for safety checks
  contact_failures_cnt_->reset();
  solver_failures_cnt_->reset();
  for(unsigned int i=0;i<velocity_lims_failures_cnt_.size();i++)
    velocity_lims_failures_cnt_[i]->reset();
  // Control torques
  des_joint_efforts_.fill(0.0);
  des_joint_efforts_solver_.fill(0.0);
  des_joint_efforts_impedance_.fill(0.0);
  // Fill initial joint positions
  joint_positions_init_ = joint_positions_;

  current_mode_ = requested_mode_ = previous_mode_ = WPG;
}

void ControllerCore::updateWpg(const double &dt)
{
  const std::vector<std::string>& foot_names = robot_model_->getFootNames();

  // Update the gait generator contact state
  for(unsigned int i = 0; i<foot_names.size(); i++)
    gait_generator_->setContactState(foot_names[i],state_estimator_->getContact(foot_names[i]),state_estimator_->getContactForce(foot_names[i]));

  // Update the footholds planner
  foot_holds_planner_->setTerrainTransform(terrain_estimator_->getTerrainPoseWorld());
  foot_holds_planner_->update(dt);

  // Update the CoM position and velocity reference
  //com_planner_->update(dt);

  // Update the base references based on the com desired position
  updateBaseReferences(com_planner_->getComPosition(),com_planner_->getComVelocity(),foot_holds_planner_->getBaseRotationReference());

  // Set the references to the feet based on their current state: Stance/Swing
  for(unsigned int i = 0; i<foot_names.size(); i++)
  {
    id_prob_->setFootReference(foot_names[i],gait_generator_->getReference(foot_names[i]),gait_generator_->getReferenceDot(foot_names[i]),
                               WORLD_FRAME_NAME);
    if(gait_generator_->isSwinging(foot_names[i]))
    {
      id_prob_->swingWithFoot(foot_names[i],robot_model_->getBaseLinkName());
      //ROS_DEBUG_STREAM_NAMED(CLASS_NAME,"Swinging: "<< foot_names[i]);
    }
    else
    {
      id_prob_->stanceWithFoot(foot_names[i],WORLD_FRAME_NAME);
      //ROS_DEBUG_STREAM_NAMED(CLASS_NAME,"Stance: "<< foot_names[i]);
    }
  }
}

void ControllerCore::updateBaseReferences(const Eigen::Vector3d &com_pos_ref, const Eigen::Vector3d &com_vel_ref, const Eigen::Matrix3d &orientation_ref)
{
  // Set the pose reference for the waist
  id_prob_->setWaistReference(orientation_ref,com_pos_ref.z(),com_vel_ref.z());
  // Set the velocity and position reference for the CoM in the solver
  id_prob_->setComReference(com_pos_ref,com_vel_ref);
}

bool ControllerCore::performSafetyChecks()
{

  bool ok = true;

  // Check if we have at least one contact with the ground
  auto contacts = state_estimator_->getContacts();
  bool contact = false;
  auto foot_names = robot_model_->getFootNames();
  for(unsigned int i=0;i<foot_names.size();i++)
    contact = contact || contacts[foot_names[i]];
  if(!contact) // && state_estimator_->getFloatingBasePosition().z() > 0.3 * robot_model_->getStandUpHeight()
    contact_failures_cnt_->increase();
  else
    contact_failures_cnt_->reset();
  if(contact_failures_cnt_->upperLimitReached())
  {
    ok = false;
    //ROS_WARN_THROTTLE_NAMED(THROTTLE_SEC,CLASS_NAME,"Lost contacts!");
  }

  // Check the base orientation
  double roll = robot_model_->getBaseRotationInWorldRPY().x();
  double pitch = robot_model_->getBaseRotationInWorldRPY().y();
  if (roll > M_PI_2 || roll < -M_PI_2) {
    //ROS_WARN_THROTTLE_NAMED(THROTTLE_SEC,CLASS_NAME,"Base roll is beyond limits (-M_PI_2,M_PI_2)");
    ok = false;
  }
  if (pitch > M_PI_2 || pitch < -M_PI_2) {
    //ROS_WARN_THROTTLE_NAMED(THROTTLE_SEC,CLASS_NAME,"Base pitch is beyond limits (-M_PI_2,M_PI_2)");
    ok = false;
  }

  // Check if the current joint velocities (only legs for the moment) are valid otherwise set robot state to anomaly
  std::vector<bool>&& checks = robot_model_->checkJointVelocities(joint_velocities_);
  auto leg_names = robot_model_->getLegNames();
  for(unsigned int j=0;j<leg_names.size();j++)
  {
    auto joints_idx = robot_model_->getLimbJointsIds(leg_names[j]);
    for(unsigned int i=0;i<joints_idx.size();i++)
    {
      if(checks[joints_idx[i]])
        velocity_lims_failures_cnt_[joints_idx[i]]->increase();
      else
        velocity_lims_failures_cnt_[joints_idx[i]]->reset();
      if(velocity_lims_failures_cnt_[joints_idx[i]]->upperLimitReached())
      {
        ok = false;
        auto names = robot_model_->getEnabledJointNames();
        //ROS_WARN_STREAM_THROTTLE_NAMED(THROTTLE_SEC,CLASS_NAME,"Reached joint velocity limit "<<names[joints_idx[i]]);
      }
    }
  }

  return ok;
}

void ControllerCore::updateImpedance(const Eigen::VectorXd& des_joint_positions, const Eigen::VectorXd& des_joint_velocities)
{
  impedance_->update();
  des_joint_efforts_impedance_ = impedance_->getKp() * (des_joint_positions - joint_positions_) + impedance_->getKd() * ( des_joint_velocities - joint_velocities_);
}

bool ControllerCore::updateSolver(const Eigen::VectorXd& des_joint_positions)
{
  // Rotate the friction cones based on the terrain orientation
  id_prob_->setFrictionConesR(terrain_estimator_->getTerrainOrientationWorld().transpose());

  // Update the postural
  //impedance_->startInertiaCompensation(true);
  impedance_->update();
  id_prob_->setPosture(impedance_->getKp(),impedance_->getKd(),des_joint_positions);
  //impedance_->startInertiaCompensation(false);

  // Get the solver solution
  if(!id_prob_->solve(des_joint_efforts_solver_))
  {
    //ROS_WARN_THROTTLE_NAMED(THROTTLE_SEC,CLASS_NAME,"Failed to solve!");
    return false;
  }
  else
    return true;
}

void ControllerCore::update(const double& dt)
{
  period_ = dt;

  // Reset control values
  des_joint_efforts_impedance_.fill(0.0);
  des_joint_efforts_solver_.fill(0.0);
  des_joint_efforts_.fill(0.0);

  // Filter the qdot
  joint_velocities_filt_ = qdot_filter_.process(joint_velocities_);

  // Filter the imu gyroscope and accelerometer
  imu_gyroscope_filt_ = imu_gyroscope_filter_.process(imu_gyroscope_);
  imu_accelerometer_filt_ = imu_accelerometer_filter_.process(imu_accelerometer_);

  // Update state estimator
  updateStateEstimator(period_);

  // Update terrain estimator
  updateTerrainEstimator(period_);

  // Update state machine
  updateStateMachine(period_);

  // Desired joint efforts
  des_joint_efforts_ = des_joint_efforts_solver_ + des_joint_efforts_impedance_;

  // Saturate desired joint efforts
  robot_model_->clampJointEfforts(des_joint_efforts_);

  // To visualize in RT_GUI
  mode_string_ = getModeAsString();
}

const std::string &ControllerCore::getRobotName()
{
  return robot_name_;
}

const std::vector<std::string>& ControllerCore::getJointNames()
{
  return joint_names_;
}

void ControllerCore::setBaseLinearVelocityCmdX(const double &v)
{
  vel_x_ = v;
  foot_holds_planner_->setBaseLinearVelocityCmdX(vel_x_);
}

void ControllerCore::setBaseLinearVelocityCmdY(const double &v)
{
  vel_y_ = v;
  foot_holds_planner_->setBaseLinearVelocityCmdY(vel_y_);
}

void ControllerCore::setBaseLinearVelocityCmdZ(const double &v)
{
  vel_z_ = v;
  foot_holds_planner_->setBaseLinearVelocityCmdZ(vel_z_);
}

void ControllerCore::setBaseAngularVelocityCmdRoll(const double &v)
{
  vel_roll_ = v;
  foot_holds_planner_->setBaseAngularVelocityCmdRoll(vel_roll_);
}

void ControllerCore::setBaseAngularVelocityCmdPitch(const double &v)
{
  vel_pitch_ = v;
  foot_holds_planner_->setBaseAngularVelocityCmdPitch(vel_pitch_);
}

void ControllerCore::setBaseAngularVelocityCmdYaw(const double &v)
{
  vel_yaw_ = v;
  foot_holds_planner_->setBaseAngularVelocityCmdYaw(vel_yaw_);
}

double ControllerCore::getBaseLinearVelocityCmdX()
{
  return vel_x_;
}

double ControllerCore::getBaseLinearVelocityCmdY()
{
  return vel_y_;
}

double ControllerCore::getBaseLinearVelocityCmdZ()
{
  return vel_z_;
}

double ControllerCore::getBaseAngularVelocityCmdRoll()
{
  return vel_roll_;
}

double ControllerCore::getBaseAngularVelocityCmdPitch()
{
  return vel_pitch_;
}

double ControllerCore::getBaseAngularVelocityCmdYaw()
{
  return vel_yaw_;
}

IDProblem* ControllerCore::getIDProblem() const
{
  return id_prob_.get();
}

GaitGenerator* ControllerCore::getGaitGenerator() const
{
  return gait_generator_.get();
}

StateEstimator* ControllerCore::getStateEstimator() const
{
  return state_estimator_.get();
}

FootholdsPlanner* ControllerCore::getFootholdsPlanner() const
{
  return foot_holds_planner_.get();
}

TerrainEstimator* ControllerCore::getTerrainEstimator() const
{
  return terrain_estimator_.get();
}

Impedance* ControllerCore::getImpedance() const
{
  return impedance_.get();
}

wolf_wbid::QuadrupedRobot* ControllerCore::getRobotModel() const
{
  return robot_model_.get();
}

StateMachine* ControllerCore::getStateMachine() const
{
  return state_machine_.get();
}

std::vector<Eigen::Vector6d>& ControllerCore::getDesiredContactForces()
{
  if(id_prob_)
    des_contact_forces_ = id_prob_->getContactWrenches();
  return des_contact_forces_;
}

std::vector<bool>& ControllerCore::getDesiredContactStates()
{
  auto foot_names = robot_model_->getFootNames();
  for(unsigned int i=0; i<foot_names.size(); i++)
    des_contact_states_[i] = gait_generator_->isInStance(foot_names[i]);
  //auto ee_names = robot_model_->getEndEffectorNames();
  //for(unsigned int i=foot_names.size(); i<ee_names.size()+foot_names.size(); i++)
  //  des_contact_states_[i] = false;
  return des_contact_states_;
}

const Eigen::VectorXd &ControllerCore::getDesiredJointEfforts() const
{
  return des_joint_efforts_;
}

std::string ControllerCore::getModeAsString()
{
  return enumToString(current_mode_);
}

std::vector<std::string> ControllerCore::getModesAsString()
{
  std::vector<std::string> modes;
  for(unsigned int i=0; i< N_MODES; i++)
    modes.push_back(enumToString(static_cast<mode_t>(i)));

  return modes;
}

} //namespace
