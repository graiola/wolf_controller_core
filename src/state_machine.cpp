/**
 * @file state_machine.cpp
 * @author Gennaro Raiola
 * @date 1 November, 2021
 * @brief This file contains the StateMachine code
 */

#include <wolf_controller_core/state_machine.h>
#include <wolf_controller_core/controller_core.h>
#include <wolf_controller_utils/geometry.h>

// RT GUI
#ifdef RT_GUI
#include <rt_gui/rt_gui_client.h>
using namespace rt_gui;
#endif

using namespace wolf_wbid;
using namespace wolf_controller;
using namespace wolf_controller_utils;

std::string enumToString(StateMachine::states_t state) {
  switch (state) {
  case StateMachine::IDLE:          return "IDLE";
  case StateMachine::INIT:          return "INIT";
  case StateMachine::STANDING_UP:   return "STANDING_UP";
  case StateMachine::ACTIVE:        return "ACTIVE";
  case StateMachine::STANDING_DOWN: return "STANDING_DOWN";
  case StateMachine::ANOMALY:       return "ANOMALY";
  default:                          return "UNKNOWN";
  }
}

//////////////////////////////////////////////////////////////////////////////
// State Machine Implementation
StateMachine::StateMachine(ControllerCore* controller)
  : state_changed_(true), controller_(controller), current_state_(IDLE), previous_state_(N_STATES), current_state_str_("IDLE") {
  states_[IDLE] = std::make_shared<ControllerIdleState>();
  states_[INIT] = std::make_shared<ControllerInitState>();
  states_[STANDING_UP] = std::make_shared<ControllerStandingUpState>();
  states_[ACTIVE] = std::make_shared<ControllerActiveState>();
  states_[STANDING_DOWN] = std::make_shared<ControllerStandingDownState>();
  states_[ANOMALY] = std::make_shared<ControllerAnomalyState>();

#ifdef RT_GUI
  // create interface
  RtGuiClient::getIstance().addLabel(std::string(wolf_controller::_rt_gui_group),std::string("Status"),&current_state_str_);
#endif
}

void StateMachine::updateStateMachine(const double& dt)
{
  if(state_changed_)
  {
    // NOTE: this is executing onExit and onEntry and Update on one tick
    if(state_changed_ && states_.find(previous_state_) != states_.end())
      states_[previous_state_]->onExit(this);

    if(state_changed_ && states_.find(current_state_) != states_.end())
      states_[current_state_]->onEntry(this);

    state_changed_ = false;
  }

  if (states_.find(current_state_) != states_.end())
    states_[current_state_]->updateStateMachine(this, dt);

  // save the state as string
  current_state_str_ = enumToString(current_state_);
}

void StateMachine::setCurrentState(StateMachine::states_t new_state)
{
  if(new_state != current_state_)
  {
    ROS_INFO_STREAM_NAMED(CLASS_NAME,"Change state to "<<enumToString(new_state));
    state_changed_ = true;
  }
  previous_state_.store(current_state_.load());
  current_state_ = new_state;
}

std::string StateMachine::getStateAsString()
{
  return enumToString(current_state_);
}

std::vector<std::string> StateMachine::getStatesAsString()
{
  std::vector<std::string> states;
  for(unsigned int i=0; i< N_STATES; i++)
    states.push_back(enumToString(static_cast<states_t>(i)));

  return states;
}

StateMachine::states_t StateMachine::getCurrentState()
{
  return current_state_;
}

StateMachine::states_t StateMachine::getPreviousState()
{
  return previous_state_;
}

ControllerCore *StateMachine::getController()
{
  return controller_;
}

/////////////////////////////////// IDLE ///////////////////////////////////////////
void ControllerIdleState::updateStateMachine(StateMachine* state_machine, const double& dt) {
  ControllerCore* controller = state_machine->getController();
  if (controller->posture_ == ControllerCore::posture_t::UP)
    state_machine->setCurrentState(StateMachine::INIT);
}

void ControllerIdleState::onEntry(StateMachine* state_machine) {
  //ControllerCore* controller = state_machine->getController();
  //controller->desired_height_ = 0.0;
  //controller->current_rpy_ = controller->robot_model_->getBaseRotationInWorldRPY();
}

void ControllerIdleState::onExit(StateMachine* state_machine) {
  ControllerCore* controller = state_machine->getController();
  controller->reset();
}

/////////////////////////////////// INIT ///////////////////////////////////////////
ControllerInitState::ControllerInitState()
{
  ramp_ = std::make_shared<Ramp>(3.0,Ramp::UP);
}

void ControllerInitState::updateStateMachine(StateMachine* state_machine, const double& dt) {
  ControllerCore* controller = state_machine->getController();
  double ramp = ramp_->update(dt);
  controller->des_joint_positions_ = ramp * controller->robot_model_->getStandDownJointPostion() +
      (1.0 - ramp) * controller->joint_positions_init_;
  controller->updateImpedance(controller->des_joint_positions_, controller->des_joint_velocities_);
  if (ramp >= 1.0)
    state_machine->setCurrentState(StateMachine::STANDING_UP);
}

void ControllerInitState::onEntry(StateMachine *state_machine)
{
  ControllerCore* controller = state_machine->getController();
  controller->des_joint_positions_ = controller->robot_model_->getStandDownJointPostion();
  controller->des_joint_velocities_.fill(0.0);
}

void ControllerInitState::onExit(StateMachine *state_machine)
{
  ControllerCore* controller = state_machine->getController();
  controller->id_prob_->reset();
  ramp_->reset();
}

/////////////////////////////////// STANDING UP ///////////////////////////////////////////
ControllerStandingUpState::ControllerStandingUpState()
{
  ramp_ = std::make_shared<Ramp>(5.0,Ramp::UP);
}

void ControllerStandingUpState::updateStateMachine(StateMachine* state_machine, const double& dt) {
  ControllerCore* controller = state_machine->getController();
  controller->updateWpg(dt);
  double ramp = ramp_->update(dt);
  desired_height_ = controller->terrain_estimator_->getTerrainPositionWorld().z() +
      ramp * controller->robot_model_->getStandUpHeight();
  controller->des_joint_positions_ = ramp * controller->robot_model_->getStandUpJointPostion() +
      (1.0 - ramp) * controller->robot_model_->getStandDownJointPostion();
  rpy_ << 0.0, 0.0, desired_yaw_;
  rpyToRot(rpy_, R_);
  pos_ << controller->com_planner_->getComPosition().x(),
          controller->com_planner_->getComPosition().y(),
          desired_height_;
  vel_.setZero();
  vel_.z() = controller->foot_holds_planner_->getBaseLinearVelocityCmdZ();
  controller->updateBaseReferences(pos_, vel_, R_);
  if (!controller->updateSolver(controller->des_joint_positions_)) {
    state_machine->setCurrentState(StateMachine::ANOMALY);
  } else if (controller->getStateEstimator()->getEstimatedBaseHeight() >= controller->robot_model_->getStandUpHeight()) {
    state_machine->setCurrentState(StateMachine::ACTIVE);
  }
}

void ControllerStandingUpState::onEntry(StateMachine *state_machine)
{
  R_.setIdentity();
  rpy_.setZero();
  vel_.setZero();
  pos_.setZero();
  ControllerCore* controller = state_machine->getController();
  desired_yaw_ = controller->robot_model_->getBaseRotationInWorldRPY().z();
  desired_height_ = 0.0;
}

void ControllerStandingUpState::onExit(StateMachine *state_machine)
{
  ramp_->reset();
}

/////////////////////////////////// ACTIVE ///////////////////////////////////////////
void ControllerActiveState::updateStateMachine(StateMachine* state_machine, const double& dt) {
  ControllerCore* controller = state_machine->getController();
  if (controller->requested_mode_ != controller->current_mode_ &&
      controller->state_estimator_->areAllFeetInContact()) {
    controller->current_mode_ = controller->requested_mode_;
  }

  switch (controller->current_mode_) {
  case ControllerCore::mode_t::WPG:
    controller->updateWpg(dt);
    controller->id_prob_->setControlMode(IDProblem::mode_t::WPG);
    controller->previous_mode_ = ControllerCore::mode_t::WPG;
    break;
  case ControllerCore::mode_t::EXT:
    controller->foot_holds_planner_->update(dt, controller->robot_model_->getBasePoseInWorld().translation(),
                                            controller->robot_model_->getBaseRotationInWorldRPY());
    controller->id_prob_->setControlMode(IDProblem::mode_t::EXT);
    controller->previous_mode_ = ControllerCore::mode_t::EXT;
    break;
  case ControllerCore::mode_t::MPC:
    controller->foot_holds_planner_->update(dt, controller->robot_model_->getBasePoseInWorld().translation(),
                                            controller->robot_model_->getBaseRotationInWorldRPY());
    controller->id_prob_->setControlMode(IDProblem::mode_t::MPC);
    controller->previous_mode_ = ControllerCore::mode_t::MPC;
    break;
  case ControllerCore::mode_t::RESET:
    controller->foot_holds_planner_->setCmd(FootholdsPlanner::RESET_BASE);
    controller->updateWpg(dt);
    controller->id_prob_->setControlMode(IDProblem::mode_t::WPG);
    controller->previous_mode_ = ControllerCore::mode_t::RESET;
    pos_ << controller->com_planner_->getComPosition().x(),
            controller->com_planner_->getComPosition().y(),
            controller->foot_holds_planner_->getBaseHeight();
    R_ = controller->foot_holds_planner_->getBaseRotationReference();
    vel_.setZero();
    controller->updateBaseReferences(pos_, vel_, R_);
    if (controller->getRobotModel()->getCurrentHeight() >= controller->robot_model_->getStandUpHeight()) {
      state_machine->setCurrentState(StateMachine::ACTIVE);
      controller->requested_mode_ = ControllerCore::mode_t::WPG;
    }
    break;
  }

  if (!controller->updateSolver(controller->robot_model_->getStandUpJointPostion()) ||
      !controller->performSafetyChecks()) {
    state_machine->setCurrentState(StateMachine::ANOMALY);
  }
  else if (controller->posture_ == ControllerCore::posture_t::DOWN)
  {
    state_machine->setCurrentState(StateMachine::STANDING_DOWN);
  }
}

void ControllerActiveState::onEntry(StateMachine *state_machine)
{
  R_.setIdentity();
  vel_.setZero();
  pos_.setZero();

  ControllerCore* controller = state_machine->getController();
  controller->foot_holds_planner_->reset();
  controller->com_planner_->reset();
  controller->state_estimator_->startContactComputation();
}

void ControllerActiveState::onExit(StateMachine *state_machine)
{

}

/////////////////////////////////// STANDING DOWN ///////////////////////////////////////////
ControllerStandingDownState::ControllerStandingDownState()
{
  ramp_ = std::make_shared<Ramp>(5.0,Ramp::DOWN);
}

void ControllerStandingDownState::updateStateMachine(StateMachine* state_machine, const double& dt) {
  ControllerCore* controller = state_machine->getController();
  controller->updateWpg(dt);
  double ramp = ramp_->update(dt);
  desired_height_ = ramp * stand_down_starting_height_;
  controller->des_joint_positions_ = (1.0 - ramp) * controller->robot_model_->getStandUpJointPostion() +
      ramp * controller->robot_model_->getStandDownJointPostion();
  rpy_ << 0.0, 0.0, controller->robot_model_->getBaseRotationInWorldRPY().z();
  rpyToRot(rpy_, R_);
  pos_ << controller->com_planner_->getComPosition().x(),
          controller->com_planner_->getComPosition().y(),
          desired_height_;
  vel_.setZero();
  vel_.z() = -controller->foot_holds_planner_->getBaseLinearVelocityCmdZ();
  controller->updateBaseReferences(pos_, vel_, R_);
  if (!controller->updateSolver(controller->des_joint_positions_)) {
    state_machine->setCurrentState(StateMachine::ANOMALY);
  } else if (desired_height_ <= EPS) {
    state_machine->setCurrentState(StateMachine::IDLE);
  }
}

void ControllerStandingDownState::onEntry(StateMachine *state_machine)
{
  R_.setIdentity();
  rpy_.setZero();
  vel_.setZero();
  pos_.setZero();
  desired_height_ = 0.0;
  ControllerCore* controller = state_machine->getController();
  stand_down_starting_height_ = controller->getRobotModel()->getCurrentHeight();
}

void ControllerStandingDownState::onExit(StateMachine *state_machine)
{
  ControllerCore* controller = state_machine->getController();
  ramp_->reset();
  controller->state_estimator_->stopContactComputation();
}

/////////////////////////////////// ANOMALY ///////////////////////////////////////////
void ControllerAnomalyState::updateStateMachine(StateMachine* state_machine, const double& dt) {
  ControllerCore* controller = state_machine->getController();
  controller->des_joint_positions_ = controller->robot_model_->getStandDownJointPostion();
  controller->updateImpedance(controller->des_joint_positions_, controller->des_joint_velocities_);
  if (!controller->getRobotModel()->isRobotFalling(dt)) {
    state_machine->setCurrentState(StateMachine::IDLE);
  }
}

void ControllerAnomalyState::onEntry(StateMachine *state_machine)
{
  ControllerCore* controller = state_machine->getController();
  controller->des_joint_velocities_.fill(0.0);
}

void ControllerAnomalyState::onExit(StateMachine *state_machine)
{
  ControllerCore* controller = state_machine->getController();
  controller->posture_ = ControllerCore::posture_t::DOWN;
  controller->terrain_estimator_->reset();
  controller->state_estimator_->stopContactComputation();
}
