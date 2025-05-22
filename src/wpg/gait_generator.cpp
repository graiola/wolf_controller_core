/**
 * @file gait_generator.cpp
 * @author Gennaro Raiola
 * @date 12 June, 2019
 * @brief This file contains the Gait and GaitGenerator
 */

#include <wolf_controller_core/wpg/gait_generator.h>
#include <wolf_controller_core/wpg/foot_trajectory_ellipse.h>

using namespace wolf_controller;
using namespace wolf_controller_utils;

Gait::Gait(const std::vector<std::string>& foot_names, const gait_t& gait_type)
{
  assert(foot_names.size() == N_LEGS);

  auto ordered_foot_names = sortByLegPrefix(foot_names);

  if(gait_type == TROT)
  {
    schedule_.push_back(foot_priority_t(ordered_foot_names[_leg_id::LF],0));
    schedule_.push_back(foot_priority_t(ordered_foot_names[_leg_id::RH],0));
    schedule_.push_back(foot_priority_t(ordered_foot_names[_leg_id::RF],1));
    schedule_.push_back(foot_priority_t(ordered_foot_names[_leg_id::LH],1));
    next_feet_to_move_.resize(2);
    max_priority_ = 1;
  }
  else if(gait_type == CRAWL)
  {
    schedule_.push_back(foot_priority_t(ordered_foot_names[_leg_id::LF],0));
    schedule_.push_back(foot_priority_t(ordered_foot_names[_leg_id::RH],1));
    schedule_.push_back(foot_priority_t(ordered_foot_names[_leg_id::RF],2));
    schedule_.push_back(foot_priority_t(ordered_foot_names[_leg_id::LH],3));
    next_feet_to_move_.resize(1);
    max_priority_ = 3;
  }
  else if(gait_type == ONE_FOOT_LF)
  {
    schedule_.push_back(foot_priority_t(ordered_foot_names[_leg_id::LF],0));
    next_feet_to_move_.resize(1);
    max_priority_ = 0;
  }
  else if(gait_type == ONE_FOOT_RH)
  {
    schedule_.push_back(foot_priority_t(ordered_foot_names[_leg_id::RH],0));
    next_feet_to_move_.resize(1);
    max_priority_ = 0;
  }
  else if(gait_type == ONE_FOOT_RF)
  {
    schedule_.push_back(foot_priority_t(ordered_foot_names[_leg_id::RF],0));
    next_feet_to_move_.resize(1);
    max_priority_ = 0;
  }
  else if(gait_type == ONE_FOOT_LH)
  {
    schedule_.push_back(foot_priority_t(ordered_foot_names[_leg_id::LH],0));
    next_feet_to_move_.resize(1);
    max_priority_ = 0;
  }
  else
  {
    throw std::runtime_error("Wrong gait type!");
  }

  reset();
}

void Gait::reset()
{
  current_priority_ = 0;
  cycle_ended_ = true;

  unsigned int idx = 0;
  for(unsigned int i=0;i < schedule_.size(); i++)
    if(schedule_[i].second == current_priority_)
      next_feet_to_move_[idx++] = schedule_[i].first;
}

void Gait::update()
{

  if(current_priority_ == 0)
    cycle_ended_ = true;
  else
    cycle_ended_ = false;

  current_priority_++;
  current_priority_ %= max_priority_+1;

  unsigned int idx = 0;
  for(unsigned int i=0;i < schedule_.size(); i++)
    if(schedule_[i].second == current_priority_)
      next_feet_to_move_[idx++] = schedule_[i].first;
}

const std::vector<std::string>& Gait::getNextSchedule()
{
  return next_feet_to_move_;
}

bool Gait::isCycleEnded()
{
  return cycle_ended_;
}

double Gait::getVelocityFactor(const gait_t& gait_type)
{
  switch(gait_type)
  {
    case gait_t::TROT:
      return 2.0;
    case gait_t::CRAWL:
      return 4.0;
  };
  return 1.0;
}

GaitGenerator::GaitGenerator(const std::vector<std::string>& foot_names, const Gait::gait_t& gait_type, const Gait::trajectory_t& trajectory_type)
{
  assert(foot_names.size()==N_LEGS);// We assume we are working with a dog
  foot_names_ = foot_names;
  gait_type_ = gait_type;

  for(unsigned int i = 0; i<foot_names_.size(); i++)
  {
    feet_[foot_names_[i]].state_machine.reset(new FootStateMachine());
    feet_[foot_names_[i]].trajectory.reset(selectTrajectoryType(trajectory_type));
    feet_[foot_names_[i]].contact        = false;
    feet_[foot_names_[i]].contact_force  = Eigen::Vector3d::Zero();
    feet_[foot_names_[i]].trigger_stance = false;
    feet_[foot_names_[i]].initial_pose = Eigen::Affine3d::Identity();
  }

  setSwingFrequency(0.0);

  gait_buffer_.resize(2);
  for(unsigned int i=0; i<gait_buffer_.size(); i++)
    gait_buffer_[i].reset(new Gait(foot_names_,gait_type_));

  current_gait_idx_ = 0;
  next_gait_idx_ = 1;
  scheduled_feet_ = gait_buffer_[current_gait_idx_]->getNextSchedule();

  step_reflex_active_ = false;

  reset();
}

void GaitGenerator::reset()
{
  for(unsigned int i = 0; i<foot_names_.size(); i++)
    feet_[foot_names_[i]].state_machine->reset();

  //for(unsigned int i=0; i<gait_buffer_.size(); i++)
  //  gait_buffer_[i]->reset();

  change_gait_ = false;
  activate_swing_ = false;
}

const std::vector<std::string>& GaitGenerator::getFootNames()
{
  return foot_names_;
}

void GaitGenerator::switchGait()
{
  if(gait_type_ == Gait::gait_t::TROT)
    setGaitType(Gait::gait_t::CRAWL);
  else
    setGaitType(Gait::gait_t::TROT);
}

void GaitGenerator::setGaitType(const Gait::gait_t& gait_type)
{
  gait_buffer_[next_gait_idx_].reset(new Gait(foot_names_,gait_type));
  change_gait_ = true;
  gait_type_ = gait_type;
}

const Eigen::Affine3d& GaitGenerator::getReference(const std::string& foot_name)
{
  return feet_[foot_name].trajectory->getReference();
}

const Eigen::Vector6d& GaitGenerator::getReferenceDot(const std::string& foot_name)
{
  return feet_[foot_name].trajectory->getReferenceDot();
}

bool GaitGenerator::isSwinging(const std::string& foot_name)
{
  return feet_[foot_name].state_machine->isSwing();
}

bool GaitGenerator::isInStance(const std::string& foot_name)
{
  return feet_[foot_name].state_machine->isStance();
}

bool GaitGenerator::isStateChanged(const std::string& foot_name)
{
  return feet_[foot_name].state_machine->isStateChanged();
}

bool GaitGenerator::isTouchDown(const std::string& foot_name)
{
  return feet_[foot_name].state_machine->isTouchDown();
}

bool GaitGenerator::isLiftOff(const std::string& foot_name)
{
  return feet_[foot_name].state_machine->isLiftOff();
}

bool GaitGenerator::isCycleEnded(const std::string& foot_name)
{
  return feet_[foot_name].state_machine->isCycleEnded();
}

bool GaitGenerator::isGaitCycleEnded()
{
  return gait_buffer_[current_gait_idx_]->isCycleEnded();
}

bool GaitGenerator::isAnyFootInLiftOff()
{
  bool result = false;
  for(feet_t::iterator it = feet_.begin(); it!=feet_.end(); ++it)
    result = result || it->second.state_machine->isLiftOff();
  return result;
}

bool GaitGenerator::isAnyFootInSwing()
{
  bool result = false;
  for(feet_t::iterator it = feet_.begin(); it!=feet_.end(); ++it)
    result = result || it->second.state_machine->isSwing();
  return result;
}

bool GaitGenerator::isAnyFootInTouchDown()
{
  bool result = false;
  for(feet_t::iterator it = feet_.begin(); it!=feet_.end(); ++it)
    result = result || it->second.state_machine->isTouchDown();
  return result;
}

bool GaitGenerator::isAnyFootInStance()
{
  bool result = false;
  for(feet_t::iterator it = feet_.begin(); it!=feet_.end(); ++it)
    result = result || it->second.state_machine->isStance();
  return result;
}

bool GaitGenerator::areAllFeetInStance()
{
  bool result = true;
  for(feet_t::iterator it = feet_.begin(); it!=feet_.end(); ++it)
    result = result && it->second.state_machine->isStance();
  return result;
}

unsigned int GaitGenerator::getNumberFeetInStance()
{
  unsigned int n = 0;
  for(feet_t::iterator it = feet_.begin(); it!=feet_.end(); ++it)
    if(it->second.state_machine->isStance())
      n++;
  return n;
}

unsigned int GaitGenerator::getNumberFeetInSwing()
{
  unsigned int n = 0;
  for(feet_t::iterator it = feet_.begin(); it!=feet_.end(); ++it)
    if(it->second.state_machine->isSwing())
      n++;
  return n;
}

void GaitGenerator::setContactState(const std::string& foot_name, const bool& contact, const Eigen::Vector3d& contact_force)
{
  feet_[foot_name].contact = contact;
  feet_[foot_name].contact_force = contact_force;
}

const bool& GaitGenerator::getContact(const std::string& foot_name)
{
  return feet_[foot_name].contact;
}

const Eigen::Vector3d& GaitGenerator::getContactForce(const std::string& foot_name)
{
  return feet_[foot_name].contact_force;
}

void GaitGenerator::setInitialPose(const std::string& foot_name, const Eigen::Affine3d& initial_pose)
{
  feet_[foot_name].trajectory->setInitialPose(initial_pose);
}

double GaitGenerator::getDutyFactor(const std::string& foot_name)
{
  return feet_[foot_name].state_machine->getDutyFactor();
}

void GaitGenerator::setDutyFactor(const double& duty_factor)
{
  for(feet_t::iterator it = feet_.begin(); it!=feet_.end(); ++it)
    it->second.state_machine->setDutyFactor(duty_factor);
}

void GaitGenerator::setDutyFactor(const std::string& foot_name, const double& duty_factor)
{
  feet_[foot_name].state_machine->setDutyFactor(duty_factor);
}

void GaitGenerator::increaseDutyFactor()
{
    setDutyFactor(getAvgDutyFactor()+0.1);
}

void GaitGenerator::decreaseDutyFactor()
{
    setDutyFactor(getAvgDutyFactor()-0.1);
}

void GaitGenerator::setSwingFrequency(const double& swing_frequency)
{
  for(feet_t::iterator it = feet_.begin(); it!=feet_.end(); ++it)
  {
    it->second.trajectory->setSwingFrequency(swing_frequency);
    it->second.state_machine->setSwingFrequency(swing_frequency);
  }
}

void GaitGenerator::increaseSwingFrequency()
{
    setSwingFrequency(getAvgSwingFrequency()+0.1);
}

void GaitGenerator::decreaseSwingFrequency()
{
    setSwingFrequency(getAvgSwingFrequency()-0.1);
}

void GaitGenerator::setSwingFrequency(const std::string& foot_name, const double& swing_frequency)
{
  feet_[foot_name].trajectory->setSwingFrequency(swing_frequency);
  feet_[foot_name].state_machine->setSwingFrequency(swing_frequency);
}

double GaitGenerator::getSwingFrequency(const std::string& foot_name)
{
  return feet_[foot_name].trajectory->getSwingFrequency();
}

double GaitGenerator::getAvgSwingFrequency()
{
  double avg = 0.0;
  for(feet_t::iterator it = feet_.begin(); it!=feet_.end(); ++it)
    avg = avg + it->second.trajectory->getSwingFrequency();
  avg = avg / feet_.size();
  return avg;
}

double GaitGenerator::getAvgStanceFrequency()
{
  double avg = 0.0;
  for(feet_t::iterator it = feet_.begin(); it!=feet_.end(); ++it)
    avg = avg + 1.0/it->second.state_machine->getStancePeriod();
  avg = avg / feet_.size();
  return avg;
}

double GaitGenerator::getAvgCycleTime()
{
  double avg = 0.0;
  for(feet_t::iterator it = feet_.begin(); it!=feet_.end(); ++it)
    avg = avg + (it->second.state_machine->getStancePeriod() + it->second.state_machine->getSwingPeriod());
  avg = avg / feet_.size();
  return avg;
}

double GaitGenerator::getAvgDutyFactor()
{
  double avg = 0.0;
  for(feet_t::iterator it = feet_.begin(); it!=feet_.end(); ++it)
    avg = avg + it->second.state_machine->getDutyFactor();
  avg = avg / feet_.size();
  return avg;
}

double GaitGenerator::getAvgTrajectoryCompletion()
{
  double avg = 0.0;
  int n_swinging_feet = 0;
  for(feet_t::iterator it = feet_.begin(); it!=feet_.end(); ++it)
  {
    if(it->second.state_machine->isSwing())
    {
      avg = avg + it->second.trajectory->getCompletion();
      n_swinging_feet++;
    }
  }
  if(n_swinging_feet>0)
    avg = avg / n_swinging_feet;
  return avg;
}

double GaitGenerator::getTrajectoryCompletion(const std::string& foot_name)
{
  return feet_[foot_name].trajectory->getCompletion();
}

double GaitGenerator::getStancePeriod(const std::string& foot_name)
{
  return feet_[foot_name].state_machine->getStancePeriod();
}

double GaitGenerator::getSwingPeriod(const std::string& foot_name)
{
  return feet_[foot_name].state_machine->getSwingPeriod();
}

Gait::gait_t GaitGenerator::getGaitType()
{
  return gait_type_;
}

double GaitGenerator::getVelocityFactor()
{
  return gait_buffer_[current_gait_idx_]->getVelocityFactor(gait_type_);
}

void GaitGenerator::setTerrainRotation(const Eigen::Matrix3d& world_R_terrain)
{
  for(feet_t::iterator it = feet_.begin(); it!=feet_.end(); ++it)
    it->second.trajectory->setTerrainRotation(world_R_terrain);
}

void GaitGenerator::setStepLength(const double& length)
{
  for(feet_t::iterator it = feet_.begin(); it!=feet_.end(); ++it)
    it->second.trajectory->setStepLength(length);
}

void GaitGenerator::setStepHeading(const double& heading)
{
  for(feet_t::iterator it = feet_.begin(); it!=feet_.end(); ++it)
    it->second.trajectory->setStepHeading(heading);
}

void GaitGenerator::setStepHeading(const std::string& foot_name, const double& heading)
{
  feet_[foot_name].trajectory->setStepHeading(heading);
}

void GaitGenerator::setStepHeadingRate(const double& yaw_rate)
{
  for(feet_t::iterator it = feet_.begin(); it!=feet_.end(); ++it)
    it->second.trajectory->setStepHeadingRate(yaw_rate);
}

void GaitGenerator::setStepHeadingRate(const std::string& foot_name, const double& heading_rate)
{
  feet_[foot_name].trajectory->setStepHeadingRate(heading_rate);
}

void GaitGenerator::setStepHeight(const double& height)
{
  for(feet_t::iterator it = feet_.begin(); it!=feet_.end(); ++it)
    it->second.trajectory->setStepHeight(height);
}

void GaitGenerator::setStepLength(const std::string& foot_name, const double& length)
{
  feet_[foot_name].trajectory->setStepLength(length);
}

void GaitGenerator::setStepHeight(const std::string& foot_name, const double& height)
{
  feet_[foot_name].trajectory->setStepHeight(height);
}

void GaitGenerator::activateSwing()
{
  activate_swing_ = true;
}

void GaitGenerator::deactivateSwing()
{
  activate_swing_ = false;
}

bool GaitGenerator::isTrajectoryFinished(const std::string& foot_name)
{
  return feet_[foot_name].trajectory->isFinished();
}

bool GaitGenerator::isFirstStep()
{
  return first_step_.update(activate_swing_);
}

bool GaitGenerator::isLastStep()
{
  return last_step_.update(!activate_swing_);
}

void GaitGenerator::update(const double& period)
{
  // 1) Check if the scheduled feet are all ready to get triggered and start the swing if this is the case.
  bool scheduled_feet_are_ready = true;
  for(unsigned int i=0; i<scheduled_feet_.size(); i++)
    if(!feet_[scheduled_feet_[i]].state_machine->isCycleEnded())
    {
      scheduled_feet_are_ready = false;
      break;
    }

  if(activate_swing_ || !gait_buffer_[current_gait_idx_]->isCycleEnded())
    if(scheduled_feet_are_ready)
    {
      for(unsigned int i=0; i<scheduled_feet_.size(); i++)
        feet_[scheduled_feet_[i]].state_machine->triggerSwing();
    }

  // 2) Update the trajectories for each foot depending on the state machine status
  for(feet_t::iterator it = feet_.begin(); it != feet_.end(); it++)
  {
#ifdef REACHING_MOTION
    it->second.trigger_stance = it->second.contact;
#else
    it->second.trigger_stance = it->second.contact || it->second.trajectory->isFinished(); //CloseLoop with trajectory end
#endif
    it->second.state_machine->update(period,it->second.trigger_stance);

    if (it->second.state_machine->isSwing())
    {
      if (it->second.state_machine->isLiftOff())
      {
        it->second.trajectory->start();
      }
      it->second.trajectory->update(period,it->second.contact_force);

      //ROS_DEBUG_STREAM_NAMED(CLASS_NAME,"Update trajectory for foot "<< it->first);
    }
    else
    {
      if (it->second.state_machine->isTouchDown())
      {
        it->second.trajectory->stop();
      }
      //it->second.trajectory->standBy();

      //ROS_DEBUG_STREAM_NAMED(CLASS_NAME,"Stop trajectory for foot "<< it->first);
    }
  }

  // 3) If the cycle for the scheduled feet is over, change the schedule to the next one (i.e. move to the next feet)
  unsigned int cnt = 0;
  for(unsigned int i=0; i<scheduled_feet_.size(); i++)
    if(feet_[scheduled_feet_[i]].state_machine->isCycleEnded())
      cnt++;

  bool trigger_next_schedule = false;
  if(cnt == scheduled_feet_.size())
    trigger_next_schedule = next_schedule_.update(true);
  else
    trigger_next_schedule = next_schedule_.update(false);

  if(trigger_next_schedule)
  {
    if(change_gait_)
      changeGait();

    gait_buffer_[current_gait_idx_]->update();
    scheduled_feet_ = gait_buffer_[current_gait_idx_]->getNextSchedule();
  }
}

void GaitGenerator::startStepReflex(bool start)
{
  step_reflex_active_ = start;
  for(feet_t::iterator it = feet_.begin(); it!=feet_.end(); ++it)
    it->second.trajectory->startStepReflex(step_reflex_active_);

  if(step_reflex_active_)
    PRINT_INFO_NAMED(CLASS_NAME,"Step reflex activated!");
  else
    PRINT_INFO_NAMED(CLASS_NAME,"Step reflex de-activated!");
}

void GaitGenerator::toggleStepReflex()
{
  step_reflex_active_ = !step_reflex_active_;
  for(feet_t::iterator it = feet_.begin(); it!=feet_.end(); ++it)
    it->second.trajectory->startStepReflex(step_reflex_active_);

  if(step_reflex_active_)
    PRINT_INFO_NAMED(CLASS_NAME,"Step reflex activated!");
  else
    PRINT_INFO_NAMED(CLASS_NAME,"Step reflex de-activated!");
}

bool GaitGenerator::isStepReflexActive()
{
  return step_reflex_active_;
}

void GaitGenerator::setStepReflexContactThreshold(const double& th)
{
  for(feet_t::iterator it = feet_.begin(); it!=feet_.end(); ++it)
    it->second.trajectory->setStepReflexContactThreshold(th);
}

void GaitGenerator::setStepReflexMaxRetraction(const double &max)
{
  for(feet_t::iterator it = feet_.begin(); it!=feet_.end(); ++it)
    it->second.trajectory->setStepReflexMaxRetraction(max);
}

void GaitGenerator::changeGait()
{
  current_gait_idx_ = current_gait_idx_ + 1;
  current_gait_idx_ = current_gait_idx_ % 2;
  next_gait_idx_    = next_gait_idx_    + 1;
  next_gait_idx_    = next_gait_idx_    % 2;

  change_gait_ = false;
}

TrajectoryInterface* GaitGenerator::selectTrajectoryType(Gait::trajectory_t trajectory_type)
{
  //PRINT_INFO_NAMED(CLASS_NAME,"Selected " << trajectory_type << " trajectory");
  switch(trajectory_type)
  {
  case Gait::ELLIPSE:
    return new Ellipse();
  default:
    throw std::runtime_error("Wrong trajectory type!");
  };
  return nullptr;
}
