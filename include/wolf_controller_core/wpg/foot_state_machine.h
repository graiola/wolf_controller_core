/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) Gennaro Raiola
 */

#ifndef WPG_FOOT_STATE_MACHINE_H
#define WPG_FOOT_STATE_MACHINE_H

#include <atomic>
#include <string>

namespace wolf_controller
{

class FootStateMachine
{
public:

  const std::string CLASS_NAME = "FootStateMachine";

  FootStateMachine();

  void reset();

  bool isSwing();

  bool isStance();

  bool isStateChanged();

  bool isTouchDown();

  bool isLiftOff();

  bool isCycleEnded();

  void triggerSwing();

  void setDutyFactor(double duty_factor);

  void setSwingFrequency(double swing_frequency);

  double getDutyFactor();

  double getStancePeriod();

  double getSwingPeriod();

  void update(const double& period, const bool& contact);

private:

  void init();

  void updatePeriods();

  double swing_time_;
  double stance_time_;
  double T_stance_;
  double T_swing_;
  bool trigger_swing_;
  std::atomic<double> swing_frequency_;
  std::atomic<double> duty_factor_;
  std::atomic<bool> cycle_ended_;

  enum states {SWING=0,STANCE};
  unsigned int state_;
  unsigned int prev_state_;

};

} // namespace

#endif
