/**
WoLF: WoLF: Whole-body Locomotion Framework for quadruped robots (c) by Gennaro Raiola

WoLF is licensed under a license Creative Commons Attribution-NonCommercial-NoDerivatives 4.0 International License.

You should have received a copy of the license along with this
work. If not, see <http://creativecommons.org/licenses/by-nc-nd/4.0/>.
**/

#ifndef STATE_MACHINE_H
#define STATE_MACHINE_H

#include <memory>
#include <vector>
#include <map>
#include <atomic>

#include <wolf_controller_utils/tools.h>

namespace wolf_controller {

class ControllerCore;
class StateMachine;

class State {
public:
  typedef std::shared_ptr<State> Ptr;

  virtual ~State() = default;
  virtual void updateStateMachine(StateMachine* state_machine, const double& dt) = 0;
  virtual void onEntry(StateMachine* state_machine) {};
  virtual void onExit(StateMachine* state_machine) {};
};

class ControllerIdleState : public State {
public:
  void updateStateMachine(StateMachine* state_machine, const double& dt) override;
  virtual void onEntry(StateMachine* state_machine) override;
  virtual void onExit(StateMachine* state_machine) override;
};

class ControllerInitState : public State {
public:
  ControllerInitState();
  void updateStateMachine(StateMachine* state_machine, const double& dt) override;
  virtual void onEntry(StateMachine* state_machine) override;
  virtual void onExit(StateMachine* state_machine) override;
  wolf_controller_utils::Ramp::Ptr ramp_;
};

class ControllerStandingUpState : public State {
public:
  ControllerStandingUpState();
  void updateStateMachine(StateMachine* state_machine, const double& dt) override;
  virtual void onEntry(StateMachine* state_machine) override;
  virtual void onExit(StateMachine* state_machine) override;
  wolf_controller_utils::Ramp::Ptr ramp_;
};

class ControllerActiveState : public State {
public:
  void updateStateMachine(StateMachine* state_machine, const double& dt) override;
  virtual void onEntry(StateMachine* state_machine) override;
  virtual void onExit(StateMachine* state_machine) override;
};

class ControllerStandingDownState : public State {
public:
  ControllerStandingDownState();
  void updateStateMachine(StateMachine* state_machine, const double& dt) override;
  virtual void onEntry(StateMachine* state_machine) override;
  virtual void onExit(StateMachine* state_machine) override;
  wolf_controller_utils::Ramp::Ptr ramp_;
};

class ControllerAnomalyState : public State {
public:
  void updateStateMachine(StateMachine* state_machine, const double& dt) override;
  virtual void onEntry(StateMachine* state_machine) override;
  virtual void onExit(StateMachine* state_machine) override;
};

class StateMachine
{
public:

  const std::string CLASS_NAME = "StateMachine";

  typedef std::shared_ptr<StateMachine> Ptr;

  typedef std::shared_ptr<const StateMachine> ConstPtr;

  enum states_t {IDLE,INIT,ANOMALY,STANDING_UP,STANDING_DOWN,ACTIVE,N_STATES=7};

  StateMachine(ControllerCore* controller);

  void updateStateMachine(const double& dt);

  void setCurrentState(states_t state);

  states_t getCurrentState();

  states_t getPreviousState();

  std::string getStateAsString();

  std::vector<std::string> getStatesAsString();

  ControllerCore* getController();

private:

  std::atomic<bool> state_changed_;
  ControllerCore* controller_;
  std::atomic<states_t> current_state_;
  std::atomic<states_t> previous_state_;
  std::string current_state_str_;
  std::map<states_t,State::Ptr> states_;

};

}

#endif
