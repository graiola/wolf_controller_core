/**
WoLF: WoLF: Whole-body Locomotion Framework for quadruped robots (c) by Gennaro Raiola

WoLF is licensed under a license Creative Commons Attribution-NonCommercial-NoDerivatives 4.0 International License.

You should have received a copy of the license along with this
work. If not, see <http://creativecommons.org/licenses/by-nc-nd/4.0/>.
**/

#ifndef WPG_GAIT_GENERATOR_H
#define WPG_GAIT_GENERATOR_H

#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/StdVector>
#include <atomic>
#include <map>
#include <wolf_controller_core/common.h>
#include <wolf_controller_core/wpg/foot_state_machine.h>
#include <wolf_controller_core/wpg/foot_trajectory_interface.h>

#include <wolf_controller_utils/geometry.h>
#include <wolf_controller_utils/tools.h>

namespace wolf_controller
{

class Gait
{

public:

  const std::string CLASS_NAME = "Gait";

  enum gait_t {CRAWL=0,TROT,ONE_FOOT_LF,ONE_FOOT_RH,ONE_FOOT_RF,ONE_FOOT_LH};
  enum trajectory_t {ELLIPSE=0};

  Gait(const std::vector<std::string>& foot_names, const gait_t& gait_type);

  void reset();

  void update();

  const std::vector<std::string>& getNextSchedule();

  bool isCycleEnded();

  double getVelocityFactor(const gait_t& gait_type);

private:

  typedef std::pair<std::string,unsigned int> foot_priority_t;
  foot_priority_t foot_priority_;
  std::vector<foot_priority_t> schedule_;
  unsigned int current_priority_;
  unsigned int max_priority_;
  bool cycle_ended_;
  std::vector<std::string> next_feet_to_move_;
};

class GaitGenerator
{
public:

  const std::string CLASS_NAME = "GaitGenerator";

  /**
     * @brief Shared pointer to GaitGenerator
     */
  typedef std::shared_ptr<GaitGenerator> Ptr;

  /**
     * @brief Shared pointer to const GaitGenerator
     */
  typedef std::shared_ptr<const GaitGenerator> ConstPtr;

  GaitGenerator(const std::vector<std::string>& foot_names, const Gait::gait_t& gait_type, const Gait::trajectory_t& trajectory_type = Gait::ELLIPSE);

  void reset();

  const std::vector<std::string>& getFootNames();

  void switchGait();

  void setGaitType(const Gait::gait_t& gait_type);

  const Eigen::Affine3d& getReference(const std::string& foot_name);

  const Eigen::Vector6d& getReferenceDot(const std::string& foot_name);

  bool isSwinging(const std::string& foot_name);

  bool isInStance(const std::string& foot_name);

  bool isStateChanged(const std::string& foot_name);

  bool isTouchDown(const std::string& foot_name);

  bool isLiftOff(const std::string& foot_name);

  bool isCycleEnded(const std::string& foot_name);

  bool isGaitCycleEnded();

  bool isAnyFootInLiftOff();

  bool isAnyFootInSwing();

  bool isAnyFootInTouchDown();

  bool isAnyFootInStance();

  bool areAllFeetInStance();

  unsigned int getNumberFeetInStance();

  unsigned int getNumberFeetInSwing();

  void setContactState(const std::string& foot_name, const bool& contact, const Eigen::Vector3d& contact_force);

  const bool& getContact(const std::string& foot_name);

  const Eigen::Vector3d& getContactForce(const std::string& foot_name);

  void setInitialPose(const std::string& foot_name, const Eigen::Affine3d& initial_pose);

  double getDutyFactor(const std::string& foot_name);

  void setDutyFactor(const double& duty_factor);

  void setDutyFactor(const std::string& foot_name, const double& duty_factor);

  void setSwingFrequency(const double& swing_frequency);

  void increaseSwingFrequency();

  void decreaseSwingFrequency();

  void increaseDutyFactor();

  void decreaseDutyFactor();

  void setSwingFrequency(const std::string& foot_name, const double& swing_frequency);

  double getSwingFrequency(const std::string& foot_name);

  double getAvgSwingFrequency();

  double getAvgStanceFrequency();

  double getAvgCycleTime();

  double getAvgDutyFactor();

  double getAvgTrajectoryCompletion();

  double getTrajectoryCompletion(const std::string& foot_name);

  double getStancePeriod(const std::string& foot_name);

  double getSwingPeriod(const std::string& foot_name);

  Gait::gait_t getGaitType();

  double getVelocityFactor();

  void setTerrainTransform(const Eigen::Affine3d& world_T_terrain);

  void setStepLength(const double& length);

  void setStepHeading(const double& heading);

  void setStepHeading(const std::string& foot_name, const double& heading);

  void setStepHeadingRate(const double& yaw_rate);

  void setStepHeadingRate(const std::string& foot_name, const double& heading_rate);

  void setStepHeight(const double& height);

  void setStepLength(const std::string& foot_name, const double& length);

  void setStepHeight(const std::string& foot_name, const double& height);

  void activateSwing();

  void deactivateSwing();

  bool isTrajectoryFinished(const std::string& foot_name);

  bool isFirstStep();

  bool isLastStep();

  void update(const double& period);

  void startStepReflex(bool start);

  void toggleStepReflex();

  bool isStepReflexActive();

  void setStepReflexContactThreshold(const double &th);

  void setStepReflexMaxRetraction(const double &max);

private:

  void changeGait();

  struct feet_status_t
  {
    std::shared_ptr<FootStateMachine> state_machine;
    std::shared_ptr<TrajectoryInterface> trajectory;
    bool contact;
    Eigen::Vector3d contact_force;
    bool trigger_stance;
    Eigen::Affine3d initial_pose;
  };

  TrajectoryInterface* selectTrajectoryType(Gait::trajectory_t trajectory_type);

  typedef std::map<std::string,feet_status_t> feet_t;
  typedef std::shared_ptr<Gait> gait_ptr_t;

  std::vector<std::string> scheduled_feet_;
  std::vector<std::string> foot_names_;

  feet_t feet_;
  std::vector<gait_ptr_t > gait_buffer_;
  std::atomic<unsigned int> current_gait_idx_;
  std::atomic<unsigned int> next_gait_idx_;
  std::atomic<bool> change_gait_;
  std::atomic<bool> activate_swing_;
  wolf_controller_utils::Trigger next_schedule_;
  wolf_controller_utils::Trigger first_step_;
  wolf_controller_utils::Trigger last_step_;

  Gait::gait_t gait_type_;

  std::atomic<bool> step_reflex_active_;

};

} // namespace


#endif
