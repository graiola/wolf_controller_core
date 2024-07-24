/**
WoLF: WoLF: Whole-body Locomotion Framework for quadruped robots (c) by Gennaro Raiola

WoLF is licensed under a license Creative Commons Attribution-NonCommercial-NoDerivatives 4.0 International License.

You should have received a copy of the license along with this
work. If not, see <http://creativecommons.org/licenses/by-nc-nd/4.0/>.
**/

#ifndef IMPEDANCE_H
#define IMPEDANCE_H

#include <memory>
#include <atomic>
#include <Eigen/Core>
#include <wolf_controller_core/state_estimator.h>
#include <wolf_wbid/quadruped_robot.h>

namespace wolf_controller
{

class Impedance
{

public:

  const std::string CLASS_NAME = "Impedance";

  /**
   * @brief Shared pointer to Impedance
   */
  typedef std::shared_ptr<Impedance> Ptr;

  /**
   * @brief Shared pointer to const Impedance
   */
  typedef std::shared_ptr<const Impedance> ConstPtr;

  Impedance(StateEstimator::Ptr state_estimator, wolf_wbid::QuadrupedRobot::Ptr robot_model);

  void update();

  // Kp Swing
  void setKpSwingLegHAA(const double& value);
  void setKpSwingLegHFE(const double& value);
  void setKpSwingLegKFE(const double& value);
  // Kd Swing
  void setKdSwingLegHAA(const double& value);
  void setKdSwingLegHFE(const double& value);
  void setKdSwingLegKFE(const double& value);
  // Kp Stance
  void setKpStanceLegHAA(const double& value);
  void setKpStanceLegHFE(const double& value);
  void setKpStanceLegKFE(const double& value);
  // Kd Stance
  void setKdStanceLegHAA(const double& value);
  void setKdStanceLegHFE(const double& value);
  void setKdStanceLegKFE(const double& value);

  const Eigen::MatrixXd& getKp() const;
  const Eigen::MatrixXd& getKd() const;

  void startInertiaCompensation(const bool& start);

  void setLegsGains(const Eigen::Vector3d &Kp_swing_leg, const Eigen::Vector3d &Kd_swing_leg,
                    const Eigen::Vector3d &Kp_stance_leg, const Eigen::Vector3d &Kd_stance_leg);
  void setLegsGains(const Eigen::Vector3d &Kp_leg, const Eigen::Vector3d &Kd_leg);
  void setArmsGains(const Eigen::VectorXd &Kp_arm, const Eigen::VectorXd &Kd_arm);

private:

  void loadMatrices();
  void loadValues();

  wolf_wbid::QuadrupedRobot::Ptr robot_model_;
  StateEstimator::Ptr state_estimator_;

  std::atomic<double> kp_swing_haa_;
  std::atomic<double> kp_swing_hfe_;
  std::atomic<double> kp_swing_kfe_;
  std::atomic<double> kd_swing_haa_;
  std::atomic<double> kd_swing_hfe_;
  std::atomic<double> kd_swing_kfe_;

  std::atomic<double> kp_stance_haa_;
  std::atomic<double> kp_stance_hfe_;
  std::atomic<double> kp_stance_kfe_;
  std::atomic<double> kd_stance_haa_;
  std::atomic<double> kd_stance_hfe_;
  std::atomic<double> kd_stance_kfe_;

  std::atomic<bool> inertia_compensation_active_;
  Eigen::MatrixXd M_, Mi_arms_, Mi_legs_, Kp_, Kd_;
  Eigen::Matrix3d Kp_swing_leg_, Kd_swing_leg_, Kp_stance_leg_, Kd_stance_leg_;
  Eigen::MatrixXd Kp_arm_, Kd_arm_;

};


} // namespace

#endif
