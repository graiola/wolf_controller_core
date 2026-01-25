/**
WoLF: WoLF: Whole-body Locomotion Framework for quadruped robots (c) by Gennaro Raiola

WoLF is licensed under a license Creative Commons Attribution-NonCommercial-NoDerivatives 4.0 International License.

You should have received a copy of the license along with this
work. If not, see <http://creativecommons.org/licenses/by-nc-nd/4.0/>.
**/

#ifndef WOLF_CONTROLLER_CORE_H
#define WOLF_CONTROLLER_CORE_H

// STD
#include <atomic>
#include <thread>
#include <chrono>
#include <memory>
// WoLF
#include <wolf_controller_core/impedance.h>
#include <wolf_controller_core/wpg/gait_generator.h>
#include <wolf_controller_core/wpg/footholds_planner.h>
#include <wolf_controller_core/wpg/com_planner.h>
#include <wolf_controller_core/state_estimator.h>
#include <wolf_controller_core/terrain_estimator.h>
#include <wolf_controller_utils/tools.h>
#include <wolf_wbid/id_problem.h>

// Eigen
#include <Eigen/Geometry>

namespace wolf_controller
{

// Forward state machine declarations
class StateMachine;
class State;
class ControllerInitState;
class ControllerIdleState;
class ControllerStandingUpState;
class ControllerActiveState;
class ControllerStandingDownState;
class ControllerAnomalyState;

class ControllerCore
{
public:

  // State machine friend classes
  friend class StateMachine;
  friend class State;
  friend class ControllerInitState;
  friend class ControllerIdleState;
  friend class ControllerStandingUpState;
  friend class ControllerActiveState;
  friend class ControllerStandingDownState;
  friend class ControllerAnomalyState;

  enum posture_t {UP=0,DOWN};
  enum mode_t {WPG=0,EXT,MPC,RESET,N_MODES=4};

  const std::string CLASS_NAME = "ControllerCore";

  /**
     * @brief Shared pointer to ControllerCore
     */
  typedef std::shared_ptr<ControllerCore> Ptr;

  /**
     * @brief Weak pointer to ControllerCore
     */
  typedef std::weak_ptr<ControllerCore> WeakPtr;

  /**
     * @brief Shared pointer to const ControllerCore
     */
  typedef std::shared_ptr<const ControllerCore> ConstPtr;

  /** @brief Constructor function */
  ControllerCore();

  /** @brief Destructor function */
  ~ControllerCore();

  /**
         * @brief Initializes controller core
         * @param const double& period
         * @param const std::string& urdf
         * @param const std::string& srdf
         * @param const std::string& robot_name
         */
  bool init(const double& period, const std::string& urdf, const std::string& srdf, const std::string& robot_name);


  /**
         * @brief Updates the controller according to the control period, be sure to setJointState and setImu before calling it
         * @param const double& dt
         */
  void update(const double& dt);

  /**
         * @brief Reset the controller
         */
  void reset();

  /**
         * @brief set the joint state
         * @param Joint positions
         * @param Joint velocities
         * @param Joint accelerations
         * @param Joint efforts
         */
  void setJointState(const Eigen::VectorXd& pos,
                     const Eigen::VectorXd& vel,
                     const Eigen::VectorXd& acc,
                     const Eigen::VectorXd& effort);

  /**
         * @brief set the joint position
         * @param index
         * @param value
         */
  void setJointPosition(const unsigned int& i, const double& value);

  /**
         * @brief set the joint velocity
         * @param index
         * @param value
         */
  void setJointVelocity(const unsigned int& i, const double& value);

  /**
         * @brief set the joint acceleration
         * @param index
         * @param value
         */
  void setJointAcceleration(const unsigned int& i, const double& value);

  /**
         * @brief set the joint effort
         * @param index
         * @param value
         */
  void setJointEffort(const unsigned int& i, const double& value);

  /**
         * @brief set the imu reading
         * @param orientation
         * @param gyroscope
         * @param accelerometer
         */
  void setImu(const Eigen::Quaterniond& q,
              const Eigen::Vector3d& gyro,
              const Eigen::Vector3d& acc);

  /**
         * @brief set the imu orientation
         * @param orientation
         */
  void setImuOrientation(const Eigen::Quaterniond& q);

  /**
         * @brief set the imu gyroscope
         * @param gyroscope
         */
  void setImuGyroscope(const Eigen::Vector3d& gyro);

  /**
         * @brief set the imu gyroscope
         * @param accelerometer
         */
  void setImuAccelerometer(const Eigen::Vector3d& acc);


  /**
         * @brief set an external estimated state for example a ground truth
         */
  void setExtEstimatedState(const Eigen::Vector3d& lin_pos,
                            const Eigen::Vector3d& lin_vel,
                            const Eigen::Vector3d& lin_acc,
                            const Eigen::Quaterniond& q,
                            const Eigen::Vector3d& ang_vel);

  /**
         * @brief set an external estimated contact state for example from a contact sensor
         */
  void setExtEstimatedContactStates(const std::map<std::string,std::pair<bool,Eigen::Vector3d> >& states);

  /**
         * @brief set an external estimated contact state for example from a contact sensor
         */
  void setExtEstimatedContactState(const std::string& contact_name, const bool& state, const Eigen::Vector3d& force);

  /**
         * @brief get the robot name
         */
  const std::string& getRobotName();

  /**
         * @brief get the robot's joint names
         */
  const std::vector<std::string>& getJointNames();

  /**
         * @brief Set the base linear velocity command along X
         * @param v
         */
  void setBaseLinearVelocityCmdX(const double& v);

  /**
         * @brief Set the base linear velocity command along Y
         * @param v
         */
  void setBaseLinearVelocityCmdY(const double& v);

  /**
         * @brief Set the base linear velocity command along Z
         * @param v
         */
  void setBaseLinearVelocityCmdZ(const double& v);

  /**
         * @brief Set the base angular velocity command along roll
         * @param v
         */
  void setBaseAngularVelocityCmdRoll(const double& v);

  /**
         * @brief Set the base angular velocity command along pitch
         * @param v
         */
  void setBaseAngularVelocityCmdPitch(const double& v);

  /**
         * @brief Set the base angular velocity command along yaw
         * @param v
         */
  void setBaseAngularVelocityCmdYaw(const double& v);

  /**
         * @brief Get the base linear velocity command along X
         * @return x velocity
         */
  double getBaseLinearVelocityCmdX();

  /**
         * @brief Get the base linear velocity command along Y
         * @return y velocity
         */
  double getBaseLinearVelocityCmdY();

  /**
         * @brief Get the base linear velocity command along Z
         * @return z velocity
         */
  double getBaseLinearVelocityCmdZ();

  /**
         * @brief Get the base angular velocity command along roll
         * @return roll velocity
         */
  double getBaseAngularVelocityCmdRoll();

  /**
         * @brief Get the base angular velocity command along pitch
         * @return pitch velocity
         */
  double getBaseAngularVelocityCmdPitch();

  /**
         * @brief Get the base angular velocity command along yaw
         * @return yaw velocity
         */
  double getBaseAngularVelocityCmdYaw();

  /**
         * @brief Set the duty factor for the feet
         * @param const double duty_factor
         */
  bool setDutyFactor(const double& duty_factor);

  /**
         * @brief Set the mu value for the friction cones
         * @param mu [0,1]
         */
  bool setFrictionConesMu(const double &mu);

  /**
         * @brief Set the swing frequency
         * @param const double& swing_frequency
         */
  bool setSwingFrequency(const double& swing_frequency);

  /**
         * @brief Set the step height
         * @param const double& swing_frequency
         */
  bool setStepHeight(const double& step_height);

  /**
         * @brief set cutoff frequency for the qdot filter
         */
  void setCutoffFreqQdot(const double& hz);

  /**
         * @brief set cutoff frequency for the imu gyroscope filter
         */
  void setCutoffFreqGyroscope(const double& hz);

  /**
         * @brief set cutoff frequency for the imu accelerometer filter
         */
  void setCutoffFreqAccelerometer(const double& hz);

  /**
         * @brief Select the control mode to use
         */
  bool selectControlMode(const std::string& mode);

  /**
         * @brief get the current control mode
         */
  unsigned int getControlMode();

  /**
         * @brief Switch between WPG and MPC
         */
  void switchControlMode();

  /**
         * @brief select the posture [UP|DOWN]
         */
  bool selectPosture(const std::string& posture);

  /**
         * @brief get the posture
         */
  posture_t getPosture();

  /**
         * @brief switch posture between UP and DOWN
         */
  void switchPosture();

  /**
         * @brief stand up
         */
  void standUp(bool stand_up);

  /**
         * @brief emergency stop
         */
  void emergencyStop();

  /**
         * @brief reset base orientation (roll and pitch) and base height
         */
  void resetBase();

  /**
         * @brief Select the gait to use
         */
  bool selectGait(const std::string& gait);

  /**
         * @brief Switch between TROT and CRAWL gait
         */
  void switchGait();

  /**
         * @brief Get desired contact forces
         */
  std::vector<Eigen::Vector6d>& getDesiredContactForces();

  /**
         * @brief Get desired contact states
         */
  std::vector<bool>& getDesiredContactStates();

  /**
         * @brief Get desired joint efforts
         */
  const Eigen::VectorXd& getDesiredJointEfforts() const;

  /**
         * @brief Get the current control mode
         */
  std::string getModeAsString();

  /**
         * @brief Get the available control modes
         */
  std::vector<std::string> getModesAsString();

  /**
         * @brief Get the id problem
         */
  wolf_wbid::IDProblem* getIDProblem() const;

  /**
         * @brief Get the gait generator pointer
         */
  GaitGenerator* getGaitGenerator() const;

  /**
         * @brief Get the gait commands interface pointer
         */
  FootholdsPlanner* getFootholdsPlanner() const;

  /**
         * @brief Get the state estimator pointer
         */
  StateEstimator* getStateEstimator() const;

  /**
         * @brief Get the impedance pointer
         */
  Impedance* getImpedance() const;

  /**
         * @brief Get the terrain estimator pointer
         */
  TerrainEstimator* getTerrainEstimator() const;

  /**
         * @brief Get Robot Model
         */
  wolf_wbid::QuadrupedRobot* getRobotModel() const;

  /**
         * @brief Get State Machine
         */
  StateMachine* getStateMachine() const;

private:

  /** @brief Number of joints */
  int n_joints_;
  /** @brief Robot name */
  std::string robot_name_;
  /** @brief Joint names */
  std::vector<std::string> joint_names_;
  /** @brief Control period */
  double period_;
  /** @brief Joint positions */
  Eigen::VectorXd joint_positions_;
  /** @brief Initial joint positions */
  Eigen::VectorXd joint_positions_init_;
  /** @brief Joint velocities */
  Eigen::VectorXd joint_velocities_;
  /** @brief Joint velocities */
  Eigen::VectorXd joint_velocities_filt_;
  /** @brief Joint accellerations */
  Eigen::VectorXd joint_accellerations_;
  /** @brief Joint efforts */
  Eigen::VectorXd joint_efforts_;
  /** @brief Desired joint positions */
  Eigen::VectorXd des_joint_positions_;
  /** @brief Desired joint velocities */
  Eigen::VectorXd des_joint_velocities_;
  /** @brief Desired joint efforts computed by the solver */
  Eigen::VectorXd des_joint_efforts_solver_;
  /** @brief Desired joint efforts computed by the impedance control */
  Eigen::VectorXd des_joint_efforts_impedance_;
  /** @brief Desired joint efforts sent to the hardware interface */
  Eigen::VectorXd des_joint_efforts_;
  /** @brief Quadruped robot model */
  wolf_wbid::QuadrupedRobot::Ptr robot_model_;
  /** @brief Impedance pointer */
  Impedance::Ptr impedance_;
  /** @brief Dynamic problem formulation */
  wolf_wbid::IDProblem::UniquePtr id_prob_;
  /** @brief Desired contact forces */
  std::vector<Eigen::Vector6d> des_contact_forces_;
  /** @brief Desired contact forces */
  std::vector<bool> des_contact_states_;
  /** @brief IMU Accelerometer */
  Eigen::Vector3d imu_accelerometer_;
  /** @brief IMU Accelerometer filtered */
  Eigen::Vector3d imu_accelerometer_filt_;
  /** @brief IMU Gyroscope */
  Eigen::Vector3d imu_gyroscope_;
  /** @brief IMU Gyroscope filtered */
  Eigen::Vector3d imu_gyroscope_filt_;
  /** @brief IMU Orientation */
  Eigen::Quaterniond imu_orientation_;
  /** @brief Gait generator */
  GaitGenerator::Ptr gait_generator_;
  /** @brief Terrain Estimator */
  TerrainEstimator::Ptr terrain_estimator_;
  /** @brief Foot holds Planner */
  FootholdsPlanner::Ptr foot_holds_planner_;
  /** @brief CoM Planner */
  ComPlanner::Ptr com_planner_;
  /** @brief State estimator */
  StateEstimator::Ptr state_estimator_;
  /** @brief qdot_filter */
  XBot::Utils::SecondOrderFilter<Eigen::VectorXd> qdot_filter_;
  /** @brief imu_gyroscope_filter */
  XBot::Utils::SecondOrderFilter<Eigen::Vector3d> imu_gyroscope_filter_;
  /** @brief imu_accelerometer filter */
  XBot::Utils::SecondOrderFilter<Eigen::Vector3d> imu_accelerometer_filter_;
  /** @brief Linear and angular velocities */
  std::atomic<double> vel_x_;
  std::atomic<double> vel_y_;
  std::atomic<double> vel_z_;
  std::atomic<double> vel_roll_;
  std::atomic<double> vel_pitch_;
  std::atomic<double> vel_yaw_;
  /** @brief Counters used for checks */
  wolf_controller_utils::Counter::Ptr solver_failures_cnt_;
  wolf_controller_utils::Counter::Ptr contact_failures_cnt_;
  std::vector<wolf_controller_utils::Counter::Ptr> velocity_lims_failures_cnt_;
  /** @brief State machine support variables */
  std::string mode_string_;
  mode_t current_mode_;
  mode_t requested_mode_;
  mode_t previous_mode_;
  posture_t posture_;
  std::shared_ptr<StateMachine> state_machine_;

  /**
         * @brief update the state estimator
         * @param dt control period
         */
  void updateStateEstimator(const double& dt);

  /**
         * @brief update the terrain estimator
         * @param dt control period
         */
  void updateTerrainEstimator(const double& dt);

  /**
         * @brief update the state machine
         * @param dt control period
         */
  void updateStateMachine(const double &dt);

  /**
         * @brief perform an execution step with the solver
         * @param des_joint_positions
         * @return false if the solver failed
         */
  bool updateSolver(const Eigen::VectorXd& des_joint_positions);

  /**
         * @brief perform an execution step with the impedance
         * @param des_joint_positions
         * @param des_joint_velocities
         */
  void updateImpedance(const Eigen::VectorXd& des_joint_positions, const Eigen::VectorXd& des_joint_velocities);

  /**
         * @brief perform an execution step with the controller's walking pattern generator
         * @param dt control period
         */
  void updateWpg(const double &dt);

  /**
         * @brief update base references
         */
  void updateBaseReferences(const Eigen::Vector3d& com_pos_ref,
                            const Eigen::Vector3d& com_vel_ref,
                            const Eigen::Matrix3d& orientation_ref);

  /**
         * @brief perform various safety checks
         * @return false if something failed
         */
  bool performSafetyChecks();

};

} //@namespace wolf_controller

#endif
