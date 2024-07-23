/**
WoLF: WoLF: Whole-body Locomotion Framework for quadruped robots (c) by Gennaro Raiola

WoLF is licensed under a license Creative Commons Attribution-NonCommercial-NoDerivatives 4.0 International License.

You should have received a copy of the license along with this
work. If not, see <http://creativecommons.org/licenses/by-nc-nd/4.0/>.
**/

#ifndef FORCE_ESTIMATOR_H
#define FORCE_ESTIMATOR_H

#include <XBotInterface/ModelInterface.h>
#include <algorithm>

namespace wolf_controller
{

class ForceTorqueSensor
{

public:

  typedef std::shared_ptr<ForceTorqueSensor> Ptr;
  typedef std::shared_ptr<const ForceTorqueSensor> ConstPtr;

  ForceTorqueSensor()
    :fx_(0), fy_(0), fz_(0), tx_(0), ty_(0), tz_(0)
  {}

  void getWrench(Eigen::Matrix<double, 6, 1>& wrench) const
  {
    wrench << fx_, fy_, fz_, tx_, ty_, tz_;
  }

  void getForce(Eigen::Vector3d& force) const
  {
    force.x() = fx_;
    force.y() = fy_;
    force.z() = fz_;
  }

  void getTorque(Eigen::Vector3d& torque) const
  {
    torque.x() = tx_;
    torque.y() = ty_;
    torque.z() = tz_;
  }

  void setWrench(const Eigen::Matrix<double, 6, 1>& wrench)
  {
    fx_ = wrench(0);
    fy_ = wrench(1);
    fz_ = wrench(2);
    tx_ = wrench(3);
    ty_ = wrench(4);
    tz_ = wrench(5);
  }

  void setForce(const Eigen::Vector3d& force)
  {
    fx_ = force.x();
    fy_ = force.y();
    fz_ = force.z();
  }

  void setTorque(const Eigen::Vector3d& torque)
  {
    tx_ = torque.x();
    ty_ = torque.y();
    tz_ = torque.z();
  }

private:

  double fx_, fy_, fz_, tx_, ty_, tz_;

};

class ForceEstimator
{

public:

  typedef std::shared_ptr<ForceEstimator> Ptr;

  typedef std::unique_ptr<ForceEstimator> UniquePtr;

  static const double DEFAULT_SVD_THRESHOLD;

  /**
     * @brief ForceEstimator constructor.
     * @param model: shared pointer to ModelInterface; client code must keep this model up to date
     * with respect to the robot state
     * @param svd_threshold: threshold for solution regularization (close to singularities)
     */
  ForceEstimator(XBot::ModelInterface::ConstPtr model,
                 double svd_threshold = DEFAULT_SVD_THRESHOLD);

  /**
    * @brief The add_link method adds one link to the list of estimated forces.
    * @param name: the name of the link
    * @param dofs: force indices to be estimated (e.g. {0,1,2} for just linear force)
    * @param chains: chain names whose joints are used for the estimation (e.g. {"left_arm"})
    * @return a shared pointer to a virtual force-torque sensor, which is updated during the
    * call to update(); use getWrench() on it to retrieve the estimated wrench.
    */
  ForceTorqueSensor::ConstPtr add_link(std::string name,
                                       std::vector<int> dofs = {},
                                       std::vector<std::string> chains = {});

  /**
     * @brief ignore measuements from the provided joints when computing
     * the force estimate
     */
  void setIgnoredJoint(const std::string& jname);

  /**
    * @brief update computes the estimation and updates all registered virtual FT-sensors
    */
  void update();

protected:

  XBot::ModelInterface::ConstPtr model_;

private:

  void compute_A_b();
  void solve();
  virtual void computeResidual(Eigen::VectorXd& res);

  struct TaskInfo
  {
    ForceTorqueSensor::Ptr sensor;
    std::vector<int> dofs;
    std::string link_name;

  };

  std::set<int> ignore_idx_;

  Eigen::MatrixXd Jtot_;
  Eigen::MatrixXd A_;
  Eigen::MatrixXd Jtmp_;

  Eigen::VectorXd y_, tau_, g_, b_, sol_;

  std::vector<TaskInfo> tasks_;
  std::set<int> meas_idx_;
  int ndofs_;

  Eigen::JacobiSVD<Eigen::MatrixXd> svd_;
  Eigen::ColPivHouseholderQR<Eigen::MatrixXd> qr_;

};

class ForceEstimatorMomentumBased : public ForceEstimator
{

public:

  typedef std::shared_ptr<ForceEstimatorMomentumBased> Ptr;

  typedef std::unique_ptr<ForceEstimatorMomentumBased> UniquePtr;

  static constexpr double DEFAULT_OBS_BW = 4.0;

  ForceEstimatorMomentumBased(XBot::ModelInterface::ConstPtr model,
                              double rate,
                              double svd_threshold = DEFAULT_SVD_THRESHOLD,
                              double obs_bw = DEFAULT_OBS_BW);

  bool getResiduals(Eigen::VectorXd &res) const;

private:

  void computeResidual(Eigen::VectorXd& res) override;
  void initMomentumObs();

  double rate_, k_obs_;

  Eigen::VectorXd y_, tau_, g_, b_, sol_;
  Eigen::VectorXd p0_, p1_, p2_, q_, qdot_, q_old_, h_, coriolis_, y_static_;
  Eigen::MatrixXd M_, M_old_, Mdot_;
};



} // namespace

#endif
