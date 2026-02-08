#include <wolf_controller_core/force_estimator.h>

using namespace wolf_controller;

const double ForceEstimator::DEFAULT_SVD_THRESHOLD = 0.05;

ForceEstimator::ForceEstimator(wolf_wbid::QuadrupedRobot::Ptr model,
                               double svd_threshold):
  model_(model),
  ndofs_(0)
{
  svd_.setThreshold(svd_threshold);
}

ForceTorqueSensor::ConstPtr ForceEstimator::add_link(std::string name,
                                                     std::vector<int> dofs,
                                                     std::vector<std::string> chains)
{
  // check link exists
  auto urdf_link = model_->getUrdf().getLink(name);

  if(!urdf_link)
  {
    throw std::invalid_argument("Invalid link '" + name + "'");
  }

  // wrench dofs if not provided
  if(dofs.size() == 0)
  {
    dofs = {0, 1, 2, 3, 4, 5};
  }

  // chains to use for estimation if not provided
  if(chains.empty())
  {
    chains = model_->getLimbNames();
  }

  // check dofs are valid
  auto it = std::find_if(dofs.begin(),
                         dofs.end(),
                         [](int dof){ return dof >= 6 || dof < 0; });
  if(it != dofs.end())
  {
    throw std::invalid_argument("Dofs must be >= 0 && < 6");
  }

  // add torque sensing dofs for the requested chains
  std::vector<int> meas_dofs;
  for(auto ch : chains)
  {
    const auto& limb_names = model_->getLimbNames();
    if(std::find(limb_names.begin(), limb_names.end(), ch) == limb_names.end())
    {
      throw std::invalid_argument("Invalid chain '" + ch + "'");
    }

    const auto& ids = model_->getLimbJointsIds(ch);
    meas_dofs.insert(meas_dofs.end(), ids.begin(), ids.end());
  }

  meas_idx_.insert(meas_dofs.begin(), meas_dofs.end());

  // remove ignored ids
  for(int ignid : ignore_idx_)
  {
    meas_idx_.erase(ignid);
  }

  // make virtual sensor and task info struct
  TaskInfo t;
  t.link_name = name;
  static int id = -1;
  t.sensor = std::make_shared<ForceTorqueSensor>();
  t.dofs = dofs;

  tasks_.push_back(t);

  ndofs_ += dofs.size();

  return t.sensor;

}

void ForceEstimator::setIgnoredJoint(const std::string &jname)
{
  int idx = model_->getJointIndex(jname);
  if(idx < 0)
  {
    throw std::invalid_argument("invalid joint '" + jname + "'");
  }

  // add to ignored ids set
  ignore_idx_.insert(idx);

  // remove from meas ids set
  meas_idx_.erase(idx);
}


void ForceEstimator::compute_A_b()
{
  Jtot_.setZero(ndofs_, model_->getJointNum());
  Jtmp_.setZero(6, model_->getJointNum());
  b_.setZero(meas_idx_.size());
  A_.setZero(meas_idx_.size(), ndofs_);

  int dof_idx = 0;
  for(TaskInfo& t : tasks_)
  {
    model_->getJacobian(t.link_name, Jtmp_);
    for(int i : t.dofs)
    {
      Jtot_.row(dof_idx++) = Jtmp_.row(i);
    }
  }

  computeResidual(y_);

  int idx = 0;
  for(int i : meas_idx_)
  {
    b_(idx) = y_(i);
    A_.row(idx) = Jtot_.col(i);
    idx++;
  }

}

void ForceEstimator::solve()
{
  svd_.compute(A_, Eigen::ComputeThinU|Eigen::ComputeThinV);
  sol_ = svd_.solve(b_);

  //    _qr.compute(_A);
  //    _sol = _qr.solve(_b);
}

void ForceEstimator::computeResidual(Eigen::VectorXd& res)
{
  model_->getJointEffort(tau_);
  model_->computeGravityCompensation(g_);

  res = g_ - tau_;

}


void ForceEstimator::update()
{
  compute_A_b();

  solve();

  int dof_idx = 0;
  for(TaskInfo& t : tasks_)
  {
    Eigen::Vector6d wrench;
    wrench.setZero();

    for(int i : t.dofs)
    {
      wrench(i) = sol_(dof_idx++);
    }

    Eigen::Matrix3d sensor_R_w;
    model_->getOrientation(t.link_name, sensor_R_w);
    sensor_R_w.transposeInPlace();

    wrench.head<3>() = sensor_R_w * wrench.head<3>();
    wrench.tail<3>() = sensor_R_w * wrench.tail<3>();

    t.sensor->setWrench(wrench);

  }
}




ForceEstimatorMomentumBased::ForceEstimatorMomentumBased(wolf_wbid::QuadrupedRobot::Ptr model,
                                                         double rate,
                                                         double svd_threshold,
                                                         double obs_bw):
  ForceEstimator(model, svd_threshold),
  k_obs_(2.0 * M_PI * obs_bw),
  rate_(rate)
{
  initMomentumObs();
}

bool ForceEstimatorMomentumBased::getResiduals(Eigen::VectorXd& res) const
{
  res = y_;
  return true;
}

void ForceEstimatorMomentumBased::computeResidual(Eigen::VectorXd& res)
{
  model_->getJointVelocity(qdot_);
  model_->getJointEffort(tau_);
  model_->computeGravityCompensation(g_);

  /* Observer */
  model_->getInertiaMatrix(M_);
  p1_ = M_ * qdot_;

  Mdot_ = (M_ - M_old_) * rate_;
  M_old_ = M_;
  model_->computeNonlinearTerm(h_);
  model_->computeGravityCompensation(g_);
  coriolis_ = h_ - g_;
  p2_ += (tau_ + (Mdot_ * qdot_ - coriolis_) - g_ + y_) / rate_;

  y_ = k_obs_*(p1_ - p2_ - p0_);

  getResiduals(res);

}

void ForceEstimatorMomentumBased::initMomentumObs()
{
  p1_.setZero(model_->getJointNum());
  p2_.setZero(model_->getJointNum());
  y_.setZero(model_->getJointNum());
  coriolis_.setZero(model_->getJointNum());
  h_.setZero(model_->getJointNum());

  model_->getInertiaMatrix(M_);
  model_->getJointPosition(q_);
  model_->getJointVelocity(qdot_);
  p0_ = M_ * qdot_;

  M_old_ = M_;
  q_old_ = q_;
}
