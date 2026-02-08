/**
 * @file impedance.cpp
 * @author Gennaro Raiola
 * @date 1 November, 2021
 * @brief This file contains the impedance gains generator
 */

#include "wolf_controller_core/impedance.h"

using namespace wolf_wbid;

namespace wolf_controller {

void Impedance::loadMatrices()
{
    Kp_swing_leg_(0,0) = kp_swing_haa_;
    Kp_swing_leg_(1,1) = kp_swing_hfe_;
    Kp_swing_leg_(2,2) = kp_swing_kfe_;
    Kd_swing_leg_(0,0) = kd_swing_haa_;
    Kd_swing_leg_(1,1) = kd_swing_hfe_;
    Kd_swing_leg_(2,2) = kd_swing_kfe_;

    Kp_stance_leg_(0,0) = kp_stance_haa_;
    Kp_stance_leg_(1,1) = kp_stance_hfe_;
    Kp_stance_leg_(2,2) = kp_stance_kfe_;
    Kd_stance_leg_(0,0) = kd_stance_haa_;
    Kd_stance_leg_(1,1) = kd_stance_hfe_;
    Kd_stance_leg_(2,2) = kd_stance_kfe_;
}

void Impedance::loadValues()
{
    kp_swing_haa_  = Kp_swing_leg_(0,0);
    kp_swing_hfe_  = Kp_swing_leg_(1,1);
    kp_swing_kfe_  = Kp_swing_leg_(2,2);
    kd_swing_haa_  = Kd_swing_leg_(0,0);
    kd_swing_hfe_  = Kd_swing_leg_(1,1);
    kd_swing_kfe_  = Kd_swing_leg_(2,2);

    kp_stance_haa_ = Kp_stance_leg_(0,0);
    kp_stance_hfe_ = Kp_stance_leg_(1,1);
    kp_stance_kfe_ = Kp_stance_leg_(2,2);
    kd_stance_haa_ = Kd_stance_leg_(0,0);
    kd_stance_hfe_ = Kd_stance_leg_(1,1);
    kd_stance_kfe_ = Kd_stance_leg_(2,2);
}

Impedance::Impedance(StateEstimator::Ptr state_estimator, QuadrupedRobot::Ptr robot_model)
{
    robot_model_ = robot_model;
    state_estimator_ = state_estimator;
    inertia_compensation_active_ = true;

    // Initialize the inertia related matrices
    robot_model_->getInertiaMatrix(M_);
    Kp_.setIdentity(M_.rows(), M_.cols());
    Kd_.setIdentity(M_.rows(), M_.cols());

    if(robot_model_->getLegNames().empty()) {
      Mi_legs_.resize(0,0);
    } else {
      Mi_legs_.setZero(3,3);
    }
    Kp_swing_leg_.setZero();
    Kd_swing_leg_.setZero();
    Kp_stance_leg_.setZero();
    Kd_stance_leg_.setZero();

    if(robot_model_->getNumberArms() >0)
    {
      if(robot_model_->getArmNames().empty()) {
        Mi_arms_.resize(0,0);
      } else {
        const auto& arm_ids = robot_model_->getLimbJointsIds(robot_model_->getArmNames()[0]);
        Mi_arms_.setZero(static_cast<int>(arm_ids.size()), static_cast<int>(arm_ids.size()));
      }
      Kp_arm_.setZero();
      Kd_arm_.setZero();
    }
}

void Impedance::startInertiaCompensation(const bool& start)
{
    inertia_compensation_active_ = start;
}

void Impedance::setLegsGains(const Eigen::Vector3d& Kp_leg, const Eigen::Vector3d& Kd_leg)
{
    Kp_swing_leg_  = Kp_leg.asDiagonal();
    Kd_swing_leg_  = Kd_leg.asDiagonal();
    Kp_stance_leg_ = Kp_leg.asDiagonal();
    Kd_stance_leg_ = Kd_leg.asDiagonal();

    loadValues();
}

void Impedance::setLegsGains(const Eigen::Vector3d& Kp_swing_leg, const Eigen::Vector3d& Kd_swing_leg,
                             const Eigen::Vector3d& Kp_stance_leg, const Eigen::Vector3d& Kd_stance_leg)
{
    Kp_swing_leg_  = Kp_swing_leg.asDiagonal();
    Kd_swing_leg_  = Kd_swing_leg.asDiagonal();
    Kp_stance_leg_ = Kp_stance_leg.asDiagonal();
    Kd_stance_leg_ = Kd_stance_leg.asDiagonal();

    loadValues();
}

void Impedance::setArmsGains(const Eigen::VectorXd& Kp_arm, const Eigen::VectorXd& Kd_arm)
{
    Kp_arm_  = Kp_arm.asDiagonal();
    Kd_arm_  = Kd_arm.asDiagonal();
}

void Impedance::update()
{
    const std::vector<std::string>& foot_names = robot_model_->getFootNames();
    const std::vector<std::string>& leg_names  = robot_model_->getLegNames();
    const std::vector<std::string>& ee_names   = robot_model_->getEndEffectorNames();
    const std::vector<std::string>& arm_names  = robot_model_->getArmNames();

    loadMatrices();

    // Feet
    if(foot_names.size() != leg_names.size())
    {
        PRINT_WARN_NAMED(CLASS_NAME,"Foot/leg name mismatch: feet=" << foot_names.size()
                         << " legs=" << leg_names.size());
    }
    for(unsigned int i=0;i<foot_names.size() && i<leg_names.size();i++)
    {
        const auto& limb_ids = robot_model_->getLimbJointsIds(leg_names[i]);
        if(limb_ids.size() < 3)
        {
            PRINT_WARN_NAMED(CLASS_NAME,"Limb '" << leg_names[i] << "' has " << limb_ids.size()
                             << " joints; expected 3. Skipping impedance for this limb.");
            continue;
        }
        int idx = static_cast<int>(limb_ids.front()); // NOTE: take the first idx, joints are contiguous after sorting
        if(idx < 0 || idx + 3 > Kp_.rows() || idx + 3 > Kp_.cols())
        {
            PRINT_WARN_NAMED(CLASS_NAME,"Invalid limb joint index range for '" << leg_names[i]
                             << "': idx=" << idx << " Kp size=" << Kp_.rows());
            continue;
        }

        if(!state_estimator_->getContact(foot_names[i]))
        {
            if(inertia_compensation_active_)
            {
                robot_model_->getLimbInertiaInverse(leg_names[i],Mi_legs_);
                Kp_.block<3,3>(idx,idx) = Mi_legs_ * Kp_swing_leg_;
                Kd_.block<3,3>(idx,idx) = Mi_legs_ * Kd_swing_leg_;
            }
            else
            {
                Kp_.block<3,3>(idx,idx) = Kp_swing_leg_;
                Kd_.block<3,3>(idx,idx) = Kd_swing_leg_;
            }
        }
        else
        {
            if(inertia_compensation_active_)
            {
                robot_model_->getLimbInertiaInverse(leg_names[i],Mi_legs_);
                Kp_.block<3,3>(idx,idx) = Mi_legs_ * Kp_stance_leg_;
                Kd_.block<3,3>(idx,idx) = Mi_legs_ * Kd_stance_leg_;
            }
            else
            {
                Kp_.block<3,3>(idx,idx) = Kp_stance_leg_;
                Kd_.block<3,3>(idx,idx) = Kd_stance_leg_;
            }
        }
    }

    // End-effectors
    if(ee_names.size() != arm_names.size())
    {
        PRINT_WARN_NAMED(CLASS_NAME,"EE/arm name mismatch: ee=" << ee_names.size()
                         << " arms=" << arm_names.size());
    }
    for(unsigned int i=0;i<ee_names.size() && i<arm_names.size();i++)
    {
        const auto& limb_ids = robot_model_->getLimbJointsIds(arm_names[i]);
        int n = static_cast<int>(limb_ids.size());
        if(n == 0)
        {
            PRINT_WARN_NAMED(CLASS_NAME,"Arm '" << arm_names[i] << "' has 0 joints; skipping impedance.");
            continue;
        }
        int idx = static_cast<int>(limb_ids.front()); // NOTE: take the first idx, joints are contiguous after sorting
        if(idx < 0 || idx + n > Kp_.rows() || idx + n > Kp_.cols())
        {
            PRINT_WARN_NAMED(CLASS_NAME,"Invalid arm joint index range for '" << arm_names[i]
                             << "': idx=" << idx << " n=" << n << " Kp size=" << Kp_.rows());
            continue;
        }

        if(inertia_compensation_active_)
        {
            robot_model_->getLimbInertiaInverse(arm_names[i],Mi_arms_);
            Kp_.block(idx,idx,n,n) = Mi_arms_ * Kp_arm_;
            Kd_.block(idx,idx,n,n) = Mi_arms_ * Kd_arm_;
        }
        else
        {
            Kp_.block(idx,idx,n,n) = Kp_arm_;
            Kd_.block(idx,idx,n,n) = Kd_arm_;
        }
    }
}

void Impedance::setKpSwingLegHAA(const double &value)
{
    assert(value>=0.0);
    kp_swing_haa_=value;
    PRINT_INFO_NAMED(CLASS_NAME,"Set Kp swing HAA: "<<value);
}

void Impedance::setKpSwingLegHFE(const double &value)
{
    assert(value>=0.0);
    kp_swing_hfe_=value;
    PRINT_INFO_NAMED(CLASS_NAME,"Set Kp swing HFE: "<<value);
}

void Impedance::setKpSwingLegKFE(const double &value)
{
    assert(value>=0.0);
    kp_swing_kfe_=value;
    PRINT_INFO_NAMED(CLASS_NAME,"Set Kp swing KFE: "<<value);
}

void Impedance::setKdSwingLegHAA(const double &value)
{
    assert(value>=0.0);
    kd_swing_haa_=value;
    PRINT_INFO_NAMED(CLASS_NAME,"Set Kd swing HAA: "<<value);
}

void Impedance::setKdSwingLegHFE(const double &value)
{
    assert(value>=0.0);
    kd_swing_hfe_=value;
    PRINT_INFO_NAMED(CLASS_NAME,"Set Kd swing HFE: "<<value);
}

void Impedance::setKdSwingLegKFE(const double &value)
{
    assert(value>=0.0);
    kd_swing_kfe_=value;
    PRINT_INFO_NAMED(CLASS_NAME,"Set Kd swing KFE: "<<value);
}

void Impedance::setKpStanceLegHAA(const double &value)
{
    assert(value>=0.0);
    kp_stance_haa_=value;
    PRINT_INFO_NAMED(CLASS_NAME,"Set Kp stance HAA: "<<value);
}

void Impedance::setKpStanceLegHFE(const double &value)
{
    assert(value>=0.0);
    kp_stance_hfe_=value;
    PRINT_INFO_NAMED(CLASS_NAME,"Set Kp stance HFE: "<<value);
}

void Impedance::setKpStanceLegKFE(const double &value)
{
    assert(value>=0.0);
    kp_stance_kfe_=value;
    PRINT_INFO_NAMED(CLASS_NAME,"Set Kp stance KFE: "<<value);
}

void Impedance::setKdStanceLegHAA(const double &value)
{
    assert(value>=0.0);
    kd_stance_haa_=value;
    PRINT_INFO_NAMED(CLASS_NAME,"Set Kd stance HAA: "<<value);
}

void Impedance::setKdStanceLegHFE(const double &value)
{
    assert(value>=0.0);
    kd_stance_hfe_=value;
    PRINT_INFO_NAMED(CLASS_NAME,"Set Kd stance HFE: "<<value);
}

void Impedance::setKdStanceLegKFE(const double &value)
{
    assert(value>=0.0);
    kd_stance_kfe_=value;
    PRINT_INFO_NAMED(CLASS_NAME,"Set Kd stance KFE: "<<value);
}

const Eigen::MatrixXd& Impedance::getKp() const
{
    return Kp_;
}

const Eigen::MatrixXd& Impedance::getKd() const
{
    return Kd_;
}

} // namespace
