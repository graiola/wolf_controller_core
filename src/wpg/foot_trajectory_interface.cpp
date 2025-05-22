/**
 * @file foot_trajectory_interface.cpp
 * @author Gennaro Raiola, Michele Focchi
 * @date 12 June, 2019
 * @brief This file contains the interface for the foot trajectory
 * and the implementation of the step reflex
 */

#include <wolf_controller_core/wpg/foot_trajectory_interface.h>
#include <wolf_controller_utils/geometry.h>

// RT LOGGER
#ifdef RT_LOGGER
#include <rt_logger/rt_logger.h>
using namespace rt_logger;
#endif

using namespace wolf_controller;
using namespace wolf_controller_utils;

TrajectoryInterface::TrajectoryInterface()
  :trajectory_id(_id++)
  ,activate_step_reflex_(false)
  ,use_terrain_projection_(false)
{
  pose_reference_ = initial_pose_ = world_T_terrain_ = Eigen::Affine3d::Identity();
  twist_reference_.setZero();
  swing_frequency_ = 0.0;
  time_ = 0.0;
  length_ = 0.0;
  heading_ = 0.0;
  heading_rate_ = 0.0;
  height_ = 0.0;
  world_R_terrain_ = terrain_R_world_ = terrain_R_swing_ = world_Rz_swing_ = Eigen::Matrix3d::Identity();
  Sz_ = Ear_ = Eigen::Matrix3d::Zero();

  terrain_normal_ << 0.0, 0.0, 0.1;
  terrain_point_  << 0.0, 0.0, 0.0;

  trajectory_finished_ = true;
#ifdef RT_LOGGER
  RtLogger::getLogger().addPublisher(TOPIC(position_reference_)+_legs_prefix[trajectory_id],position_reference_);
  RtLogger::getLogger().addPublisher(TOPIC(velocity_reference_)+_legs_prefix[trajectory_id],velocity_reference_);
#endif

  reflex_ = std::make_shared<TrajectoryReflex>(this);
}

const Eigen::Affine3d& TrajectoryInterface::getReference()
{
  return pose_reference_;
}

const Eigen::Vector6d& TrajectoryInterface::getReferenceDot()
{
  return twist_reference_;
}

const Eigen::Affine3d& TrajectoryInterface::getInitialPose()
{
  return initial_pose_;
}

void TrajectoryInterface::setInitialPose(const Eigen::Affine3d& initial_pose)
{
#ifdef OPEN_LOOP_TRAJECTORY
  // Open loop trajectory
  initial_pose_ = pose_reference_;
#else
  // Closed loop trajectory: to be used if the tracking is good
  initial_pose_ = initial_pose;
  pose_reference_ = initial_pose;
#endif
}

bool TrajectoryInterface::isFinished()
{
  return trajectory_finished_;
}

void TrajectoryInterface::start()
{
  time_ = 0.0;
  twist_reference_.setZero();
  trajectory_finished_ = false;
  compute_reflex_trajectory_ = false;
  reflex_->standBy();
}

void TrajectoryInterface::stop()
{
  twist_reference_.setZero();
}

void TrajectoryInterface::setStepHeading(const double& heading)
{
  heading_ = heading;
}

void TrajectoryInterface::setStepHeadingRate(const double& heading_rate)
{
  heading_rate_ = heading_rate;
}

void TrajectoryInterface::setTerrainTransform(const Eigen::Affine3d& world_T_terrain)
{
  world_T_terrain_ = world_T_terrain;
}

void TrajectoryInterface::setStepHeight(const double& height)
{
  height_ = height;
}

void TrajectoryInterface::setStepLength(const double& length)
{
  length_ = length;
}

const double& TrajectoryInterface::getStepLength() const
{
  return length_;
}

const double& TrajectoryInterface::getStepHeight() const
{
  return height_;
}

const double& TrajectoryInterface::getStepHeading() const
{
  return heading_;
}

const double& TrajectoryInterface::getStepHeadingRate() const
{
  return heading_rate_;
}

const Eigen::Matrix3d& TrajectoryInterface::getTerrainRotation() const
{
  return world_R_terrain_;
}

void TrajectoryInterface::setSwingFrequency(const double& swing_frequency)
{
  if(swing_frequency >= 0.0 && swing_frequency <= 6.0)
    swing_frequency_ = swing_frequency;
  else
    PRINT_WARN_NAMED(CLASS_NAME,"Swing frequency has to be between 0.0 and 6.0 [Hz]!");
}

double TrajectoryInterface::getSwingFrequency()
{
  return swing_frequency_;
}

Eigen::Vector3d TrajectoryInterface::projectToTerrain(const Eigen::Vector3d& point)
{
  double distance = terrain_normal_.dot(point - terrain_point_);
  return point - distance * terrain_normal_;
}

void TrajectoryInterface::update(const double& period,const Eigen::Vector3d& contact_force)
{
  time_ += period;

  world_R_terrain_ = world_T_terrain_.linear();
  terrain_normal_  = world_T_terrain_.linear().col(2);
  terrain_point_   = world_T_terrain_.translation();

  // Update the rotation between the swing frame and world
  double c = std::cos(heading_);
  double s = std::sin(heading_);
  world_Rz_swing_(0,0) = c;
  world_Rz_swing_(0,1) = -s;
  world_Rz_swing_(1,0) = s;
  world_Rz_swing_(1,1) = c;
  world_Rz_swing_(2,2) = 1;

  xyz_     = trajectoryFunction(time_);
  xyz_dot_ = trajectoryFunctionDot(time_);

  // Reflex
  if(activate_step_reflex_)
  {
    if (time_<0.5*(1.0/swing_frequency_) && reflex_->checkForFrontalImpact(contact_force))
       compute_reflex_trajectory_ = true;

    if(compute_reflex_trajectory_)
       reflex_->update(period);

    xyz_ = xyz_ + reflex_->getReflex();
    xyz_dot_ = xyz_dot_ + reflex_->getReflexDot();
  }

  // Rotate the trajectory position wrt world and terrain
  terrain_R_world_.noalias() = world_R_terrain_.transpose();
  terrain_R_swing_.noalias() = terrain_R_world_ * world_Rz_swing_;
  xyz_rotated_.noalias() = terrain_R_swing_ * xyz_;

  if(!use_terrain_projection_)
    pose_reference_.translation() = initial_pose_.translation() + xyz_rotated_;
  else
  {
    // Project the z on the terrain
    swing_target_ = initial_pose_.translation() + xyz_rotated_;
    projected_target_ = projectToTerrain(swing_target_);

    double alpha = std::sin(M_PI * swing_frequency_ * time_);
    pose_reference_.translation() = swing_target_;
    pose_reference_.translation().z() = (1 - alpha) * swing_target_.z() + alpha * projected_target_.z();
  }

  position_reference_ = pose_reference_.translation(); // For visualization only

  // Rotate the trajectory velocity wrt world
  rpy_rates_.setZero();
  omegas_.setZero();
  rpy_.setZero();
  rpy_(2) = heading_;
  rpy_rates_(2) = heading_rate_;
  rpyToEarWorld(rpy_,Ear_);
  omegas_ = Ear_ * rpy_rates_;
  Sz_(0,1) = -omegas_(2);
  Sz_(1,0) = -omegas_(2);
  twist_reference_.setZero(); // No angular velocities
  // We update the terrain frame each time all the feet are in touchdown, therefore
  // the terrain estimation does not contribute to the feet velocities but it just rotates them
  twist_reference_.head(3) = terrain_R_world_ * (Sz_ * world_Rz_swing_ * xyz_ + world_Rz_swing_ * xyz_dot_);
  velocity_reference_ = twist_reference_.head(3); // For visualization only

  if(swing_frequency_*time_>=1.0)
    trajectory_finished_ = true;
}

double TrajectoryInterface::getCompletion()
{
  return swing_frequency_*time_;
}

void TrajectoryInterface::startStepReflex(bool start)
{
  activate_step_reflex_ = start;
}

void TrajectoryInterface::setStepReflexContactThreshold(const double &th)
{
  reflex_->setContactForceThreshold(th);
}

void TrajectoryInterface::setStepReflexMaxRetraction(const double &max)
{
  reflex_->setMaxStepRetraction(max);
}

TrajectoryReflex::TrajectoryReflex(TrajectoryInterface* const trajectory_interface_ptr)
{
  if(trajectory_interface_ptr)
    trajectory_interface_ptr_ = trajectory_interface_ptr;
  else
    throw std::runtime_error("TrajectoryInterface not initialized yet");

  retraction_force_angle_ = 150.0/180.0*3.14; // Default angle
  init();

  standBy();

  force_angle_lim_min_ = -120.0 * (M_PI / 180.0); // lower than 90 ° the robot is sagging because the trajectory is going down and the reflex is going up and they cancel each other
  force_angle_lim_max_ =  140.0 * (M_PI / 180.0);
  force_th_ = 10.0; // FIXME
}

void TrajectoryReflex::init()
{
  reflex_duration_ = 0.5 * (1.0/trajectory_interface_ptr_->getSwingFrequency());
  retraction_duration_ = 0.5 * reflex_duration_;
  Kd_r_ = 10.0 / reflex_duration_;
  Kp_r_ = 0.25 * (Kd_r_*Kd_r_);

  computeRetractionForce(trajectory_interface_ptr_->getStepHeight()); // Internal default Value

  r0_     = std::sqrt(trajectory_interface_ptr_->xyz_(0)*trajectory_interface_ptr_->xyz_(0) + trajectory_interface_ptr_->xyz_(2)*trajectory_interface_ptr_->xyz_(2));
  t0_     = trajectory_interface_ptr_->time_;

  r_ddot_ = r_dot_ = 0.0;
  r_  = r0_;
  Fr_ = 0.0;

  contact_force_swing_frame_.setZero();
}

void TrajectoryReflex::computeRetractionForce(const double& max_retraction)
{
  double lambda = 5.0/(reflex_duration_);
  double t_max = (-retraction_duration_*lambda*lambda*std::exp(retraction_duration_ * lambda))/(lambda*lambda*(1.0-std::exp(retraction_duration_*lambda)));
  double tmp = (1-(1+t_max*lambda)*std::exp(-t_max*lambda)) - (1-(1+(t_max-retraction_duration_)*lambda)*std::exp(-(t_max-retraction_duration_)*lambda));
  //max_retraction = height/sin(retraction_force_angle);
  //force intensity to have that max_retraction in the retractionDuration time interval
  Fr_max_ = max_retraction * Kp_r_ / tmp;
}

void TrajectoryReflex::update(const double& period)
{

  if(!init_done_)
  {
    init();
    init_done_ = true;
    //PRINT_INFO_NAMED(CLASS_NAME,"Activate step reflex for foot: "<<_legs_prefix[trajectory_interface_ptr_->trajectory_id]);
  }

  double t = trajectory_interface_ptr_->time_;
  double theta = M_PI * trajectory_interface_ptr_->getSwingFrequency() * t;

  if(t >= (retraction_duration_+t0_))
    Fr_ = 0.0;
  else
    Fr_ = Fr_max_;

  r_ddot_ = - Kp_r_ * r_ - Kd_r_ * r_dot_ + Fr_;
  r_dot_  = r_ddot_ * period + r_dot_;
  r_      = r_dot_  * period + r_;

  xyz_reflex_(0) = - r_ * std::cos(theta);
  xyz_reflex_(1) = 0.0;
  xyz_reflex_(2) = r_ * std::sin(theta);

  xyz_dot_reflex_(0) = - r_dot_ * std::cos(theta) + r_ * std::sin(theta);
  xyz_dot_reflex_(1) = 0.0;
  xyz_dot_reflex_(2) = r_dot_ * std::sin(theta) + r_ * std::cos(theta);

}

void TrajectoryReflex::standBy()
{
  init_done_ = false;
  xyz_reflex_ = xyz_dot_reflex_ = Eigen::Vector3d::Zero();
}

bool TrajectoryReflex::checkForFrontalImpact(const Eigen::Vector3d& contact_force)
{

   contact_force_swing_frame_ = trajectory_interface_ptr_->world_Rz_swing_.transpose() * contact_force;

   // Compute force angle
   double angle = std::atan2(contact_force_swing_frame_.z(), contact_force_swing_frame_.x());
   bool force_inside_limits;
   if ((angle<force_angle_lim_min_)||(angle>force_angle_lim_max_))
     force_inside_limits = true;
   else
     force_inside_limits = false;
   double force_norm_xz = std::sqrt(contact_force_swing_frame_.z()*contact_force_swing_frame_.z()+contact_force_swing_frame_.x()*contact_force_swing_frame_.x());

   bool frontal_impact_flag = (force_norm_xz > force_th_) && force_inside_limits;
   return frontal_impact_flag;
}

const Eigen::Vector3d &TrajectoryReflex::getReflex()
{
  return xyz_reflex_;
}

const Eigen::Vector3d &TrajectoryReflex::getReflexDot()
{
  return xyz_dot_reflex_;
}

void TrajectoryReflex::setContactForceAngleLimits(const double &min, const double &max)
{
  force_angle_lim_min_ = min;
  force_angle_lim_max_ = max;
}

void TrajectoryReflex::setContactForceThreshold(const double &th)
{
  if(th>0.0)
    force_th_ = th;
  else
    PRINT_WARN_NAMED(CLASS_NAME,"Contact force threshold must be positive!");
}

void TrajectoryReflex::setMaxStepRetraction(const double &max)
{
  if(max>=0.0)
    computeRetractionForce(max);
  else
    PRINT_WARN_NAMED(CLASS_NAME,"Max step retraction must be positive!");
}
