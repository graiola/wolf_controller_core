/**
 * @file com_planner.cpp
 * @brief DCM-based COM reference planner for quadruped locomotion
 */

 #include <wolf_controller_core/wpg/com_planner.h>

 using namespace wolf_wbid;
 
 namespace wolf_controller {
 
 ComPlanner::ComPlanner(QuadrupedRobot::Ptr robot_model,
                        FootholdsPlanner::Ptr foothold_planner,
                        TerrainEstimator::Ptr terrain_estimator)
 {
   robot_model_ = robot_model;
   foothold_planner_ = foothold_planner;
   terrain_estimator_ = terrain_estimator;
 
   com_velocity_ref_.setZero();
   com_position_ref_.setZero();
 
   support_polygon_edges_.resize(N_LEGS);
 
   // Set filter time steps and frequencies
   com_position_ref_filter_.setTimeStep(_period);
   com_velocity_ref_filter_.setTimeStep(_period);
   com_position_ref_filter_.setOmega(2.0 * M_PI * 1.0 / foothold_planner_->getCycleTime());
   com_velocity_ref_filter_.setOmega(2.0 * M_PI * 1.0 / foothold_planner_->getCycleTime());
 
   // Initialize COM reference position
   computeComPositionReference(_period);
 }
 
 // Compute center of support polygon by averaging all feet positions
 void ComPlanner::computeSupportPolygonCenter()
 {
   auto foot_positions = robot_model_->getFeetPositionInWorld();
   auto foot_names = foothold_planner_->getFootNames();
   support_polygon_center_.setZero();
   for (unsigned int i = 0; i < foot_names.size(); i++) {
     support_polygon_center_ += foot_positions[foot_names[i]];
     support_polygon_edges_[i] = foot_positions[foot_names[i]];
   }
   support_polygon_center_ /= N_LEGS;
 }
 
 // Compute the COM velocity reference using DCM planning
 void ComPlanner::computeComVelocityReference(const double &dt)
 {
   // Get commanded base velocity and project it into the terrain frame
   base_velocity_ = foothold_planner_->getBaseLinearVelocityReference();
   com_velocity_ref_.setZero();
   com_velocity_ref_ = terrain_estimator_->getTerrainOrientationWorld().transpose() * base_velocity_;
 
   // Scale with inverse velocity factor to adapt to different gait settings
   com_velocity_ref_ = 1.0 / foothold_planner_->getVelocityFactor() * com_velocity_ref_;
 
   // Get current COM state
   robot_model_->getCOM(com_position_);
   //robot_model_->getCOMVelocity(com_velocity_);
 
   // Compute DCM parameters
   constexpr double min_height = 0.05;
   const double z = std::max(com_position_.z(), min_height);
   const double tau = std::sqrt(GRAVITY / z);
 
   // Get current capture point (ICP)
   dcm_target_ = foothold_planner_->getPushRecovery()->getCapturePoint();
 
   // Compute nominal DCM based on desired motion
   dcm_nominal_ = com_position_.head<2>() + com_velocity_ref_.head<2>() / tau;
 
   // Blend capture point and nominal target
   double alpha = 0.5;
   dcm_target_ = (1.0 - alpha) * dcm_target_ + alpha * dcm_nominal_;
 
   // Compute velocity reference from DCM target
   Eigen::Vector2d com_vel_ref_xy = tau * (dcm_target_ - com_position_.head<2>());
   com_velocity_ref_.head<2>() = com_vel_ref_xy;
 }
 
 // Compute the COM position reference
 void ComPlanner::computeComPositionReference(const double &dt)
 {
   // Anchor to support polygon center during stance phase
   if (foothold_planner_->areAllFeetInStance())
     computeSupportPolygonCenter();
 
   // Initialize reference at support center
   com_position_ref_ << support_polygon_center_(0), support_polygon_center_(1), foothold_planner_->getBaseHeight();
 
   // Integrate reference velocity over time for forward motion
   com_position_ref_.head<2>() += com_velocity_ref_.head<2>() * dt;
 }
 
 // Main update function called each control cycle
 void ComPlanner::update(const double &dt)
 {
   // Update filter parameters
   com_position_ref_filter_.setTimeStep(dt);
   com_velocity_ref_filter_.setTimeStep(dt);
   com_position_ref_filter_.setOmega(2.0 * M_PI * 1.0 / foothold_planner_->getCycleTime());
   com_velocity_ref_filter_.setOmega(2.0 * M_PI * 1.0 / foothold_planner_->getCycleTime());
 
   // Compute reference trajectories
   computeComVelocityReference(dt);
   computeComPositionReference(dt);
 
   // Filter output for smoothness
   com_position_ref_ = com_position_ref_filter_.process(com_position_ref_);
   com_velocity_ref_ = com_velocity_ref_filter_.process(com_velocity_ref_);
 }
 
 // Getters
 const Eigen::Vector3d &ComPlanner::getComVelocity() const {
   return com_velocity_ref_;
 }
 
 const Eigen::Vector3d &ComPlanner::getComPosition() const {
   return com_position_ref_;
 }
 
 // Reset functions
 void ComPlanner::reset() {
   resetPosition();
   resetVelocity();
 }
 
 void ComPlanner::resetPosition() {
   robot_model_->getCOM(com_position_ref_);
   com_position_ref_filter_.reset(com_position_ref_);
 }
 
 void ComPlanner::resetVelocity() {
   com_velocity_ref_.setZero();
   com_velocity_ref_filter_.reset(com_velocity_ref_);
 }
 
 } // namespace wolf_controller
 