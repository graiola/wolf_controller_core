/**
 * @file com_planner.cpp
 * @brief DCM-based COM reference planner for quadruped locomotion
 */

 #include <wolf_controller_core/wpg/com_planner.h>

 using namespace wolf_wbid;
 
 namespace wolf_controller {
 
 ComPlanner::ComPlanner(QuadrupedRobot::Ptr robot_model,
                        GaitGenerator::Ptr gait_generator,
                        TerrainEstimator::Ptr terrain_estimator)
 {
   robot_model_ = robot_model;
   gait_generator_ = gait_generator;
   terrain_estimator_ = terrain_estimator;
 
   com_velocity_ref_.setZero();
   com_position_ref_.setZero();
 
   support_polygon_edges_.resize(N_LEGS);
 
   // Set filter time steps and frequencies
   com_position_ref_filter_.setTimeStep(_period);
   com_velocity_ref_filter_.setTimeStep(_period);
   com_position_ref_filter_.setOmega(2.0 * M_PI * 1.0 / gait_generator_->getAvgCycleTime());
   com_velocity_ref_filter_.setOmega(2.0 * M_PI * 1.0 / gait_generator_->getAvgCycleTime());
 
   // Initialize COM reference position
   computeComPositionReference(_period,0.0);
 }
 
 // Compute center of support polygon by averaging all feet positions
 void ComPlanner::computeSupportPolygonCenter()
 {
   auto foot_positions = robot_model_->getFeetPositionInWorld();
   auto foot_names = gait_generator_->getFootNames();
   support_polygon_center_.setZero();
   for (unsigned int i = 0; i < foot_names.size(); i++) {
     support_polygon_center_ += foot_positions[foot_names[i]];
     support_polygon_edges_[i] = foot_positions[foot_names[i]];
   }
   support_polygon_center_ /= N_LEGS;
 }
 
 // Compute the COM velocity reference using DCM planning
 void ComPlanner::computeComVelocityReference(const double &dt, const Eigen::Vector3d& base_velocity)
 {
   // Get commanded base velocity and project it into the terrain frame
   base_velocity_ = base_velocity;
   com_velocity_ref_.setZero();
   com_velocity_ref_ = terrain_estimator_->getTerrainOrientationWorld().transpose() * base_velocity_;
 
   // Scale with inverse velocity factor to adapt to different gait settings
   com_velocity_ref_ = com_velocity_ref_ / gait_generator_->getVelocityFactor();
 
   // Get current COM state
   robot_model_->getCOM(com_position_);
   robot_model_->getCOMVelocity(com_velocity_);
 
   // Compute DCM parameters
   constexpr double min_height = 0.05;
   const double z = std::max(com_position_.z(), min_height);
   const double omega = std::sqrt(GRAVITY / z);  // Correct definition
   const double tau = 1.0 / omega;
 
   // Compute capture point (instantaneous DCM target)
   dcm_target_ = com_position_.head<2>() + com_velocity_.head<2>() / omega;
 
   // Compute nominal DCM based on desired velocity
   dcm_nominal_ = com_position_.head<2>() + com_velocity_ref_.head<2>() / omega;
 
   // Blend capture point and nominal target
   const double alpha = 0.5;
   dcm_target_ = (1.0 - alpha) * dcm_target_ + alpha * dcm_nominal_;
 
   // Compute velocity reference from DCM target
   com_velocity_ref_.head<2>() = omega * (dcm_target_ - com_position_.head<2>());
 }
 
 // Compute the COM position reference
 void ComPlanner::computeComPositionReference(const double &dt, const double& base_height)
 {
   // Anchor to support polygon center during stance phase
   if (gait_generator_->areAllFeetInStance())
     computeSupportPolygonCenter();
 
   // Initialize reference at support center
   com_position_ref_ << support_polygon_center_(0), support_polygon_center_(1), base_height;
 
   // Integrate reference velocity over time for forward motion
   com_position_ref_.head<2>() += com_velocity_ref_.head<2>() * dt;
 }

 const Eigen::Vector2d& ComPlanner::getCapturePoint() const
 {
   return dcm_target_;
 }
 
 // Main update function called each control cycle
 void ComPlanner::update(const double &dt, const Eigen::Vector3d& base_velocity, const double& base_height)
 {
   // Update filter parameters
   com_position_ref_filter_.setTimeStep(dt);
   com_velocity_ref_filter_.setTimeStep(dt);
   com_position_ref_filter_.setOmega(2.0 * M_PI * 1.0 / gait_generator_->getAvgCycleTime());
   com_velocity_ref_filter_.setOmega(2.0 * M_PI * 1.0 / gait_generator_->getAvgCycleTime());
 
   // Compute reference trajectories
   computeComVelocityReference(dt,base_velocity);
   computeComPositionReference(dt,base_height);
 
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
 