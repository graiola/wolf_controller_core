/**
 * @file terrain_estimator.cpp
 * @author Gennaro Raiola, Michele Focchi
 * @date 12 June, 2019
 * @brief Terrain estimator based on foot position fitting
 */

 #include <wolf_controller_core/terrain_estimator.h>
 #include <wolf_controller_core/common.h>
 #include <wolf_controller_utils/filters.h>
 #include <wolf_controller_utils/geometry.h>
 
 using namespace wolf_controller_utils;
 using namespace wolf_wbid;
 
 namespace wolf_controller {
 
 TerrainEstimator::TerrainEstimator(StateEstimator::Ptr state_estimator,
                                    QuadrupedRobot::Ptr robot_model)
   :use_plane_projection_(true)
 {
   assert(state_estimator);
   state_estimator_ = state_estimator;
 
   assert(robot_model);
   robot_model_ = robot_model;
 
   A_.resize(N_LEGS,3);
   Ai_.resize(3,N_LEGS);
   b_.resize(N_LEGS);
 
   reset();
 }
 
 bool TerrainEstimator::computeTerrainEstimation(const double& dt)
 {
   posture_adjustment_dot_world_.setZero();
 
   auto foot_names = robot_model_->getFootNames();
   int contacts = 0;
   for(unsigned int i = 0; i < foot_names.size(); i++)
     contacts += state_estimator_->getContact(foot_names[i]) ? 1 : 0;
 
   if(contacts >= 3)
   {
     // 2 - Update A and b with only contacting feet
     update();
 
     tmp_matrix3d_.noalias() = A_.transpose() * A_;
 
     if(tmp_matrix3d_.determinant() != 0.0)
     {
       Ai_.noalias() = tmp_matrix3d_.inverse() * A_.transpose();
       terrain_normal_ = Ai_ * b_;
     }
     else
     {
       PRINT_WARN_NAMED(CLASS_NAME,"Can not solve the problem!");
       return false;
     }
 
     // 3 - Normalize the terrain normal
     terrain_normal_(2) = 1.0;
     terrain_normal_ = terrain_normal_ / terrain_normal_.norm();
 
     // 4 - Extract the estimated values for roll and pitch
     estimated_pitch_  = std::atan(terrain_normal_(0) / terrain_normal_(2));
     estimated_roll_   = std::atan(-terrain_normal_(1) * std::sin(estimated_pitch_) / terrain_normal_(0));
   }
 
   // 5 - Filter
   roll_  = secondOrderFilter(roll_, roll_filt_, estimated_roll_, 1.0);
   pitch_ = secondOrderFilter(pitch_, pitch_filt_, estimated_pitch_, 1.0);
 
   // 6 - Check output limits
   if((roll_ > min_roll_) && (roll_ < max_roll_) &&
      (pitch_ > min_pitch_) && (pitch_ < max_pitch_))
   {
     // 7 - Update the roll and pitch output values
     roll_out_world_  = roll_;
     pitch_out_world_ = pitch_;
   }
   else
   {
     PRINT_WARN_THROTTLE_NAMED(5.0, CLASS_NAME, "Angles beyond limits!");
     return false;
   }
 
   // 7 - Update the resulting Transformation
   tmp_vector3d_ << roll_out_world_, pitch_out_world_, 0.0;
   rpyToRotTranspose(tmp_vector3d_, world_R_terrain_);
   world_T_terrain_.translation() = world_X_terrain_;
   world_T_terrain_.linear() = world_R_terrain_;
 
   hf_T_terrain_ = robot_model_->getHfRotationInWorld().transpose() * world_T_terrain_;
   hf_X_terrain_ = hf_T_terrain_.translation();
   hf_R_terrain_ = hf_T_terrain_.linear();
   rotTransposeToRpy(hf_R_terrain_, tmp_vector3d_);
   roll_out_hf_ = tmp_vector3d_(0);
   pitch_out_hf_ = tmp_vector3d_(1);
 
   // 8 - Update the state estimator
   state_estimator_->setTerrainNormal(terrain_normal_);
 
   // 9 - Posture adjustment
   double terrain_h_base = state_estimator_->getFloatingBasePosition()(2);
   posture_adjustment_ = terrain_h_base * std::tan(pitch_out_hf_);
   posture_adjustment_dot_ = (posture_adjustment_ - posture_adjustment_prev_) / dt;
   posture_adjustment_prev_ = posture_adjustment_;
   posture_adjustment_dot_base_ << posture_adjustment_dot_, 0.0, 0.0;
   posture_adjustment_dot_world_ = robot_model_->getBaseRotationInWorld() * posture_adjustment_dot_base_;
 
   return true;
 }
 
 void TerrainEstimator::update()
 {
   auto foot_names = robot_model_->getFootNames();
   auto foot_positions = robot_model_->getFeetPositionInWorld();
 
   std::vector<Eigen::Vector3d> contact_positions;
   for (const auto& name : foot_names)
   {
     if (state_estimator_->getContact(name))
       contact_positions.push_back(foot_positions[name]);
   }
 
   int n_contacts = contact_positions.size();
   if (n_contacts < 3)
   {
     PRINT_WARN_NAMED(CLASS_NAME, "Not enough contacts to compute terrain.");
     return;
   }
 
   A_.resize(n_contacts, 3);
   b_.resize(n_contacts);
 
   double avg_x = 0.0, avg_y = 0.0, avg_z = 0.0;
 
   for (int i = 0; i < n_contacts; ++i)
   {
     const auto& pos = contact_positions[i];
     A_(i, 0) = pos(0);
     A_(i, 1) = pos(1);
     A_(i, 2) = 1.0;
     b_(i)    = -pos(2);
 
     avg_x += pos(0);
     avg_y += pos(1);
     avg_z += pos(2);
   }
 
   avg_x /= n_contacts;
   avg_y /= n_contacts;
   avg_z /= n_contacts;
 
   if (use_plane_projection_)
   {
     Eigen::Vector3d plane_coeffs = A_.colPivHouseholderQr().solve(b_);
     double z_on_plane = -(plane_coeffs(0) * avg_x + plane_coeffs(1) * avg_y + plane_coeffs(2));
     world_X_terrain_ << avg_x, avg_y, z_on_plane;
   }
   else
   {
     world_X_terrain_ << avg_x, avg_y, avg_z;
   }
 }
 
 void TerrainEstimator::reset()
 {
 
   roll_ = roll_filt_ = roll_out_world_ = roll_out_hf_ = estimated_roll_ = 0.0;
   pitch_ = pitch_filt_ = pitch_out_world_ = pitch_out_hf_ = estimated_pitch_ = 0.0;
 
   terrain_normal_ << 0.0, 0.0, 1.0;
   world_X_terrain_ = hf_X_terrain_ = Eigen::Vector3d::Zero();
   world_T_terrain_ = hf_T_terrain_ = Eigen::Affine3d::Identity();
   world_R_terrain_ = hf_R_terrain_ = Eigen::Matrix3d::Identity();
 
   posture_adjustment_ = posture_adjustment_prev_ = posture_adjustment_dot_ = 0.0;
   posture_adjustment_dot_world_ = Eigen::Vector3d::Zero();
 }
 
 void TerrainEstimator::setMinRoll(const double min) { min_roll_ = min; }
 void TerrainEstimator::setMinPitch(const double min) { min_pitch_ = min; }
 void TerrainEstimator::setMaxRoll(const double max) { max_roll_ = max; }
 void TerrainEstimator::setMaxPitch(const double max) { max_pitch_ = max; }
 
 const double& TerrainEstimator::getRollInWorld() const { return roll_out_world_; }
 const double& TerrainEstimator::getPitchInWorld() const { return pitch_out_world_; }
 const double& TerrainEstimator::getRollInHf() const { return roll_out_hf_; }
 const double& TerrainEstimator::getPitchInHf() const { return pitch_out_hf_; }
 
 const Eigen::Vector3d &TerrainEstimator::getPostureAdjustmentDot() const
 { return posture_adjustment_dot_world_; }
 
 const Eigen::Vector3d &TerrainEstimator::getTerrainNormal() const
 { return terrain_normal_; }
 
 const Eigen::Vector3d &TerrainEstimator::getTerrainPositionWorld() const
 { return world_X_terrain_; }
 
 const Eigen::Vector3d &TerrainEstimator::getTerrainPositionHf() const
 { return hf_X_terrain_; }
 
 const Eigen::Matrix3d &TerrainEstimator::getTerrainOrientationWorld() const
 { return world_R_terrain_; }
 
 const Eigen::Matrix3d &TerrainEstimator::getTerrainOrientationHf() const
 { return hf_R_terrain_; }
 
 const Eigen::Affine3d &TerrainEstimator::getTerrainPoseWorld() const
 { return world_T_terrain_; }
 
 const Eigen::Affine3d &TerrainEstimator::getTerrainPoseHf() const
 { return hf_T_terrain_; }
 
 bool TerrainEstimator::isOnFlatTerrain()
 {
   return (std::abs(roll_out_world_) <= 0.01 && std::abs(pitch_out_world_) <= 0.01);
 }
 
 } // namespace wolf_controller
 