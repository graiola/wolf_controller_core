/**
 * @file terrain_estimator.cpp
 * @author Gennaro Raiola, Michele Focchi
 * @date 12 June, 2019
 * @brief Terrain estimator based on foot position fitting (real-time safe)
 */

 #include <wolf_controller_core/terrain_estimator.h>
 #include <wolf_controller_core/common.h>
 #include <wolf_controller_utils/filters.h>
 #include <wolf_controller_utils/geometry.h>
 
 using namespace wolf_controller_utils;
 using namespace wolf_wbid;
 
 namespace wolf_controller {
 
 // Constants
 constexpr int N_LEGS_MAX = 4;
 
 TerrainEstimator::TerrainEstimator(StateEstimator::Ptr state_estimator,
                                    QuadrupedRobot::Ptr robot_model)
   : use_plane_projection_(true)
 {
   assert(state_estimator);
   state_estimator_ = state_estimator;
 
   assert(robot_model);
   robot_model_ = robot_model;
 
   // Fixed-size initialization
   A_fixed_.setZero();
   b_fixed_.setZero();
   Ai_fixed_.setZero();
   tmp_matrix3d_.setZero();
   contact_positions_.setZero();
 
   reset();
 }
 
 bool TerrainEstimator::computeTerrainEstimation(const double& dt)
 {
   posture_adjustment_dot_world_.setZero();
 
   auto foot_names = robot_model_->getFootNames();
   int contacts = 0;
   int contact_idx = 0;
 
   for (unsigned int i = 0; i < foot_names.size(); ++i)
   {
     if (state_estimator_->getContact(foot_names[i]))
     {
       contact_positions_.col(contact_idx) = robot_model_->getFeetPositionInWorld().at(foot_names[i]);
       contact_idx++;
     }
   }
 
   contacts = contact_idx;
 
   if (contacts >= 3)
   {
     // 2 - Fill A_fixed_ and b_fixed_ using rows
     double avg_x = 0.0, avg_y = 0.0, avg_z = 0.0;
 
     for (int i = 0; i < contacts; ++i)
     {
       const auto& pos = contact_positions_.col(i);
       A_fixed_.row(i) << pos(0), pos(1), 1.0;
       b_fixed_(i) = -pos(2);
 
       avg_x += pos(0);
       avg_y += pos(1);
       avg_z += pos(2);
     }
 
     avg_x /= contacts;
     avg_y /= contacts;
     avg_z /= contacts;
 
     // Compute pseudo-inverse without allocation
     tmp_matrix3d_.noalias() = A_fixed_.topRows(contacts).transpose() * A_fixed_.topRows(contacts);
 
     if (tmp_matrix3d_.determinant() != 0.0)
     {
       Ai_fixed_.noalias() = tmp_matrix3d_.inverse() * A_fixed_.topRows(contacts).transpose();
       terrain_normal_ = Ai_fixed_.leftCols(contacts) * b_fixed_.head(contacts);
     }
     else
     {
       PRINT_WARN_NAMED(CLASS_NAME,"Can not solve the problem!");
       return false;
     }
 
     // Normalize terrain normal
     terrain_normal_(2) = 1.0;
     terrain_normal_ = terrain_normal_ / terrain_normal_.norm();
 
     // Extract roll and pitch
     estimated_pitch_  = std::atan(terrain_normal_(0) / terrain_normal_(2));
     estimated_roll_   = std::atan(-terrain_normal_(1) * std::sin(estimated_pitch_) / terrain_normal_(0));
 
     // Optional: plane projection
     if (use_plane_projection_)
     {
       plane_coeffs_ = A_fixed_.topRows(contacts).colPivHouseholderQr().solve(b_fixed_.head(contacts));
       double z_on_plane = -(plane_coeffs_(0) * avg_x + plane_coeffs_(1) * avg_y + plane_coeffs_(2));
       world_X_terrain_ << avg_x, avg_y, z_on_plane;
     }
     else
     {
       world_X_terrain_ << avg_x, avg_y, avg_z;
     }
   }
 
   // 5 - Filter
   roll_  = secondOrderFilter(roll_, roll_filt_, estimated_roll_, 1.0);
   pitch_ = secondOrderFilter(pitch_, pitch_filt_, estimated_pitch_, 1.0);
 
   // 6 - Check limits
   if ((roll_ > min_roll_) && (roll_ < max_roll_) &&
       (pitch_ > min_pitch_) && (pitch_ < max_pitch_))
   {
     roll_out_world_  = roll_;
     pitch_out_world_ = pitch_;
   }
   else
   {
     PRINT_WARN_THROTTLE_NAMED(5.0, CLASS_NAME, "Angles beyond limits!");
     return false;
   }
 
   // 7 - Compute transforms
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
 
   // 8 - Set terrain normal
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
 