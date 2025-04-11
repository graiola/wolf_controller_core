/**
WoLF: WoLF: Whole-body Locomotion Framework for quadruped robots (c) by Gennaro Raiola

WoLF is licensed under a license Creative Commons Attribution-NonCommercial-NoDerivatives 4.0 International License.

You should have received a copy of the license along with this
work. If not, see <http://creativecommons.org/licenses/by-nc-nd/4.0/>.
**/

#ifndef TERRAIN_ESTIMATOR_H
#define TERRAIN_ESTIMATOR_H

#include <Eigen/Core>
#include <Eigen/Dense>
#include <atomic>
#include <wolf_controller_core/state_estimator.h>
#include <wolf_controller_utils/tools.h>
#include <wolf_wbid/quadruped_robot.h>

namespace wolf_controller
{

class TerrainEstimator {

public:

    const std::string CLASS_NAME = "TerrainEstimator";

    /**
     * @brief Shared pointer to TerrainEstimator
     */
    typedef std::shared_ptr<TerrainEstimator> Ptr;

    /**
     * @brief Shared pointer to const TerrainEstimator
     */
    typedef std::shared_ptr<const TerrainEstimator> ConstPtr;

    TerrainEstimator(StateEstimator::Ptr state_estimator,
                     wolf_wbid::QuadrupedRobot::Ptr robot_model);

    bool computeTerrainEstimation(const double& dt);

    void reset();

    void setMinRoll(const double min);
    void setMinPitch(const double min);
    void setMaxRoll(const double max);
    void setMaxPitch(const double max);

    const double& getRollInWorld() const;
    const double& getPitchInWorld() const;
    const double& getRollInHf() const;
    const double& getPitchInHf() const;

    const Eigen::Vector3d& getPostureAdjustmentDot() const;

    const Eigen::Vector3d& getTerrainNormal() const;

    const Eigen::Vector3d& getTerrainPositionWorld() const;
    const Eigen::Vector3d& getTerrainPositionHf()  const;

    const Eigen::Matrix3d& getTerrainOrientationWorld() const;
    const Eigen::Matrix3d& getTerrainOrientationHf()  const;

    const Eigen::Affine3d& getTerrainPoseWorld() const;
    const Eigen::Affine3d& getTerrainPoseHf()  const;

    bool isOnFlatTerrain();

private:

    static constexpr int N_LEGS_MAX = 4;

    // Fixed-size members
    Eigen::Matrix<double, N_LEGS_MAX, 3> A_fixed_;
    Eigen::Matrix<double, 3, N_LEGS_MAX> Ai_fixed_;
    Eigen::Matrix<double, N_LEGS_MAX, 1> b_fixed_;
    Eigen::Matrix<double, 3, N_LEGS_MAX> contact_positions_;

    Eigen::Matrix3d tmp_matrix3d_;
    Eigen::Vector3d tmp_vector3d_;

    Eigen::Vector3d terrain_normal_;
    Eigen::Vector3d hf_X_terrain_;
    Eigen::Matrix3d hf_R_terrain_;
    Eigen::Affine3d hf_T_terrain_;
    Eigen::Vector3d world_X_terrain_;
    Eigen::Matrix3d world_R_terrain_;
    Eigen::Affine3d world_T_terrain_;

    bool use_plane_projection_;

    StateEstimator::Ptr state_estimator_;
    wolf_wbid::QuadrupedRobot::Ptr robot_model_;

    double roll_;
    double pitch_;

    double estimated_roll_;
    double estimated_pitch_;

    double roll_filt_;
    double pitch_filt_;

    double max_roll_;
    double max_pitch_;

    double min_roll_;
    double min_pitch_;

    double roll_out_world_;
    double pitch_out_world_;
    double roll_out_hf_;
    double pitch_out_hf_;

    double posture_adjustment_;
    double posture_adjustment_dot_;
    double posture_adjustment_prev_;
    Eigen::Vector3d posture_adjustment_dot_world_;
    Eigen::Vector3d posture_adjustment_dot_base_;
    Eigen::Vector3d plane_coeffs_;
};

} // namespace wolf_controller

#endif // TERRAIN_ESTIMATOR_H
