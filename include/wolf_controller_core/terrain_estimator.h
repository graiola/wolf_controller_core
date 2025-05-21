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
#include <memory>
#include <map>
#include <wolf_controller_core/state_estimator.h>
#include <wolf_controller_utils/tools.h>
#include <wolf_wbid/quadruped_robot.h>

namespace wolf_controller {

/**
 * @brief TerrainEstimator estimates terrain orientation (pitch/roll) using foot contacts.
 * 
 * It fits a plane to the contact foot positions and extracts the slope to update
 * body posture and height references. Uses second-order filtering for smooth output.
 */
class TerrainEstimator {
public:
    using Ptr = std::shared_ptr<TerrainEstimator>;
    using ConstPtr = std::shared_ptr<const TerrainEstimator>;

    /**
     * @brief Constructor
     * @param state_estimator Provides current contact states and base pose
     * @param robot_model Robot model for accessing kinematics and foot transforms
     */
    TerrainEstimator(StateEstimator::Ptr state_estimator,
                     wolf_wbid::QuadrupedRobot::Ptr robot_model);

    /**
     * @brief Compute terrain normal and pitch/roll based on foot contacts.
     * @param dt Time delta for filtering and differentiation
     * @return True if terrain estimation is valid
     */
    bool computeTerrainEstimation(const double& dt);

    /// Reset internal state and filters
    void reset();

    // --- Set limits on terrain slope for filtering and validity checks ---
    void setMinRoll(const double min);
    void setMinPitch(const double min);
    void setMaxRoll(const double max);
    void setMaxPitch(const double max);

    // --- Terrain pose in world frame ---
    const double& getRollInWorld() const;
    const double& getPitchInWorld() const;

    // --- Terrain pose in horizontal frame ---
    const double& getRollInHf() const;
    const double& getPitchInHf() const;

    // --- Output values for posture compensation ---
    const Eigen::Vector3d& getPostureAdjustmentDot() const;

    // --- Terrain parameters ---
    const Eigen::Vector3d& getTerrainNormal() const;
    const Eigen::Vector3d& getTerrainPositionWorld() const;
    const Eigen::Vector3d& getTerrainPositionHf() const;
    const Eigen::Matrix3d& getTerrainOrientationWorld() const;
    const Eigen::Matrix3d& getTerrainOrientationHf() const;
    const Eigen::Affine3d& getTerrainPoseWorld() const;
    const Eigen::Affine3d& getTerrainPoseHf() const;

    /// @return True if estimated terrain is nearly flat
    bool isOnFlatTerrain();

private:
    /// Builds A and b matrices from current foot positions for plane fitting
    void updateFootMatrix();

    // Least-squares matrices
    Eigen::MatrixXd A_;
    Eigen::VectorXd b_;

    // Terrain parameters
    Eigen::Vector3d terrain_normal_;
    Eigen::Vector3d hf_X_terrain_;
    Eigen::Matrix3d hf_R_terrain_;
    Eigen::Affine3d hf_T_terrain_;
    Eigen::Vector3d world_X_terrain_;
    Eigen::Matrix3d world_R_terrain_;
    Eigen::Affine3d world_T_terrain_;

    // Dependencies
    StateEstimator::Ptr state_estimator_;
    wolf_wbid::QuadrupedRobot::Ptr robot_model_;

    // Estimated and filtered terrain angles
    double roll_, pitch_;
    double estimated_roll_, estimated_pitch_;
    double max_roll_ = 0.4, max_pitch_ = 0.4;
    double min_roll_ = -0.4, min_pitch_ = -0.4;

    // Output values (after filtering)
    double roll_out_world_, pitch_out_world_;
    double roll_out_hf_, pitch_out_hf_;

    // Posture compensation variables
    double posture_adjustment_;
    double posture_adjustment_dot_;
    double posture_adjustment_prev_;
    Eigen::Vector3d posture_adjustment_dot_world_;
    Eigen::Vector3d posture_adjustment_dot_base_;

    // Second-order filter for roll and pitch [roll, pitch, unused]
    XBot::Utils::SecondOrderFilter<Eigen::Vector3d> rpy_filter_;
};

} // namespace wolf_controller

#endif // TERRAIN_ESTIMATOR_H
