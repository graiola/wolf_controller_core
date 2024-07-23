/**
WoLF: WoLF: Whole-body Locomotion Framework for quadruped robots (c) by Gennaro Raiola

WoLF is licensed under a license Creative Commons Attribution-NonCommercial-NoDerivatives 4.0 International License.

You should have received a copy of the license along with this
work. If not, see <http://creativecommons.org/licenses/by-nc-nd/4.0/>.
**/

#ifndef COMMON_H
#define COMMON_H

#include <string>
#include <vector>
#include <assert.h>
#include <rt_logger/rt_logger.h>

namespace wolf_controller
{
#define ODOM_FRAME "odom"
#define BASE_FOOTPRINT_FRAME "base_footprint"
#define BASE_STABILIZED_FRAME "base_stabilized"
#define ANGULAR_VELOCITIES_WRT_BASE // Comment it if the IMU's velocities are defined wrt world
#define GRAVITY 9.81 // Gravity value
//#define REACHING_MOTION
#define FLOATING_BASE_DOFS 6
#define N_LEGS 4 // Fixed number of legs supported
#define N_ARMS 1 // Fixed number of arms supported
#define THREADS_SLEEP_TIME_ms 4
#define THROTTLE_SEC 3.0
//#define COMPUTE_COST
//#define DEBUG
#define EPS 0.00001 //std::numeric_limits<double>::epsilon()
extern double _period;
extern std::string _robot_name;
extern std::string _robot_model_name;
extern std::string _tf_prefix;
extern std::string _rt_gui_group;
#define TOPIC( data ) (_robot_name+"/wolf_controller/"#data)
//#define OPEN_LOOP_TRAJECTORY

// NOTE: by default we use the same leg order as RBDL (alphabetic order)
extern std::vector<std::string> _dof_names;
extern std::vector<std::string> _cartesian_names;
extern std::vector<std::string> _xyz;
extern std::vector<std::string> _rpy;
extern std::vector<std::string> _joints_prefix;
extern std::vector<std::string> _legs_prefix;
enum _leg_id {LF=0,LH,RF,RH};

} // namespace

#endif
