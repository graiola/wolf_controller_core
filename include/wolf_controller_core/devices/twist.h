/**
WoLF: WoLF: Whole-body Locomotion Framework for quadruped robots (c) by Gennaro Raiola

WoLF is licensed under a license Creative Commons Attribution-NonCommercial-NoDerivatives 4.0 International License.

You should have received a copy of the license along with this
work. If not, see <http://creativecommons.org/licenses/by-nc-nd/4.0/>.
**/

#ifndef DEVICES_TWIST_H
#define DEVICES_TWIST_H

#include <geometry_msgs/Twist.h>
#include <wolf_controller/devices/ros.h>

class TwistHandler : public DeviceHandlerRosInterface<geometry_msgs::Twist>
{

public:

    /**
     * @brief Shared pointer to TwistHandler
     */
    typedef std::shared_ptr<TwistHandler> Ptr;

    /**
     * @brief Shared pointer to const TwistHandler
     */
    typedef std::shared_ptr<const TwistHandler> ConstPtr;

    TwistHandler(ros::NodeHandle& node, wolf_controller::Controller* controller_ptr, const std::string& topic = "twist")
        :DeviceHandlerRosInterface(node,controller_ptr,topic)
    {

    }

    void cmdCallback(const geometry_msgs::Twist& msg)
    {
        start_swing_             = false;
        base_velocity_x_cmd_     = static_cast<double>(msg.linear.x);
        base_velocity_y_cmd_     = static_cast<double>(msg.linear.y);
        base_velocity_z_cmd_     = static_cast<double>(msg.linear.z);
        base_velocity_roll_cmd_  = static_cast<double>(msg.angular.x);
        base_velocity_pitch_cmd_ = static_cast<double>(msg.angular.y);
        base_velocity_yaw_cmd_   = static_cast<double>(msg.angular.z);

        if(std::abs(base_velocity_x_cmd_)     > 0.0) { base_velocity_x_scale_     = 1.0; start_swing_ = true; } else { base_velocity_x_scale_     = 0.0; }
        if(std::abs(base_velocity_y_cmd_)     > 0.0) { base_velocity_y_scale_     = 1.0; start_swing_ = true; } else { base_velocity_y_scale_     = 0.0; }
        if(std::abs(base_velocity_z_cmd_)     > 0.0) { base_velocity_z_scale_     = 1.0;                      } else { base_velocity_z_scale_     = 0.0; }
        if(std::abs(base_velocity_roll_cmd_)  > 0.0) { base_velocity_roll_scale_  = 1.0;                      } else { base_velocity_roll_scale_  = 0.0; }
        if(std::abs(base_velocity_pitch_cmd_) > 0.0) { base_velocity_pitch_scale_ = 1.0;                      } else { base_velocity_pitch_scale_ = 0.0; }
        if(std::abs(base_velocity_yaw_cmd_)   > 0.0) { base_velocity_yaw_scale_   = 1.0; start_swing_ = true; } else { base_velocity_yaw_scale_   = 0.0; }

        if( std::abs(base_velocity_x_cmd_)    >0.0 ||
            std::abs(base_velocity_y_cmd_)    >0.0 ||
            std::abs(base_velocity_z_cmd_)    >0.0 ||
            std::abs(base_velocity_roll_cmd_) >0.0 ||
            std::abs(base_velocity_pitch_cmd_)>0.0 ||
            std::abs(base_velocity_yaw_cmd_)  >0.0 )
            activate();
    }
};


#endif
