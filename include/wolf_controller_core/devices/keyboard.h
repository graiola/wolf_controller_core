/**
WoLF: WoLF: Whole-body Locomotion Framework for quadruped robots (c) by Gennaro Raiola

WoLF is licensed under a license Creative Commons Attribution-NonCommercial-NoDerivatives 4.0 International License.

You should have received a copy of the license along with this
work. If not, see <http://creativecommons.org/licenses/by-nc-nd/4.0/>.
**/

#ifndef DEVICES_KEYBOARD_H
#define DEVICES_KEYBOARD_H

#include <geometry_msgs/Twist.h>
#include <wolf_controller/devices/ros.h>

class KeyboardHandler : public DeviceHandlerRosInterface<geometry_msgs::Twist>
{

public:

    /**
     * @brief Shared pointer to KeyboardHandler
     */
    typedef std::shared_ptr<KeyboardHandler> Ptr;

    /**
     * @brief Shared pointer to const KeyboardHandler
     */
    typedef std::shared_ptr<const KeyboardHandler> ConstPtr;

    KeyboardHandler(ros::NodeHandle& node, wolf_controller::Controller* controller_ptr, const std::string& topic = "keyboard")
        :DeviceHandlerRosInterface(node,controller_ptr,topic)
    {

    }

    void cmdCallback(const geometry_msgs::Twist& msg)
    {
        base_velocity_x_scale_     = static_cast<double>(msg.linear.x);
        base_velocity_y_scale_     = static_cast<double>(msg.linear.y);
        base_velocity_z_scale_     = static_cast<double>(msg.linear.z);
        base_velocity_roll_scale_  = static_cast<double>(msg.angular.x);
        base_velocity_pitch_scale_ = static_cast<double>(msg.angular.y);
        base_velocity_yaw_scale_   = static_cast<double>(msg.angular.z);

        base_velocity_x_cmd_       = controller_ptr_->getBaseLinearVelocityCmdX();
        base_velocity_y_cmd_       = controller_ptr_->getBaseLinearVelocityCmdY();
        base_velocity_z_cmd_       = controller_ptr_->getBaseLinearVelocityCmdZ();
        base_velocity_roll_cmd_    = controller_ptr_->getBaseAngularVelocityCmdRoll();
        base_velocity_pitch_cmd_   = controller_ptr_->getBaseAngularVelocityCmdPitch();
        base_velocity_yaw_cmd_     = controller_ptr_->getBaseAngularVelocityCmdYaw();

        if(std::abs(base_velocity_x_scale_) > 0.0 || std::abs(base_velocity_y_scale_) > 0.0 || std::abs(base_velocity_yaw_scale_) > 0.0)
            start_swing_ = true;
        else
            start_swing_ = false;

        if( std::abs(base_velocity_x_scale_)    >0.0 ||
            std::abs(base_velocity_y_scale_)    >0.0 ||
            std::abs(base_velocity_z_scale_)    >0.0 ||
            std::abs(base_velocity_roll_scale_) >0.0 ||
            std::abs(base_velocity_pitch_scale_)>0.0 ||
            std::abs(base_velocity_yaw_scale_)  >0.0 )
            activate();
    }
};


#endif
