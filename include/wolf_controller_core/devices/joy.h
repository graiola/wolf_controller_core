/**
WoLF: WoLF: Whole-body Locomotion Framework for quadruped robots (c) by Gennaro Raiola

WoLF is licensed under a license Creative Commons Attribution-NonCommercial-NoDerivatives 4.0 International License.

You should have received a copy of the license along with this
work. If not, see <http://creativecommons.org/licenses/by-nc-nd/4.0/>.
**/

#ifndef DEVICES_JOY_H
#define DEVICES_JOY_H

#include <sensor_msgs/Joy.h>
#include <wolf_controller/devices/ros.h>
#include <wolf_controller/common.h>
#include <wolf_controller_utils/tools.h>

class JoyHandler : public DeviceHandlerRosInterface<sensor_msgs::Joy::ConstPtr>
{

public:

    typedef std::function<void ()> funct_t;

    /**
     * @brief Shared pointer to JoyHandler
     */
    typedef std::shared_ptr<JoyHandler> Ptr;

    /**
     * @brief Shared pointer to const JoyHandler
     */
    typedef std::shared_ptr<const JoyHandler> ConstPtr;

    struct FunctionTrigger
    {
        wolf_controller_utils::Trigger t_;
        funct_t f_;
    };

    JoyHandler(ros::NodeHandle& node, wolf_controller::Controller* controller_ptr, const std::string& topic = "joy")
        :DeviceHandlerRosInterface(node,controller_ptr,topic)
    {
        // Set handlers
        if(controller_ptr!=nullptr)
        {
            switch_posture_.f_ = boost::bind(&wolf_controller::Controller::switchPosture,controller_ptr);
            switch_control_mode_.f_ = boost::bind(&wolf_controller::Controller::switchControlMode,controller_ptr);
            emergency_stop_.f_ = boost::bind(&wolf_controller::Controller::emergencyStop,controller_ptr);
            reset_base_.f_ = boost::bind(&wolf_controller::Controller::resetBase,controller_ptr);
        }
        else
            throw std::runtime_error("Controller not initialized yet!");

        if(controller_ptr->getGaitGenerator()!=nullptr)
            switch_gait_.f_ = boost::bind(&wolf_controller::Controller::switchGait,controller_ptr);
        else
            throw std::runtime_error("GaitGenerator not initialized yet!");
    }

    virtual ~JoyHandler() {};

    virtual void cmdCallback(const sensor_msgs::Joy::ConstPtr& msg) = 0;

protected:

    void cmdCallback()
    {

        base_velocity_x_cmd_       = controller_ptr_->getBaseLinearVelocityCmdX();
        base_velocity_y_cmd_       = controller_ptr_->getBaseLinearVelocityCmdY();
        base_velocity_z_cmd_       = controller_ptr_->getBaseLinearVelocityCmdZ();
        base_velocity_roll_cmd_    = controller_ptr_->getBaseAngularVelocityCmdRoll();
        base_velocity_pitch_cmd_   = controller_ptr_->getBaseAngularVelocityCmdPitch();
        base_velocity_yaw_cmd_     = controller_ptr_->getBaseAngularVelocityCmdYaw();

        if(step_height_.getStatus() == wolf_controller_utils::AxisToTrigger::UP)
            controller_ptr_->getFootholdsPlanner()->increaseStepHeight();
        else if (step_height_.getStatus() == wolf_controller_utils::AxisToTrigger::DOWN)
            controller_ptr_->getFootholdsPlanner()->decreaseStepHeight();

        if( start_swing_                             ||
            std::abs(base_velocity_x_scale_)    >0.0 ||
            std::abs(base_velocity_y_scale_)    >0.0 ||
            std::abs(base_velocity_z_scale_)    >0.0 ||
            std::abs(base_velocity_roll_scale_) >0.0 ||
            std::abs(base_velocity_pitch_scale_)>0.0 ||
            std::abs(base_velocity_yaw_scale_)  >0.0 )
            activate();
    }

    FunctionTrigger switch_posture_;
    FunctionTrigger switch_gait_;
    FunctionTrigger switch_control_mode_;
    FunctionTrigger emergency_stop_;
    FunctionTrigger reset_base_;
    wolf_controller_utils::AxisToTrigger step_height_;
};

class Ps3JoyHandler : public JoyHandler
{
public:

   Ps3JoyHandler(ros::NodeHandle& node, wolf_controller::Controller* controller_ptr, const std::string& topic = "joy")
        :JoyHandler(node,controller_ptr,topic)
   {
   }

    virtual void cmdCallback(const sensor_msgs::Joy::ConstPtr& msg)
    {

        if(msg.get() && !msg->axes.empty() && !msg->buttons.empty())
        {

            base_velocity_y_scale_     = static_cast<double>(msg->axes[0]);
            base_velocity_x_scale_     = static_cast<double>(msg->axes[1]);
            base_velocity_z_scale_     = (static_cast<double>(msg->buttons[5])-static_cast<double>(msg->buttons[7])); //R1 and R2

            base_velocity_yaw_scale_   = static_cast<double>(msg->axes[2]);
            base_velocity_pitch_scale_ = static_cast<double>(msg->axes[3]);
            base_velocity_roll_scale_  = -static_cast<double>(msg->axes[4]);

            start_swing_               = static_cast<bool>(msg->buttons[4]); // L1 button

            step_height_.update(static_cast<double>(msg->axes[5]));

            if(switch_posture_.f_ && switch_posture_.t_.update(static_cast<bool>(msg->buttons[9]))) // start
                switch_posture_.f_();

            if(switch_gait_.f_ && switch_gait_.t_.update(static_cast<bool>(msg->buttons[8]))) // select
                switch_gait_.f_();

            if(switch_control_mode_.f_ && switch_control_mode_.t_.update(static_cast<bool>(msg->buttons[0]))) // 1
                switch_control_mode_.f_();

            if(emergency_stop_.f_ && emergency_stop_.t_.update(static_cast<bool>(msg->buttons[1]))) // 2
                emergency_stop_.f_();

            if(reset_base_.f_ && reset_base_.t_.update(static_cast<bool>(msg->buttons[6]))) // L2
                reset_base_.f_();
        }

        JoyHandler::cmdCallback();
    }
};

class XboxJoyHandler : public JoyHandler
{
public:

    XboxJoyHandler(ros::NodeHandle& node, wolf_controller::Controller* controller_ptr, const std::string& topic = "joy")
         :JoyHandler(node,controller_ptr,topic)
    {
    }

    virtual void cmdCallback(const sensor_msgs::Joy::ConstPtr& msg)
    {

        if(msg.get() && !msg->axes.empty() && !msg->buttons.empty())
        {

            base_velocity_y_scale_     = static_cast<double>(msg->axes[0]);
            base_velocity_x_scale_     = static_cast<double>(msg->axes[1]);
            base_velocity_z_scale_     = static_cast<double>(msg->axes[7]);

            base_velocity_yaw_scale_   = static_cast<double>(msg->axes[3]);
            base_velocity_pitch_scale_ = static_cast<double>(msg->axes[4]);
            base_velocity_roll_scale_  = -static_cast<double>(msg->axes[6]);

            start_swing_               = static_cast<bool>(msg->buttons[4]); // L1 button

            step_height_.update(static_cast<double>(msg->axes[5]));

            if(switch_posture_.f_ && switch_posture_.t_.update(static_cast<bool>(msg->buttons[6]))) // start
                switch_posture_.f_();

            if(switch_gait_.f_ && switch_gait_.t_.update(static_cast<bool>(msg->buttons[7]))) // select
                switch_gait_.f_();

            if(switch_control_mode_.f_ && switch_control_mode_.t_.update(static_cast<bool>(msg->buttons[0]))) // 1
                switch_control_mode_.f_();

            if(emergency_stop_.f_ && emergency_stop_.t_.update(static_cast<bool>(msg->buttons[1]))) // 2
                emergency_stop_.f_();

            if(reset_base_.f_ && reset_base_.t_.update(static_cast<bool>(msg->buttons[5]))) // R1
                reset_base_.f_();
        }

        JoyHandler::cmdCallback();
    }
};

class SpaceJoyHandler : public JoyHandler
{
public:

    SpaceJoyHandler(ros::NodeHandle& node, wolf_controller::Controller* controller_ptr, const std::string& topic = "/spacenav/joy")
         :JoyHandler(node,controller_ptr,topic)
    {
        th = 0.5;
    }

    virtual void cmdCallback(const sensor_msgs::Joy::ConstPtr& msg)
    {

        if(msg.get() && !msg->axes.empty() && !msg->buttons.empty())
        {

            base_velocity_x_scale_     = wolf_controller_utils::sgn(msg->axes[0]) * (std::abs(static_cast<double>(msg->axes[0])) > 0.5 ? 1.0 : 0.0);
            base_velocity_y_scale_     = wolf_controller_utils::sgn(msg->axes[1]) * (std::abs(static_cast<double>(msg->axes[1])) > 0.5 ? 1.0 : 0.0);
            base_velocity_z_scale_     = wolf_controller_utils::sgn(msg->axes[2]) * (std::abs(static_cast<double>(msg->axes[2])) > 0.5 ? 1.0 : 0.0);

            base_velocity_roll_scale_  = wolf_controller_utils::sgn(msg->axes[3]) * (std::abs(static_cast<double>(msg->axes[3])) > 0.5 ? 1.0 : 0.0);
            base_velocity_pitch_scale_ = wolf_controller_utils::sgn(msg->axes[4]) * (std::abs(static_cast<double>(msg->axes[4])) > 0.5 ? 1.0 : 0.0);
            base_velocity_yaw_scale_   = wolf_controller_utils::sgn(msg->axes[5]) * (std::abs(static_cast<double>(msg->axes[5])) > 0.5 ? 1.0 : 0.0);

            if(std::abs(base_velocity_x_scale_) > 0.0 || std::abs(base_velocity_y_scale_) > 0.0 || std::abs(base_velocity_yaw_scale_) > 0.0)
                start_swing_ = true;
            else
                start_swing_ = false;

            if(switch_posture_.f_ && switch_posture_.t_.update(static_cast<bool>(msg->buttons[0]))) // Left button
                switch_posture_.f_();

            if(reset_base_.f_ && reset_base_.t_.update(static_cast<bool>(msg->buttons[1]))) // Right button
                reset_base_.f_();
        }

        JoyHandler::cmdCallback();
    }
private:
    double th;
};

#endif
