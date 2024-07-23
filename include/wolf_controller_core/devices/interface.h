/**
WoLF: WoLF: Whole-body Locomotion Framework for quadruped robots (c) by Gennaro Raiola

WoLF is licensed under a license Creative Commons Attribution-NonCommercial-NoDerivatives 4.0 International License.

You should have received a copy of the license along with this
work. If not, see <http://creativecommons.org/licenses/by-nc-nd/4.0/>.
**/

#ifndef DEVICES_INTERFACE_H
#define DEVICES_INTERFACE_H

// WoLF
#include <wolf_controller/common.h>

// WoLF
#include <wolf_controller_utils/mux.h>

// STD
#include <memory>
#include <algorithm>

class DeviceHandlerInterface : public wolf_controller_utils::Input
{

public:

    /**
     * @brief Shared pointer to DeviceHandlerInterface
     */
    typedef std::shared_ptr<DeviceHandlerInterface> Ptr;

    /**
     * @brief Shared pointer to const DeviceHandlerInterface
     */
    typedef std::shared_ptr<const DeviceHandlerInterface> ConstPtr;

    DeviceHandlerInterface()
    {
      reset();
    }

    void reset()
    {
      base_velocity_x_scale_     = 0.0;
      base_velocity_y_scale_     = 0.0;
      base_velocity_z_scale_     = 0.0;
      base_velocity_yaw_scale_   = 0.0;
      base_velocity_pitch_scale_ = 0.0;
      base_velocity_roll_scale_  = 0.0;
      base_velocity_x_cmd_       = 0.0;
      base_velocity_y_cmd_       = 0.0;
      base_velocity_z_cmd_       = 0.0;
      base_velocity_yaw_cmd_     = 0.0;
      base_velocity_pitch_cmd_   = 0.0;
      base_velocity_roll_cmd_    = 0.0;
      start_swing_               = false;
    }

    virtual ~DeviceHandlerInterface() {}

    virtual void updateInput()
    {
        update();
    }

    virtual void resetInput()
    {
        reset();
    }

protected:

    virtual void update() = 0;

    double base_velocity_x_scale_;
    double base_velocity_y_scale_;
    double base_velocity_z_scale_;
    double base_velocity_yaw_scale_;
    double base_velocity_pitch_scale_;
    double base_velocity_roll_scale_;
    double base_velocity_x_cmd_;
    double base_velocity_y_cmd_;
    double base_velocity_z_cmd_;
    double base_velocity_yaw_cmd_;
    double base_velocity_pitch_cmd_;
    double base_velocity_roll_cmd_;
    bool   start_swing_;
};

class DevicesHandler
{
public:

    enum priority_t {HIGH=0,MEDIUM=1,LOW=2};

    DevicesHandler() {}

    void addDevice(priority_t priority, DeviceHandlerInterface::Ptr device)
    {
        devices_.push_back(device);
        mux_.addInput(priority,device.get());
    }

    void writeToOutput(const double& period)
    {
        mux_.selectInput(period);
    }

private:
    std::vector<DeviceHandlerInterface::Ptr> devices_;
    wolf_controller_utils::Mux mux_;
};



#endif
