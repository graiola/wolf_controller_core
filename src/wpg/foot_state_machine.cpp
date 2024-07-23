/**
 * @file foot_state_machine.cpp
 * @author Gennaro Raiola
 * @date 12 June, 2019
 * @brief This file contains the state machine for the foot state
 */

#include <wolf_controller_core/wpg/foot_state_machine.h>
#include <wolf_controller_utils/common.h>

using namespace wolf_controller;


FootStateMachine::FootStateMachine()
{
    // Inputs
    duty_factor_ = 0.8; // It is defined as T_stance / T_cycle
    swing_frequency_ = 0.3;
    reset();
}

void FootStateMachine::reset()
{
    init();
    cycle_ended_ = true; // We set true so that the swing can be triggered
    updatePeriods();
    state_ = prev_state_ = states::STANCE;
}

bool FootStateMachine::isSwing()
{
    if(state_ == states::SWING)
        return true;
    else
        return false;
}

bool FootStateMachine::isStance()
{
    if(state_ == states::STANCE)
        return true;
    else
        return false;
}

bool FootStateMachine::isStateChanged()
{
    if(prev_state_ != state_)
        return true;
    else
        return false;
}

bool FootStateMachine::isTouchDown()
{
    if(prev_state_ == states::SWING && state_ == states::STANCE)
        return true;
    else
        return false;
}

bool FootStateMachine::isLiftOff()
{
    if(prev_state_ == states::STANCE && state_ == states::SWING)
        return true;
    else
        return false;
}

bool FootStateMachine::isCycleEnded()
{
    if(cycle_ended_)
        return true;
    else
        return false;
}

void FootStateMachine::triggerSwing()
{
    trigger_swing_ = true;
}

void FootStateMachine::setDutyFactor(double duty_factor)
{
    if(duty_factor >= 0 && duty_factor <1)
        duty_factor_ = duty_factor;
    else
        PRINT_WARN_NAMED(CLASS_NAME,"Duty factor has to be defined between 0.0 and 1.0!");
}

void FootStateMachine::setSwingFrequency(double swing_frequency)
{
    if(swing_frequency >= 0)
        swing_frequency_ = swing_frequency;
    else
        PRINT_WARN_NAMED(CLASS_NAME,"Swing frequency has to be positive definite!");
}

double FootStateMachine::getDutyFactor()
{
    return duty_factor_;
}

double FootStateMachine::getStancePeriod()
{
    return T_stance_;
}

double FootStateMachine::getSwingPeriod()
{
    return T_swing_;
}

void FootStateMachine::update(const double& period, const bool& contact)
{

    prev_state_ = state_;

    updatePeriods();

    switch (state_)
    {

    case states::STANCE:

        stance_time_+=period;

        if(stance_time_ >= T_stance_)
            cycle_ended_ = true;

        if(trigger_swing_ && cycle_ended_)
        {
            state_ = states::SWING;
            cycle_ended_ = false;
        }

        break;

    case states::SWING:

        swing_time_+=period;

        //if(swing_frequency_>0)
        //  half_swing_time = 1/(2*swing_frequency_); // NOTE: since we use f'=2f, we should divide by 4 and not by 2.
        // If swing frequency is geq than 0
        // deactivate the contact sensing for half of the swing period
        if(contact && swing_time_>=T_swing_/2.0)
        {
            state_ = states::STANCE;
            init();
        }

        break;

    default:
        break;

    };
}

void FootStateMachine::init()
{
    stance_time_ = 0.0;
    swing_time_ = 0.0;
    trigger_swing_ = false;
}

void FootStateMachine::updatePeriods()
{
    T_swing_ = 1/swing_frequency_;
    T_stance_ = duty_factor_/(1 - duty_factor_) * T_swing_;
}
