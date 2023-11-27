#pragma once

#include "mecanum_wheel_speed_calculator_interface.h"
#include <pluginlib/class_list_macros.h>

class MecanumWheelSpeedCalculatorController : public MecanumWheelSpeedCalculatorInterface {
public:
    MecanumWheelSpeedCalculatorController() = default;

    virtual bool init(hardware_interface::VelocityJointInterface* hw, ros::NodeHandle& nh) override;
    virtual void starting(const ros::Time& time) override;
    virtual void stopping(const ros::Time& time) override;
    virtual void update(const ros::Time& time, const ros::Duration& period) override;

private:
    // Add necessary member variables and functions for MecanumWheelSpeedCalculator
};

PLUGINLIB_EXPORT_CLASS(MecanumWheelSpeedCalculatorController, MecanumWheelSpeedCalculatorInterface)
