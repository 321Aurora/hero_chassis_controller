#pragma once

#include "velocity_transformer_interface.h"
#include <pluginlib/class_list_macros.h>

class VelocityTransformerController : public VelocityTransformerInterface {
public:
    VelocityTransformerController() = default;

    virtual bool init(hardware_interface::VelocityJointInterface* hw, ros::NodeHandle& nh) override;
    virtual void starting(const ros::Time& time) override;
    virtual void stopping(const ros::Time& time) override;
    virtual void update(const ros::Time& time, const ros::Duration& period) override;

private:
    // Add necessary member variables and functions for VelocityTransformer
};

PLUGINLIB_EXPORT_CLASS(VelocityTransformerController, VelocityTransformerInterface)
