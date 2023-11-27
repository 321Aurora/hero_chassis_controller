#pragma once

#include <controller_interface/controller.h>
#include <hardware_interface/joint_command_interface.h>

class OdometryPublisherInterface : public controller_interface::Controller<hardware_interface::VelocityJointInterface> {
public:
    virtual bool init(hardware_interface::VelocityJointInterface* hw, ros::NodeHandle& nh) = 0;
    virtual void starting(const ros::Time& time) = 0;
    virtual void stopping(const ros::Time& time) = 0;
    virtual void update(const ros::Time& time, const ros::Duration& period) = 0;
};
