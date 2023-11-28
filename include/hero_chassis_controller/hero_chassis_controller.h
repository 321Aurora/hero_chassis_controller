#ifndef HERO_CHASSIS_CONTROLLER_HERO_CHASSIS_CONTROLLER_H
#define HERO_CHASSIS_CONTROLLER_HERO_CHASSIS_CONTROLLER_H

#include <controller_interface/controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <controller_interface/controller_base.h>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Bool.h>
#include <cmath>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <control_toolbox/pid.h>
#include <yaml-cpp/yaml.h>
#include <nav_msgs/Odometry.h>

namespace hero_chassis_controller {

class HeroChassisController : public controller_interface::Controller<hardware_interface::EffortJointInterface>{
public:
  HeroChassisController() ;
  ~HeroChassisController() override = default;
  bool init(hardware_interface::EffortJointInterface *effort_joint_interface,
            ros::NodeHandle &root_nh, ros::NodeHandle &controller_nh) override;

  void update(const ros::Time &time, const ros::Duration &period) override;

  void cmdVelCallback(const geometry_msgs::Twist::ConstPtr &msg) ;


  hardware_interface::JointHandle front_left_joint_;
  hardware_interface::JointHandle front_right_joint_;
  hardware_interface::JointHandle back_left_joint_;
  hardware_interface::JointHandle back_right_joint_;

  control_toolbox::Pid pid_front_left_;
  control_toolbox::Pid pid_front_right_;
  control_toolbox::Pid pid_back_left_;
  control_toolbox::Pid pid_back_right_;

  double wheel_base_;
  double wheel_track_;

  std::string velocity_mode_;

  ros::Subscriber cmd_vel_sub_;
  ros::Publisher odom_pub_;
  tf::TransformBroadcaster tf_broadcaster_;

  double linear_velocity_x_;
  double angular_velocity_z_;
  double chassis_velocity_;

  geometry_msgs::Pose pose_;

  ros::Time last_time_;

private:
    tf::TransformListener tf_listener_;
    geometry_msgs::TwistStamped cmd_vel_;
};

} // namespace hero_chassis_controller


#endif   // HERO_CHASSIS_CONTROLLER_HERO_CHASSIS_CONTROLLER_H


