#include <ros/ros.h>
#include "hero_chassis_controller/hero_chassis_controller.h"
#include <pluginlib/class_list_macros.hpp>

namespace hero_chassis_controller {

HeroChassisController::HeroChassisController()
    : linear_velocity_x_(0.0),
      angular_velocity_z_(0.0),
      chassis_velocity_(0.0),
      last_time_(ros::Time(0)) {
    pose_.orientation = tf::createQuaternionMsgFromYaw(0.0);
}

bool HeroChassisController::init(hardware_interface::EffortJointInterface *effort_joint_interface,
                                 ros::NodeHandle &root_nh, ros::NodeHandle &controller_nh) {
  front_left_joint_ =
      effort_joint_interface->getHandle("left_front_wheel_joint");
  front_right_joint_ =
      effort_joint_interface->getHandle("right_front_wheel_joint");
  back_left_joint_ = effort_joint_interface->getHandle("left_back_wheel_joint");
  back_right_joint_ =
      effort_joint_interface->getHandle("right_back_wheel_joint");

  if (!controller_nh.getParam("wheel_params/wheel_base", wheel_base_) ||
      !controller_nh.getParam("wheel_params/wheel_track", wheel_track_) ||
      !controller_nh.getParam("velocity_mode", velocity_mode_)) {
    ROS_ERROR("Failed to get parameters");
    return false;
  }

  // Initialize PID controllers
  if (!pid_front_left_.init(ros::NodeHandle(controller_nh, "pid_params/front_left_pid"))) {
    ROS_ERROR("Failed to initialize PID for front left wheel");
    return false;
  }
  if (!pid_front_right_.init(ros::NodeHandle(controller_nh, "pid_params/front_right_pid"))) {
    ROS_ERROR("Failed to initialize PID for front right wheel");
    return false;
  }
  if (!pid_back_left_.init(ros::NodeHandle(controller_nh, "pid_params/back_left_pid"))) {
    ROS_ERROR("Failed to initialize PID for back left wheel");
    return false;
  }
  if (!pid_back_right_.init(ros::NodeHandle(controller_nh, "pid_params/back_right_pid"))) {
    ROS_ERROR("Failed to initialize PID for back right wheel");
    return false;
  }

  cmd_vel_sub_ = root_nh.subscribe("/cmd_vel", 1, &HeroChassisController::cmdVelCallback, this);
  odom_pub_ = root_nh.advertise<nav_msgs::Odometry>("/odom", 1);

  return true;
}

void HeroChassisController::update(const ros::Time &time, const ros::Duration &period) {
  // Calculate chassis velocity based on linear and angular velocities
  if (velocity_mode_ == "chassis") {
    chassis_velocity_ = linear_velocity_x_;
  } else if (velocity_mode_ == "global") {
    chassis_velocity_ = linear_velocity_x_ / std::cos(angular_velocity_z_);
  } else {
    ROS_WARN("Invalid velocity_mode specified, defaulting to chassis mode");
    chassis_velocity_ = linear_velocity_x_;
  }

  // Calculate wheel velocities using inverse kinematics
  double vx = chassis_velocity_;
  double vy = 0.0;  // Assuming no lateral motion for simplicity
  double omega = angular_velocity_z_;
  double d = wheel_base_ / 2.0 + wheel_track_ / 2.0;
  double front_left_velocity = vx + vy - d * omega;
  double front_right_velocity = vx - vy + d * omega;
  double back_left_velocity = vx - vy - d * omega;
  double back_right_velocity = vx + vy + d * omega;

  // Compute PID control effort for each wheel
  double front_left_effort = pid_front_left_.computeCommand(front_left_velocity, period);
  double front_right_effort = pid_front_right_.computeCommand(front_right_velocity, period);
  double back_left_effort = pid_back_left_.computeCommand(back_left_velocity, period);
  double back_right_effort = pid_back_right_.computeCommand(back_right_velocity, period);

  // Apply effort to each wheel
  front_left_joint_.setCommand(front_left_effort);
  front_right_joint_.setCommand(front_right_effort);
  back_left_joint_.setCommand(back_left_effort);
  back_right_joint_.setCommand(back_right_effort);

  // Update odom
  const double MIN_DELTA_TIME=0.01;
  double delta_time = (time - last_time_).toSec();
  if(delta_time>MIN_DELTA_TIME) {
      last_time_ = time;
  }

  double linear_distance = chassis_velocity_ * delta_time;
  double angular_distance = angular_velocity_z_ * delta_time;

  double delta_x = linear_distance * std::cos(angular_distance / 2.0);
  double delta_y = linear_distance * std::sin(angular_distance / 2.0);

  // Create a quaternion representing the change in orientation
  tf::Quaternion delta_orientation;
    delta_orientation.setRPY(0, 0, angular_distance);
    delta_orientation.normalize();

  // Update the pose
  pose_.position.x += (delta_x * std::cos(pose_.orientation.z) - delta_y * std::sin(pose_.orientation.z));
  pose_.position.y += (delta_x * std::sin(pose_.orientation.z) + delta_y * std::cos(pose_.orientation.z));
  pose_.orientation = tf::createQuaternionMsgFromYaw(tf::getYaw(pose_.orientation) + angular_distance);

  // Broadcast the transform
  tf::Transform transform;
  transform.setOrigin(tf::Vector3(pose_.position.x, pose_.position.y, 0.0));
  transform.setRotation(tf::Quaternion(pose_.orientation.x, pose_.orientation.y, pose_.orientation.z, pose_.orientation.w));

  tf_broadcaster_.sendTransform(tf::StampedTransform(transform, time, "odom", "base_link"));

  // Publish odometry
  nav_msgs::Odometry odom_msg;
  odom_msg.header.stamp = time;
  odom_msg.header.frame_id = "odom";
  odom_msg.child_frame_id = "base_link";

  odom_msg.pose.pose = pose_;
  odom_msg.twist.twist.linear.x = chassis_velocity_;
  odom_msg.twist.twist.angular.z = angular_velocity_z_;

  odom_pub_.publish(odom_msg);
}

    void HeroChassisController::cmdVelCallback(const geometry_msgs::Twist::ConstPtr &msg) {
        // Convert the speed command from the world coordinate system (odom) to the chassis coordinate system
        geometry_msgs::TwistStamped cmd_vel_world;
        cmd_vel_world.header.stamp = ros::Time::now();
        cmd_vel_world.header.frame_id = "odom";
        cmd_vel_world.twist = *msg;

        try {
            tf_listener_.waitForTransform("base_link", "odom", ros::Time(0), ros::Duration(1.0));

            // Transform linear velocity
            geometry_msgs::Vector3Stamped cmd_linear;
            cmd_linear.header.stamp = ros::Time(0);
            cmd_linear.header.frame_id = "odom";
            cmd_linear.vector.x = cmd_vel_world.twist.linear.x;
            cmd_linear.vector.y = cmd_vel_world.twist.linear.y;
            cmd_linear.vector.z = cmd_vel_world.twist.linear.z;
            tf_listener_.transformVector("base_link", cmd_linear, cmd_linear);

            // Transform angular velocity
            geometry_msgs::Vector3Stamped cmd_angular;
            cmd_angular.header.stamp = ros::Time(0);
            cmd_angular.header.frame_id = "odom";
            cmd_angular.vector.x = cmd_vel_world.twist.angular.x;
            cmd_angular.vector.y = cmd_vel_world.twist.angular.y;
            cmd_angular.vector.z = cmd_vel_world.twist.angular.z;
            tf_listener_.transformVector("base_link", cmd_angular, cmd_angular);

            cmd_vel_world.twist.linear.x = cmd_linear.vector.x;
            cmd_vel_world.twist.linear.y = cmd_linear.vector.y;
            cmd_vel_world.twist.linear.z = cmd_linear.vector.z;
            cmd_vel_world.twist.angular.x = cmd_angular.vector.x;
            cmd_vel_world.twist.angular.y = cmd_angular.vector.y;
            cmd_vel_world.twist.angular.z = cmd_angular.vector.z;

        } catch (tf::TransformException &ex) {
            ROS_ERROR("将cmd_vel从odom变换到base_link时出错：%s", ex.what());
            return;
        }

        linear_velocity_x_ = cmd_vel_world.twist.linear.x;
        angular_velocity_z_ = cmd_vel_world.twist.angular.z;
    }

PLUGINLIB_EXPORT_CLASS(hero_chassis_controller::HeroChassisController, controller_interface::ControllerBase)
}  // namespace hero_chassis_controller




