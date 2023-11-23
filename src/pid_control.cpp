#include <iostream>
#include <yaml-cpp/yaml.h>
#include <control_toolbox/pid.h>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>

struct PIDParams {
    double p;
    double i;
    double d;
};

PIDParams loadPIDParams(const std::string& config_file, const std::string& pid_name) {
    try {
        YAML::Node config = YAML::LoadFile(config_file);
        return {config[pid_name]["p"].as<double>(),
                config[pid_name]["i"].as<double>(),
                config[pid_name]["d"].as<double>()};
    } catch (const YAML::Exception& e) {
        std::cerr << "Error loading PID parameters: " << e.what() << std::endl;
        return {1.0, 0.0, 0.0};  // 使用默认值，你可以根据需要进行调整
    }
}

class WheelController {
public:
    WheelController(const PIDParams& params, const ros::NodeHandle& nh, const std::string& wheel_name)
            : nh_(nh), wheel_name_(wheel_name) {
        pid_controller_.initPid(params.p, params.i, params.d, 0.0, 0.0);
        velocity_publisher_ = nh_.advertise<geometry_msgs::Twist>("/" + wheel_name + "/cmd_vel", 1);
        velocity_subscriber_ = nh_.subscribe<nav_msgs::Odometry>("/" + wheel_name + "/odom", 1,
                                                                 &WheelController::velocityCallback, this);
    }

    void update(double setpoint) {
        double current_speed = current_speed_;  // 使用保存的实际速度值

        double error = setpoint - current_speed;

        // 在这里根据麦克纳姆轮的运动学计算并发布控制速度
        geometry_msgs::Twist twist_msg;
        twist_msg.linear.x = pid_controller_.computeCommand(error, ros::Time::now() - last_time_);
        twist_msg.angular.z = 0.0;  // 角速度，也根据实际需要调整
        velocity_publisher_.publish(twist_msg);

        last_time_ = ros::Time::now();
    }

private:
    control_toolbox::Pid pid_controller_;
    ros::NodeHandle nh_;
    std::string wheel_name_;
    ros::Publisher velocity_publisher_;
    ros::Subscriber velocity_subscriber_;
    double current_speed_;
    ros::Time last_time_;

    void velocityCallback(const nav_msgs::Odometry::ConstPtr& msg) {
        // 当接收到底盘速度消息时调用，更新实际速度值
        current_speed_ = msg->twist.twist.linear.x;  // 假设线速度信息在twist消息中
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "mecanum_wheel_controller_node");
    ros::NodeHandle nh;

    std::string config_file_path = "pid_config.yaml";

    // 创建轮子控制器
    std::map<std::string, WheelController> wheel_controllers;
    wheel_controllers.emplace(std::piecewise_construct,
                              std::forward_as_tuple("wheel1"),
                              std::forward_as_tuple(loadPIDParams(config_file_path, "wheel1_pid"), nh, "wheel1"));
    wheel_controllers.emplace(std::piecewise_construct,
                              std::forward_as_tuple("wheel2"),
                              std::forward_as_tuple(loadPIDParams(config_file_path, "wheel2_pid"), nh, "wheel2"));
    wheel_controllers.emplace(std::piecewise_construct,
                              std::forward_as_tuple("wheel3"),
                              std::forward_as_tuple(loadPIDParams(config_file_path, "wheel3_pid"), nh, "wheel3"));
    wheel_controllers.emplace(std::piecewise_construct,
                              std::forward_as_tuple("wheel4"),
                              std::forward_as_tuple(loadPIDParams(config_file_path, "wheel4_pid"), nh, "wheel4"));

    // 在一个循环中运行 PID 控制
    for (int i = 0; i < 100; ++i) {
        double setpoint = 10.0;

        for (auto& kv : wheel_controllers) {
            WheelController& controller = kv.second;
            controller.update(setpoint);
        }

        ros::spinOnce();
        ros::Duration(0.1).sleep();
    }

    return 0;
}
