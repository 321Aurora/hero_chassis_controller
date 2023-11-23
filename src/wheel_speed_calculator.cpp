#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "std_msgs/Float64.h"
#include <cmath>

class MecanumWheelSpeedCalculator {
public:
    MecanumWheelSpeedCalculator() {
        // 从参数服务器获取底盘参数
        ros::NodeHandle nh("~");
        nh.param<double>("wheelbase", wheelbase, 0.5);  // 轴距
        nh.param<double>("track", track, 0.3);           // 轮距
        nh.param<std::string>("base_frame", base_frame, "base_link");  // 底盘坐标系

        // 订阅速度指令
        twist_sub = nh.subscribe("/cmd_vel", 10, &MecanumWheelSpeedCalculator::twistCallback, this);

        // 发布各个轮子的期望速度
        front_left_wheel_pub = nh.advertise<std_msgs::Float64>("/front_left_wheel_speed", 10);
        front_right_wheel_pub = nh.advertise<std_msgs::Float64>("/front_right_wheel_speed", 10);
        rear_left_wheel_pub = nh.advertise<std_msgs::Float64>("/rear_left_wheel_speed", 10);
        rear_right_wheel_pub = nh.advertise<std_msgs::Float64>("/rear_right_wheel_speed", 10);
    }

    void twistCallback(const geometry_msgs::Twist::ConstPtr& twist_msg) {
        double linear_x = twist_msg->linear.x;
        double linear_y = twist_msg->linear.y;
        double angular_z = twist_msg->angular.z;

        // 计算各个轮子的期望速度
        double front_left_wheel_speed = linear_x - linear_y - (wheelbase + track) * angular_z;
        double front_right_wheel_speed = linear_x + linear_y + (wheelbase + track) * angular_z;
        double rear_left_wheel_speed = linear_x + linear_y - (wheelbase + track) * angular_z;
        double rear_right_wheel_speed = linear_x - linear_y + (wheelbase + track) * angular_z;

        // 发布期望速度
        std_msgs::Float64 front_left_wheel_msg, front_right_wheel_msg, rear_left_wheel_msg, rear_right_wheel_msg;
        front_left_wheel_msg.data = front_left_wheel_speed;
        front_right_wheel_msg.data = front_right_wheel_speed;
        rear_left_wheel_msg.data = rear_left_wheel_speed;
        rear_right_wheel_msg.data = rear_right_wheel_speed;

        front_left_wheel_pub.publish(front_left_wheel_msg);
        front_right_wheel_pub.publish(front_right_wheel_msg);
        rear_left_wheel_pub.publish(rear_left_wheel_msg);
        rear_right_wheel_pub.publish(rear_right_wheel_msg);
    }

    void run() {
        ros::spin();
    }

private:
    ros::NodeHandle nh;
    ros::Subscriber twist_sub;
    ros::Publisher front_left_wheel_pub, front_right_wheel_pub, rear_left_wheel_pub, rear_right_wheel_pub;
    double wheelbase;  // 轴距
    double track;      // 轮距
    std::string base_frame;  // 底盘坐标系
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "mecanum_wheel_speed_calculator");
    MecanumWheelSpeedCalculator mecanum_wheel_speed_calculator;
    mecanum_wheel_speed_calculator.run();
    return 0;
}
