#include "ros/ros.h"
#include "std_msgs/Header.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/Twist.h"
#include "tf/transform_broadcaster.h"
#include "tf/LinearMath/Quaternion.h"

class OdometryPublisher {
public:
    OdometryPublisher() : nh("~") {
        // 初始化ROS参数
        nh.param<std::string>("odom_frame", odom_frame, "odom");
        nh.param<std::string>("base_frame", base_frame, "base_link");

        // 订阅/cmd_vel话题，用于接收机器人速度指令
        cmd_vel_sub = nh.subscribe("/cmd_vel", 10, &OdometryPublisher::cmdVelCallback, this);

        // 发布/odom话题
        odom_pub = nh.advertise<nav_msgs::Odometry>("/odom", 10);

        // 初始化变量
        x = y = theta = 0.0;
        last_time = ros::Time::now();
    }

    void cmdVelCallback(const geometry_msgs::Twist::ConstPtr& msg) {
        // 计算里程计信息
        ros::Time current_time = ros::Time::now();
        double dt = (current_time - last_time).toSec();

        double linear_vel_x = msg->linear.x;
        double linear_vel_y = msg->linear.y;
        double angular_vel = msg->angular.z;

        // 计算机器人的运动学模型（麦克纳姆轮底盘）
        double delta_x = (linear_vel_x * cos(theta) - linear_vel_y * sin(theta)) * dt;
        double delta_y = (linear_vel_x * sin(theta) + linear_vel_y * cos(theta)) * dt;
        double delta_theta = angular_vel * dt;

        // 更新机器人位置
        x += delta_x;
        y += delta_y;
        theta += delta_theta;

        // 发布里程计信息
        nav_msgs::Odometry odom;
        odom.header.stamp = current_time;
        odom.header.frame_id = odom_frame;
        odom.child_frame_id = base_frame;
        odom.pose.pose.position.x = x;
        odom.pose.pose.position.y = y;
        odom.pose.pose.position.z = 0.0;

        tf::Quaternion quat;
        quat.setRPY(0, 0, theta);
        tf::quaternionTFToMsg(quat, odom.pose.pose.orientation);

        odom_pub.publish(odom);

        // 发布坐标变换关系
        static tf::TransformBroadcaster br;
        tf::Transform transform;
        transform.setOrigin(tf::Vector3(x, y, 0.0));
        transform.setRotation(quat);
        br.sendTransform(tf::StampedTransform(transform, current_time, odom_frame, base_frame));

        last_time = current_time;
    }

private:
    ros::NodeHandle nh;
    ros::Subscriber cmd_vel_sub;
    ros::Publisher odom_pub;

    std::string odom_frame;
    std::string base_frame;

    double x, y, theta;
    ros::Time last_time;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "odometry_publisher");
    OdometryPublisher odometry_publisher;
    ros::spin();
    return 0;
}

