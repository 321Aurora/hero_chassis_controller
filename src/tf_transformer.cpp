#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Bool.h>
#include <cmath>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>

class MecanumWheelSpeedCalculator {
public:
    MecanumWheelSpeedCalculator() {
        // 从参数服务器获取底盘参数
        ros::NodeHandle nh("~");
        nh.param<double>("wheelbase", wheelbase, 0.5);  // 轴距
        nh.param<double>("track", track, 0.3);           // 轮距
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
        publishWheelSpeeds(front_left_wheel_speed, front_right_wheel_speed, rear_left_wheel_speed, rear_right_wheel_speed);
    }

    double getFrontLeftWheelSpeed() const {
        return front_left_wheel_speed_;
    }

    double getFrontRightWheelSpeed() const {
        return front_right_wheel_speed_;
    }

    double getRearLeftWheelSpeed() const {
        return rear_left_wheel_speed_;
    }

    double getRearRightWheelSpeed() const {
        return rear_right_wheel_speed_;
    }

    void publishWheelSpeeds() {
        // 发布期望速度
        std_msgs::Float64 front_left_wheel_msg, front_right_wheel_msg, rear_left_wheel_msg, rear_right_wheel_msg;
        front_left_wheel_msg.data = front_left_wheel_speed_;
        front_right_wheel_msg.data = front_right_wheel_speed_;
        rear_left_wheel_msg.data = rear_left_wheel_speed_;
        rear_right_wheel_msg.data = rear_right_wheel_speed_;

        front_left_wheel_pub.publish(front_left_wheel_msg);
        front_right_wheel_pub.publish(front_right_wheel_msg);
        rear_left_wheel_pub.publish(rear_left_wheel_msg);
        rear_right_wheel_pub.publish(rear_right_wheel_msg);
    }

private:
    ros::NodeHandle nh;
    ros::Publisher front_left_wheel_pub = nh.advertise<std_msgs::Float64>("/front_left_wheel_speed", 10);
    ros::Publisher front_right_wheel_pub = nh.advertise<std_msgs::Float64>("/front_right_wheel_speed", 10);
    ros::Publisher rear_left_wheel_pub = nh.advertise<std_msgs::Float64>("/rear_left_wheel_speed", 10);
    ros::Publisher rear_right_wheel_pub = nh.advertise<std_msgs::Float64>("/rear_right_wheel_speed", 10);
    double wheelbase;  // 轴距
    double track;      // 轮距
    double front_left_wheel_speed_;
    double front_right_wheel_speed_;
    double rear_left_wheel_speed_;
    double rear_right_wheel_speed_;

    void publishWheelSpeeds(double front_left, double front_right, double rear_left, double rear_right) {
        // 更新内部变量
        front_left_wheel_speed_ = front_left;
        front_right_wheel_speed_ = front_right;
        rear_left_wheel_speed_ = rear_left;
        rear_right_wheel_speed_ = rear_right;

        // 调用发布函数
        publishWheelSpeeds();
    }
};

class VelocityTransformer {
public:
    VelocityTransformer() : nh_("~") {
        // 从配置文件中加载参数
        nh_.param<std::string>("base_frame", base_frame_, "base_link");
        nh_.param<std::string>("global_frame", global_frame_, "odom");
        nh_.param<bool>("use_inverse_kinematics", use_inverse_kinematics_, false);

        // 设置ROS订阅器
        cmd_vel_sub_ = nh_.subscribe("/cmd_vel", 1, &VelocityTransformer::cmdVelCallback, this);

        // 初始化底盘速度控制器
        mecanum_wheel_speed_calculator_ = new MecanumWheelSpeedCalculator();

        // 发布各个轮子的期望速度
        front_left_wheel_pub_ = nh_.advertise<std_msgs::Float64>("/front_left_wheel_speed", 10);
        front_right_wheel_pub_ = nh_.advertise<std_msgs::Float64>("/front_right_wheel_speed", 10);
        rear_left_wheel_pub_ = nh_.advertise<std_msgs::Float64>("/rear_left_wheel_speed", 10);
        rear_right_wheel_pub_ = nh_.advertise<std_msgs::Float64>("/rear_right_wheel_speed", 10);

        // 订阅用于切换速度模式的参数
        use_inverse_kinematics_param_sub_ = nh_.subscribe("/use_inverse_kinematics_param", 1, &VelocityTransformer::updateInverseKinematicsParam, this);
    }

    void cmdVelCallback(const geometry_msgs::Twist::ConstPtr& cmd_vel_msg) {
        // 将速度从全局坐标系转换到底盘坐标系
        geometry_msgs::Twist base_cmd_vel;
        if (transformVelocity(cmd_vel_msg, base_cmd_vel)) {
            // 在底盘坐标系下执行速度指令
            if (use_inverse_kinematics_) {
                // 使用逆运动学计算各轮子的期望速度
                calculateWheelVelocities(base_cmd_vel);
            } else {
                // 直接在底盘坐标系下执行速度指令
                executeBaseVelocityCommand(base_cmd_vel);
            }
        }
    }

    void updateInverseKinematicsParam(const std_msgs::Bool::ConstPtr& inverse_kinematics_msg) {
        // 更新 use_inverse_kinematics_ 参数
        use_inverse_kinematics_ = inverse_kinematics_msg->data;
        ROS_INFO("Updated use_inverse_kinematics parameter to: %s", use_inverse_kinematics_ ? "true" : "false");
    }

    bool transformVelocity(const geometry_msgs::Twist::ConstPtr& cmd_vel_msg, geometry_msgs::Twist& base_cmd_vel) {
        tf::TransformListener listener;
        tf::StampedTransform transform;
        try {
            listener.waitForTransform(global_frame_, base_frame_, ros::Time(0), ros::Duration(3.0));
            listener.lookupTransform(global_frame_, base_frame_, ros::Time(0), transform);
        } catch (tf::TransformException& ex) {
            ROS_ERROR("%s", ex.what());
            return false;
        }

        // 转换线速度
        tf::Vector3 linear_vel(cmd_vel_msg->linear.x, cmd_vel_msg->linear.y, cmd_vel_msg->linear.z);
        linear_vel = transform.getBasis() * linear_vel;
        base_cmd_vel.linear.x = linear_vel.x();
        base_cmd_vel.linear.y = linear_vel.y();
        base_cmd_vel.linear.z = linear_vel.z();

        // 转换角速度
        tf::Vector3 angular_vel(cmd_vel_msg->angular.x, cmd_vel_msg->angular.y, cmd_vel_msg->angular.z);
        angular_vel = transform.getBasis() * angular_vel;
        base_cmd_vel.angular.x = angular_vel.x();
        base_cmd_vel.angular.y = angular_vel.y();
        base_cmd_vel.angular.z = angular_vel.z();

        return true;
    }

    void executeBaseVelocityCommand(const geometry_msgs::Twist& base_cmd_vel) {
        // 将底盘速度指令发送给底盘速度控制器
        mecanum_wheel_speed_calculator_->twistCallback(boost::make_shared<geometry_msgs::Twist>(base_cmd_vel));

        // 发布期望速度
        mecanum_wheel_speed_calculator_->publishWheelSpeeds();
    }

    void calculateWheelVelocities(const geometry_msgs::Twist& base_cmd_vel) {
        // 在这里实现麦克纳姆轮的逆运动学计算，得到各个轮子的期望速度
        // 使用底盘速度控制器的逆运动学计算功能
        mecanum_wheel_speed_calculator_->twistCallback(boost::make_shared<geometry_msgs::Twist>(base_cmd_vel));

        // 发布期望速度
        mecanum_wheel_speed_calculator_->publishWheelSpeeds();
    }

private:
    ros::NodeHandle nh_;
    ros::Subscriber cmd_vel_sub_;
    ros::Publisher front_left_wheel_pub_, front_right_wheel_pub_, rear_left_wheel_pub_, rear_right_wheel_pub_;
    std::string base_frame_;
    std::string global_frame_;
    bool use_inverse_kinematics_;
    MecanumWheelSpeedCalculator* mecanum_wheel_speed_calculator_;
    ros::Subscriber use_inverse_kinematics_param_sub_;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "velocity_transformer_node");
    VelocityTransformer velocity_transformer;
    ros::spin();
    return 0;
}
