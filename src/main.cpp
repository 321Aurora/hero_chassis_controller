
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
#include <hero_chassis_controller/hero_chassis_controller.h>
#include <hardware_interface/joint_command_interface.h>

namespace hero_chassis_controller {
struct PIDParams {
    double p;
    double i;
    double d;
};

struct WheelParams {
    double wheelbase;
    double track;
};

WheelParams loadWheelParams(const std::string& config_file) {
    try {
        YAML::Node config = YAML::LoadFile(config_file);
        return {config["wheelbase"].as<double>(), config["track"].as<double>()};
    } catch (const YAML::Exception& e) {
        std::cerr << "Error loading wheel parameters: " << e.what() << std::endl;
        return {0.5, 0.3};  // Default values, adjust as needed
    }
}

PIDParams loadPIDParams(const std::string& config_file, const std::string& pid_name) {
    try {
        YAML::Node config = YAML::LoadFile(config_file);
        return {config[pid_name]["p"].as<double>(),
                config[pid_name]["i"].as<double>(),
                config[pid_name]["d"].as<double>()};
    } catch (const YAML::Exception& e) {
        std::cerr << "Error loading PID parameters: " << e.what() << std::endl;
        return {1.0, 0.0, 0.0};  // Default values, adjust as needed
    }
}

class MecanumWheelSpeedCalculator {
public:
    MecanumWheelSpeedCalculator(const std::string& config_file) : nh("~"), config_file_(config_file) {
        WheelParams wheel_params = loadWheelParams(config_file_);
        wheelbase = wheel_params.wheelbase;
        track = wheel_params.track;

        front_left_wheel_pub = nh.advertise<std_msgs::Float64>("/front_left_wheel_speed", 10);
        front_right_wheel_pub = nh.advertise<std_msgs::Float64>("/front_right_wheel_speed", 10);
        rear_left_wheel_pub = nh.advertise<std_msgs::Float64>("/rear_left_wheel_speed", 10);
        rear_right_wheel_pub = nh.advertise<std_msgs::Float64>("/rear_right_wheel_speed", 10);

        // Initialize PID parameters with default values or from config file
        front_left_pid_params_ = loadPIDParams(config_file_, "front_left_pid");
        front_right_pid_params_ = loadPIDParams(config_file_, "front_right_pid");
        rear_left_pid_params_ = loadPIDParams(config_file_, "rear_left_pid");
        rear_right_pid_params_ = loadPIDParams(config_file_, "rear_right_pid");

        // Initialize PID controllers
        front_left_pid_.initPid(front_left_pid_params_.p, front_left_pid_params_.i, front_left_pid_params_.d, 1.0, -1.0);
        front_right_pid_.initPid(front_right_pid_params_.p, front_right_pid_params_.i, front_right_pid_params_.d, 1.0, -1.0);
        rear_left_pid_.initPid(rear_left_pid_params_.p, rear_left_pid_params_.i, rear_left_pid_params_.d, 1.0, -1.0);
        rear_right_pid_.initPid(rear_right_pid_params_.p, rear_right_pid_params_.i, rear_right_pid_params_.d, 1.0, -1.0);
    }



    void twistCallback(const geometry_msgs::Twist::ConstPtr& twist_msg) {
        double linear_x = twist_msg->linear.x;
        double linear_y = twist_msg->linear.y;
        double angular_z = twist_msg->angular.z;

        double front_left_wheel_speed = linear_x - linear_y - (wheelbase + track) * angular_z;
        double front_right_wheel_speed = linear_x + linear_y + (wheelbase + track) * angular_z;
        double rear_left_wheel_speed = linear_x + linear_y - (wheelbase + track) * angular_z;
        double rear_right_wheel_speed = linear_x - linear_y + (wheelbase + track) * angular_z;

        front_left_wheel_speed_ = front_left_pid_.computeCommand(front_left_wheel_speed, ros::Duration(1));
        front_right_wheel_speed_ = front_right_pid_.computeCommand(front_right_wheel_speed, ros::Duration(1));
        rear_left_wheel_speed_ = rear_left_pid_.computeCommand(rear_left_wheel_speed, ros::Duration(1));
        rear_right_wheel_speed_ = rear_right_pid_.computeCommand(rear_right_wheel_speed, ros::Duration(1));
    }

    void publishWheelSpeeds() {
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
    ros::Publisher front_left_wheel_pub, front_right_wheel_pub, rear_left_wheel_pub, rear_right_wheel_pub;
    double wheelbase;
    double track;
    std::string config_file_;
    PIDParams front_left_pid_params_;
    PIDParams front_right_pid_params_;
    PIDParams rear_left_pid_params_;
    PIDParams rear_right_pid_params_;
    control_toolbox::Pid front_left_pid_;
    control_toolbox::Pid front_right_pid_;
    control_toolbox::Pid rear_left_pid_;
    control_toolbox::Pid rear_right_pid_;
    double front_left_wheel_speed_;
    double front_right_wheel_speed_;
    double rear_left_wheel_speed_;
    double rear_right_wheel_speed_;
};

class VelocityTransformer {
public:
    VelocityTransformer(const std::string& config_file) : nh_("~"), config_file_(config_file) {
        // 从配置文件中加载参数
        nh_.param<std::string>("base_frame", base_frame_, "base_link");
        nh_.param<std::string>("global_frame", global_frame_, "odom");
        nh_.param<bool>("use_inverse_kinematics", use_inverse_kinematics_, false);

        // 设置ROS订阅器
        cmd_vel_sub_ = nh_.subscribe("/cmd_vel", 1, &VelocityTransformer::cmdVelCallback, this);

        // 初始化底盘速度控制器
        mecanum_wheel_speed_calculator_ = new MecanumWheelSpeedCalculator(config_file_);

        // 发布各个轮子的期望速度
        front_left_wheel_pub_ = nh_.advertise<std_msgs::Float64>("/front_left_wheel_speed", 10);
        front_right_wheel_pub_ = nh_.advertise<std_msgs::Float64>("/front_right_wheel_speed", 10);
        rear_left_wheel_pub_ = nh_.advertise<std_msgs::Float64>("/rear_left_wheel_speed", 10);
        rear_right_wheel_pub_ = nh_.advertise<std_msgs::Float64>("/rear_right_wheel_speed", 10);

        // 订阅用于切换速度模式的参数
        use_inverse_kinematics_param_sub_ = nh_.subscribe("/use_inverse_kinematics_param", 1, &VelocityTransformer::updateInverseKinematicsParam, this);

        // 初始化TF广播器
        tf_broadcaster_ = new tf::TransformBroadcaster();
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
        tf::TransformListener tf_listener;
        tf::StampedTransform transform;

        ros::Time current_time = ros::Time::now();

        try {
            // 检查变换是否可用
            if (!tf_listener.waitForTransform(global_frame_, base_frame_, current_time, ros::Duration(5.0))) {
                ROS_WARN("无法从 %s 转换到 %s", global_frame_.c_str(), base_frame_.c_str());
                return false;
            }

            // 获取变换
            tf_listener.lookupTransform(global_frame_, base_frame_, current_time, transform);

            // 转换线性速度
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

        } catch (tf::TransformException& ex) {
            ROS_ERROR("%s", ex.what());
            return false;
        }

        return true;
    }

    void executeBaseVelocityCommand(const geometry_msgs::Twist& base_cmd_vel) {
        // 将底盘速度指令发送给底盘速度控制器
        mecanum_wheel_speed_calculator_->twistCallback(boost::make_shared<geometry_msgs::Twist>(base_cmd_vel));

        // 发布期望速度
        mecanum_wheel_speed_calculator_->publishWheelSpeeds();

        // 发布 "odom" 到 "base_link" 的坐标变换
        publishOdomToBaseLinkTransform();
    }

    void calculateWheelVelocities(const geometry_msgs::Twist& base_cmd_vel) {
        // 在这里实现麦克纳姆轮的逆运动学计算，得到各个轮子的期望速度
        // 使用底盘速度控制器的逆运动学计算功能
        mecanum_wheel_speed_calculator_->twistCallback(boost::make_shared<geometry_msgs::Twist>(base_cmd_vel));

        // 发布期望速度
        mecanum_wheel_speed_calculator_->publishWheelSpeeds();

        // 发布 "odom" 到 "base_link" 的坐标变换
        publishOdomToBaseLinkTransform();
    }

    void publishOdomToBaseLinkTransform() {
        // 获取 "odom" 到 "base_link" 的坐标变换
        tf::Transform transform;
        transform.setOrigin(tf::Vector3(0.0, 0.0, 0.0));
        tf::Quaternion quat;
        quat.setRPY(0, 0, 0);
        transform.setRotation(quat);

        // 发布坐标变换
        tf_broadcaster_->sendTransform(tf::StampedTransform(transform, ros::Time::now(), "odom", "base_link"));
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
    tf::TransformBroadcaster* tf_broadcaster_;
    std::string config_file_;
};

class OdometryPublisher {
public:
    OdometryPublisher(const std::string& config_file) : nh("~") {
        // 初始化ROS参数
        nh.param<std::string>("odom_frame", odom_frame, "odom");
        nh.param<std::string>("base_frame", base_frame, "base_link");

        // 订阅/cmd_vel话题，用于接收机器人速度指令
        cmd_vel_sub = nh.subscribe("/cmd_vel", 10, &OdometryPublisher::cmdVelCallback, this);

        // 发布/odom话题
        odom_pub = nh.advertise<nav_msgs::Odometry>("/odom", 10);

        // 加载PID参数
        PIDParams linear_pid_params = loadPIDParams(config_file, "linear_pid");
        PIDParams angular_pid_params = loadPIDParams(config_file, "angular_pid");

        // 初始化线速度和角速度PID控制器
        linear_pid.initPid(linear_pid_params.p, linear_pid_params.i, linear_pid_params.d, 1.0, -1.0);
        angular_pid.initPid(angular_pid_params.p, angular_pid_params.i, angular_pid_params.d, 1.0, -1.0);

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

        // 计算线速度和角速度的PID输出
        double linear_error = sqrt(pow(x, 2) + pow(y, 2));
        double angular_error = theta;

        double linear_pid_output = linear_pid.computeCommand(linear_error, ros::Duration(dt));
        double angular_pid_output = angular_pid.computeCommand(angular_error, ros::Duration(dt));

        // 根据PID输出调整机器人速度
        double corrected_linear_vel_x = linear_vel_x + linear_pid_output;
        double corrected_angular_vel = angular_vel + angular_pid_output;

        // 发布校正后的速度指令
        geometry_msgs::Twist corrected_cmd_vel;
        corrected_cmd_vel.linear.x = corrected_linear_vel_x;
        corrected_cmd_vel.linear.y = linear_vel_y;
        corrected_cmd_vel.angular.z = corrected_angular_vel;

        // 发布校正后的速度指令
        corrected_cmd_vel_pub.publish(corrected_cmd_vel);

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
    ros::Publisher corrected_cmd_vel_pub = nh.advertise<geometry_msgs::Twist>("/corrected_cmd_vel", 10);
    std::string odom_frame;
    std::string base_frame;
    double x, y, theta;
    ros::Time last_time;
    control_toolbox::Pid linear_pid;
    control_toolbox::Pid angular_pid;
};


int main(int argc, char** argv) {
    ros::init(argc, argv, "mecanum_controller");
    std::string config_file ="controllers.yaml";

    if (argc < 2) {
        ROS_WARN("未提供配置文件。使用默认参数。");
    } else {
        config_file = argv[1];
        ROS_INFO("使用配置文件：%s", config_file.c_str());
    }

    MecanumWheelSpeedCalculator mecanum_wheel_speed_calculator(config_file);
    VelocityTransformer velocity_transformer(config_file);
    OdometryPublisher odometry_publisher(config_file);

    ros::spin();
    return 0;
 }
}
