#include "waypoint_follower.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

WaypointFollower::WaypointFollower() 
    : Node("waypoint_follower"), 
      odom_received_(false),
      target_received_(false) {
    
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/odometry/filtered", 10, 
        std::bind(&WaypointFollower::odometryCallback, this, std::placeholders::_1));
    
    next_waypoint_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "next_waypoint", 10,
        std::bind(&WaypointFollower::nextWaypointCallback, this, std::placeholders::_1));
    
    waypoints_sub_ = this->create_subscription<nav_msgs::msg::Path>(
        "waypoints", 10,
        std::bind(&WaypointFollower::waypointsCallback, this, std::placeholders::_1));
    
    wrench_pub_ = this->create_publisher<geometry_msgs::msg::Wrench>("/cmd_wrench", 10);
    
    loadParameters();
    last_command_time_ = this->now();
}

void WaypointFollower::loadParameters() {
    // Load publishing rate
    publish_rate_ = this->declare_parameter("publish_rate", 30.0);
    
    // Load force and torque limits
    max_force_x_ = this->declare_parameter("max_force_x", 50.0);
    max_force_y_ = this->declare_parameter("max_force_y", 50.0);
    max_force_z_ = this->declare_parameter("max_force_z", 50.0);
    max_torque_x_ = this->declare_parameter("max_torque_x", 10.0);
    max_torque_y_ = this->declare_parameter("max_torque_y", 10.0);
    max_torque_z_ = this->declare_parameter("max_torque_z", 5.0);

    // Load PID gains for force
    kp_force_x_ = this->declare_parameter("kp_force_x", 1.0);
    ki_force_x_ = this->declare_parameter("ki_force_x", 0.01);
    kd_force_x_ = this->declare_parameter("kd_force_x", 0.1);
    kp_force_y_ = this->declare_parameter("kp_force_y", 1.0);
    ki_force_y_ = this->declare_parameter("ki_force_y", 0.01);
    kd_force_y_ = this->declare_parameter("kd_force_y", 0.1);
    kp_force_z_ = this->declare_parameter("kp_force_z", 1.0);
    ki_force_z_ = this->declare_parameter("ki_force_z", 0.01);
    kd_force_z_ = this->declare_parameter("kd_force_z", 0.1);

    // Load PID gains for torque
    kp_torque_x_ = this->declare_parameter("kp_torque_x", 0.5);
    ki_torque_x_ = this->declare_parameter("ki_torque_x", 0.005);
    kd_torque_x_ = this->declare_parameter("kd_torque_x", 0.05);
    kp_torque_y_ = this->declare_parameter("kp_torque_y", 0.5);
    ki_torque_y_ = this->declare_parameter("ki_torque_y", 0.005);
    kd_torque_y_ = this->declare_parameter("kd_torque_y", 0.05);
    kp_torque_z_ = this->declare_parameter("kp_torque_z", 0.5);
    ki_torque_z_ = this->declare_parameter("ki_torque_z", 0.005);
    kd_torque_z_ = this->declare_parameter("kd_torque_z", 0.05);

    // Initialize PID controllers
    pid_force_x_.initPid(kp_force_x_, ki_force_x_, kd_force_x_, max_force_x_, -max_force_x_);
    pid_force_y_.initPid(kp_force_y_, ki_force_y_, kd_force_y_, max_force_y_, -max_force_y_);
    pid_force_z_.initPid(kp_force_z_, ki_force_z_, kd_force_z_, max_force_z_, -max_force_z_);
    pid_torque_x_.initPid(kp_torque_x_, ki_torque_x_, kd_torque_x_, max_torque_x_, -max_torque_x_);
    pid_torque_y_.initPid(kp_torque_y_, ki_torque_y_, kd_torque_y_, max_torque_y_, -max_torque_y_);
    pid_torque_z_.initPid(kp_torque_z_, ki_torque_z_, kd_torque_z_, max_torque_z_, -max_torque_z_);
}

void WaypointFollower::nextWaypointCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    target_pose_ = msg->pose.pose;
    target_received_ = true;
    RCLCPP_INFO(this->get_logger(), "Received new target waypoint");
}

void WaypointFollower::waypointsCallback(const nav_msgs::msg::Path::SharedPtr msg) {
    waypoints_ = msg->poses;
    RCLCPP_INFO(this->get_logger(), "Received %zu waypoints", waypoints_.size());
}

void WaypointFollower::odometryCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    current_odom_ = *msg;
    odom_received_ = true;
}

geometry_msgs::msg::Wrench WaypointFollower::computeCommand() {
    geometry_msgs::msg::Wrench cmd;
    if (!target_received_) {
        return cmd;
    }

    // Compute position errors in world frame
    double error_x = target_pose_.position.x - current_odom_.pose.pose.position.x;
    double error_y = target_pose_.position.y - current_odom_.pose.pose.position.y;
    double error_z = target_pose_.position.z - current_odom_.pose.pose.position.z;

    // Convert quaternions to RPY for both current pose and target
    tf2::Quaternion q_current(
        current_odom_.pose.pose.orientation.x,
        current_odom_.pose.pose.orientation.y,
        current_odom_.pose.pose.orientation.z,
        current_odom_.pose.pose.orientation.w);

    tf2::Quaternion q_target(
        target_pose_.orientation.x,
        target_pose_.orientation.y,
        target_pose_.orientation.z,
        target_pose_.orientation.w);

    double roll_current, pitch_current, yaw_current;
    double roll_target, pitch_target, yaw_target;

    tf2::Matrix3x3(q_current).getRPY(roll_current, pitch_current, yaw_current);
    tf2::Matrix3x3(q_target).getRPY(roll_target, pitch_target, yaw_target);

    // Rotate the translation error into the robot's local frame
    tf2::Vector3 error_world(error_x, error_y, error_z);
    tf2::Vector3 error_body = tf2::quatRotate(q_current.inverse(), error_world);

    // Extract rotated errors
    double error_x_body = error_body.x();
    double error_y_body = error_body.y();
    double error_z_body = error_body.z();

    // Compute orientation errors
    double error_roll = roll_target - roll_current;
    double error_pitch = pitch_target - pitch_current;
    double error_yaw = yaw_target - yaw_current;

    auto now = this->now();
    rclcpp::Duration dt = now - last_command_time_;
    last_command_time_ = now;

    RCLCPP_INFO(this->get_logger(), "x_error: %f", error_x_body);

    // Compute forces using PID controllers (now using rotated errors)
    double force_x = pid_force_x_.computeCommand(error_x_body, dt.nanoseconds());
    double force_y = pid_force_y_.computeCommand(error_y_body, dt.nanoseconds());
    double force_z = pid_force_z_.computeCommand(error_z_body, dt.nanoseconds());

    // Compute torques using PID controllers
    double torque_x = pid_torque_x_.computeCommand(error_roll, dt.nanoseconds());
    double torque_y = pid_torque_y_.computeCommand(error_pitch, dt.nanoseconds());
    double torque_z = pid_torque_z_.computeCommand(error_yaw, dt.nanoseconds());

    RCLCPP_INFO(this->get_logger(), "x_force: %f", force_x);

    // Apply force limits
    cmd.force.x = std::clamp(force_x, -max_force_x_, max_force_x_);
    cmd.force.y = std::clamp(force_y, -max_force_y_, max_force_y_);
    cmd.force.z = std::clamp(force_z, -max_force_z_, max_force_z_);

    // Apply torque limits
    cmd.torque.x = std::clamp(torque_x, -max_torque_x_, max_torque_x_);
    cmd.torque.y = std::clamp(torque_y, -max_torque_y_, max_torque_y_);
    cmd.torque.z = std::clamp(torque_z, -max_torque_z_, max_torque_z_);

    return cmd;
}


void WaypointFollower::run() {
    auto timer_callback = [this]() -> void {
        if (odom_received_ && target_received_) {
            auto cmd = computeCommand();
            wrench_pub_->publish(cmd);
        }
    };

    timer_ = this->create_wall_timer(
        std::chrono::duration<double>(1.0/publish_rate_),
        timer_callback);

    rclcpp::spin(this->get_node_base_interface());
}

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<WaypointFollower>();
    
    try {
        node->run();
    } catch (const std::exception& e) {
        RCLCPP_ERROR(node->get_logger(), "Exception in waypoint follower node: %s", e.what());
        return 1;
    }
    
    rclcpp::shutdown();
    return 0;
}