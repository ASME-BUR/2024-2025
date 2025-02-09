#ifndef WAYPOINT_FOLLOWER_HPP
#define WAYPOINT_FOLLOWER_HPP

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/wrench.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "control_toolbox/pid.hpp"
#include <vector>
#include "geometry_msgs/msg/pose.hpp"

class WaypointFollower : public rclcpp::Node {
public:
    WaypointFollower();
    void run();

private:
    void odometryCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
    void nextWaypointCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
    void waypointsCallback(const nav_msgs::msg::Path::SharedPtr msg);
    geometry_msgs::msg::Wrench computeCommand();
    void loadParameters();

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr next_waypoint_sub_;
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr waypoints_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Wrench>::SharedPtr wrench_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    std::vector<geometry_msgs::msg::PoseStamped> waypoints_;
    geometry_msgs::msg::Pose target_pose_;
    double publish_rate_;

    // Force and torque limits
    double max_force_x_, max_force_y_, max_force_z_;
    double max_torque_x_, max_torque_y_, max_torque_z_;

    // PID controllers
    control_toolbox::Pid pid_force_x_, pid_force_y_, pid_force_z_;
    control_toolbox::Pid pid_torque_x_, pid_torque_y_, pid_torque_z_;

    // PID gains
    double kp_force_x_, ki_force_x_, kd_force_x_;
    double kp_force_y_, ki_force_y_, kd_force_y_;
    double kp_force_z_, ki_force_z_, kd_force_z_;

    double kp_torque_x_, ki_torque_x_, kd_torque_x_;
    double kp_torque_y_, ki_torque_y_, kd_torque_y_;
    double kp_torque_z_, ki_torque_z_, kd_torque_z_;

    nav_msgs::msg::Odometry current_odom_;
    bool odom_received_;
    bool target_received_;
    rclcpp::Time last_command_time_;
};

#endif // WAYPOINT_FOLLOWER_HPP