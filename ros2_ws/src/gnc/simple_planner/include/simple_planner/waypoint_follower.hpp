#ifndef WAYPOINT_FOLLOWER_HPP
#define WAYPOINT_FOLLOWER_HPP

#include <ros/ros.h>
#include <geometry_msgs/Wrench.h>
#include <nav_msgs/Odometry.h>
#include <control_toolbox/pid.h>
#include <vector>
#include <geometry_msgs/Pose.h>

class WaypointFollower {
public:
    WaypointFollower(ros::NodeHandle& nh);
    void run();

private:
    void odometryCallback(const nav_msgs::Odometry::ConstPtr& msg);
    geometry_msgs::Wrench computeCommand();
    void loadParameters();
    bool loadWaypoints();

    ros::NodeHandle nh_;
    ros::Subscriber odom_sub_;
    ros::Publisher wrench_pub_;

    std::vector<geometry_msgs::Pose> waypoints_;
    size_t current_waypoint_;
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

    nav_msgs::Odometry current_odom_;
    bool odom_received_;
    ros::Time last_command_time_;
};

#endif // WAYPOINT_FOLLOWER_HPP