#include "WaypointFollower.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

WaypointFollower::WaypointFollower(ros::NodeHandle& nh) 
    : nh_(nh), current_waypoint_(0), odom_received_(false) {
    
    odom_sub_ = nh_.subscribe("/odometry", 10, &WaypointFollower::odometryCallback, this);
    wrench_pub_ = nh_.advertise<geometry_msgs::Wrench>("/cmd_wrench", 10);

    loadParameters();
    loadWaypoints();
    last_command_time_ = ros::Time::now();
}

void WaypointFollower::loadParameters() {
    // Load publishing rate
    nh_.param("publish_rate", publish_rate_, 10.0);

    // Load force and torque limits
    nh_.param("max_force_x", max_force_x_, 50.0);
    nh_.param("max_force_y", max_force_y_, 50.0);
    nh_.param("max_force_z", max_force_z_, 50.0);
    nh_.param("max_torque_x", max_torque_x_, 10.0);
    nh_.param("max_torque_y", max_torque_y_, 10.0);
    nh_.param("max_torque_z", max_torque_z_, 5.0);

    // Load PID gains for force
    nh_.param("kp_force_x", kp_force_x_, 1.0);
    nh_.param("ki_force_x", ki_force_x_, 0.01);
    nh_.param("kd_force_x", kd_force_x_, 0.1);

    nh_.param("kp_force_y", kp_force_y_, 1.0);
    nh_.param("ki_force_y", ki_force_y_, 0.01);
    nh_.param("kd_force_y", kd_force_y_, 0.1);

    nh_.param("kp_force_z", kp_force_z_, 1.0);
    nh_.param("ki_force_z", ki_force_z_, 0.01);
    nh_.param("kd_force_z", kd_force_z_, 0.1);

    // Load PID gains for torque
    nh_.param("kp_torque_x", kp_torque_x_, 0.5);
    nh_.param("ki_torque_x", ki_torque_x_, 0.005);
    nh_.param("kd_torque_x", kd_torque_x_, 0.05);

    nh_.param("kp_torque_y", kp_torque_y_, 0.5);
    nh_.param("ki_torque_y", ki_torque_y_, 0.005);
    nh_.param("kd_torque_y", kd_torque_y_, 0.05);

    nh_.param("kp_torque_z", kp_torque_z_, 0.5);
    nh_.param("ki_torque_z", ki_torque_z_, 0.005);
    nh_.param("kd_torque_z", kd_torque_z_, 0.05);

    // Initialize PID controllers
    pid_force_x_.initPid(kp_force_x_, ki_force_x_, kd_force_x_, max_force_x_, -max_force_x_);
    pid_force_y_.initPid(kp_force_y_, ki_force_y_, kd_force_y_, max_force_y_, -max_force_y_);
    pid_force_z_.initPid(kp_force_z_, ki_force_z_, kd_force_z_, max_force_z_, -max_force_z_);

    pid_torque_x_.initPid(kp_torque_x_, ki_torque_x_, kd_torque_x_, max_torque_x_, -max_torque_x_);
    pid_torque_y_.initPid(kp_torque_y_, ki_torque_y_, kd_torque_y_, max_torque_y_, -max_torque_y_);
    pid_torque_z_.initPid(kp_torque_z_, ki_torque_z_, kd_torque_z_, max_torque_z_, -max_torque_z_);
}

bool WaypointFollower::loadWaypoints() {
    XmlRpc::XmlRpcValue waypoints_list;
    if (!nh_.getParam("waypoints", waypoints_list)) {
        ROS_ERROR("Failed to load waypoints from parameter server");
        return false;
    }

    if (waypoints_list.getType() != XmlRpc::XmlRpcValue::TypeArray) {
        ROS_ERROR("Waypoints parameter is not an array");
        return false;
    }

    waypoints_.clear();
    for (int i = 0; i < waypoints_list.size(); ++i) {
        if (waypoints_list[i].getType() != XmlRpc::XmlRpcValue::TypeArray ||
            waypoints_list[i].size() != 7) {  // x, y, z, qx, qy, qz, qw
            ROS_ERROR_STREAM("Invalid waypoint format at index " << i);
            continue;
        }

        geometry_msgs::Pose pose;
        pose.position.x = static_cast<double>(waypoints_list[i][0]);
        pose.position.y = static_cast<double>(waypoints_list[i][1]);
        pose.position.z = static_cast<double>(waypoints_list[i][2]);
        pose.orientation.x = static_cast<double>(waypoints_list[i][3]);
        pose.orientation.y = static_cast<double>(waypoints_list[i][4]);
        pose.orientation.z = static_cast<double>(waypoints_list[i][5]);
        pose.orientation.w = static_cast<double>(waypoints_list[i][6]);

        waypoints_.push_back(pose);
    }

    return !waypoints_.empty();
}

void WaypointFollower::odometryCallback(const nav_msgs::Odometry::ConstPtr& msg) {
    current_odom_ = *msg;
    odom_received_ = true;
}

geometry_msgs::Wrench WaypointFollower::computeCommand() {
    geometry_msgs::Wrench cmd;

    if (waypoints_.empty() || current_waypoint_ >= waypoints_.size()) {
        return cmd;
    }

    // Compute position errors
    double error_x = waypoints_[current_waypoint_].position.x - current_odom_.pose.pose.position.x;
    double error_y = waypoints_[current_waypoint_].position.y - current_odom_.pose.pose.position.y;
    double error_z = waypoints_[current_waypoint_].position.z - current_odom_.pose.pose.position.z;

    // Convert quaternions to RPY for both current pose and target
    tf2::Quaternion q_current(
        current_odom_.pose.pose.orientation.x,
        current_odom_.pose.pose.orientation.y,
        current_odom_.pose.pose.orientation.z,
        current_odom_.pose.pose.orientation.w);
    
    tf2::Quaternion q_target(
        waypoints_[current_waypoint_].orientation.x,
        waypoints_[current_waypoint_].orientation.y,
        waypoints_[current_waypoint_].orientation.z,
        waypoints_[current_waypoint_].orientation.w);

    double roll_current, pitch_current, yaw_current;
    double roll_target, pitch_target, yaw_target;
    
    tf2::Matrix3x3(q_current).getRPY(roll_current, pitch_current, yaw_current);
    tf2::Matrix3x3(q_target).getRPY(roll_target, pitch_target, yaw_target);

    // Compute orientation errors
    double error_roll = roll_target - roll_current;
    double error_pitch = pitch_target - pitch_current;
    double error_yaw = yaw_target - yaw_current;

    ros::Duration dt = ros::Time::now() - last_command_time_;
    last_command_time_ = ros::Time::now();

    // Compute forces using PID controllers
    double force_x = pid_force_x_.computeCommand(error_x, dt);
    double force_y = pid_force_y_.computeCommand(error_y, dt);
    double force_z = pid_force_z_.computeCommand(error_z, dt);

    // Compute torques using PID controllers
    double torque_x = pid_torque_x_.computeCommand(error_roll, dt);
    double torque_y = pid_torque_y_.computeCommand(error_pitch, dt);
    double torque_z = pid_torque_z_.computeCommand(error_yaw, dt);

    // Apply force limits
    cmd.force.x = std::max(-max_force_x_, std::min(max_force_x_, force_x));
    cmd.force.y = std::max(-max_force_y_, std::min(max_force_y_, force_y));
    cmd.force.z = std::max(-max_force_z_, std::min(max_force_z_, force_z));

    // Apply torque limits
    cmd.torque.x = std::max(-max_torque_x_, std::min(max_torque_x_, torque_x));
    cmd.torque.y = std::max(-max_torque_y_, std::min(max_torque_y_, torque_y));
    cmd.torque.z = std::max(-max_torque_z_, std::min(max_torque_z_, torque_z));

    return cmd;
}

void WaypointFollower::run() {
    ros::Rate rate(publish_rate_);

    while (ros::ok()) {
        if (odom_received_) {
            geometry_msgs::Wrench cmd = computeCommand();
            wrench_pub_.publish(cmd);
        }

        ros::spinOnce();
        rate.sleep(); 
    }
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "waypoint_follower_node");
    ros::NodeHandle nh;

    try {
        WaypointFollower follower(nh);
        follower.run();
    }
    catch (const std::exception& e) {
        ROS_ERROR_STREAM("Exception in waypoint follower node: " << e.what());
        return 1;
    }

    return 0;
}