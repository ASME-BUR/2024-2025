#ifndef SIMPLE_MANAGER
#define SIMPLE_MANAGER

#include <memory>

#include "behaviortree_cpp/behavior_tree.h"
#include "behaviortree_cpp/bt_factory.h"

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/joy.hpp"

#include "std_msgs/msg/float32_multi_array.hpp"

#include "bur_msgs/msg/cv_detection.hpp"
#include "bur_msgs/msg/cv_detections.hpp"

#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"

#include "rclcpp/rclcpp.hpp"

const int YOLO_GATE = 0;
const int YOLO_BUOY = 1;

const int ZED_WIDTH = 1920;
const int ZED_HEIGHT = 1080;

struct VisionTarget
{
    public:
        VisionTarget() {};
        VisionTarget(int target_id) : id_(target_id) {};
        VisionTarget(int target_id, geometry_msgs::msg::Pose p) : 
            id_(target_id), p_(p) {};

        void updatePose(geometry_msgs::msg::Pose p) { this->p_ = p; }
        // void updateVelocity(geometry_msgs::msg::Twist vel) { this->vel_ = vel;}

        geometry_msgs::msg::Pose p_;
        // geometry_msgs::msg::Twist vel_;

        int id_;
        bool detected_ = false;
};

class SimpleManager : public rclcpp::Node
{
    public:
        SimpleManager();

        void initialize_tree(BT::BehaviorTreeFactory &factory);
        void initialize_targets();

        geometry_msgs::msg::Pose const get_current_position() { return current_pos_; }
        void set_current_position(geometry_msgs::msg::Pose p) { this->current_pos_ = p; }

        geometry_msgs::msg::Pose get_goal_pose() { return this->goal_pose_; }
        void set_goal_pose(geometry_msgs::msg::Pose target_pos) { this->goal_pose_ = target_pos; }

        void publish_joy_msg(sensor_msgs::msg::Joy joy_msg) {  this->joy_pub_->publish(joy_msg); }

        geometry_msgs::msg::Pose start_position_;

        std::vector<VisionTarget> vision_targets_;
        std::vector<bur_msgs::msg::CVDetection> detected_;

    private:
    
        BT::Tree behavior_tree_;

        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr localizer_sub_;
        rclcpp::Subscription<bur_msgs::msg::CVDetections>::SharedPtr vision_sub_;
        rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr depth_sub_;
        
        rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pose_pub_;
        rclcpp::Publisher<sensor_msgs::msg::Joy>::SharedPtr joy_pub_;
        rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr obstacle_pub_;

        geometry_msgs::msg::Pose current_pos_;
        geometry_msgs::msg::Twist current_vel_;
        geometry_msgs::msg::TransformStamped baselink2Odom;

        geometry_msgs::msg::Pose goal_pose_;

        std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
        std::unique_ptr<tf2_ros::Buffer> tf_buffer_;

        rclcpp::TimerBase::SharedPtr pubTimer_;
        rclcpp::TimerBase::SharedPtr btTimer_;

        double depth = 1000;

        void tick_behavior();

        // ROS Callbacks
        void publish_goal_pose();
        void localizer_callback(const nav_msgs::msg::Odometry::SharedPtr msg);
        void vision_callback(const bur_msgs::msg::CVDetections::SharedPtr msg);
        void depth_callback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) {
            this->depth = msg->pose.pose.position.z;
        }
};

#endif
