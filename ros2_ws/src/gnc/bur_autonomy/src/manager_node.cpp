#include "manager_node.h"
#include "bt_nodes.h"

#include "tf2/exceptions.h"
#include "rclcpp/rclcpp.hpp"


SimpleManager::SimpleManager() : rclcpp::Node::Node("simple_manager")
{
    this->declare_parameter("localizer_topic", "/odometry/filtered");
    this->declare_parameter("vision_topic", "/vision");

    this->declare_parameter("goal_topic", "/des_pose");
    this->declare_parameter("joy_topic", "/joy");
    this->declare_parameter("pub_rate", 10);

    this->declare_parameter("behavior_tree", "tree.xml");
    this->declare_parameter("tick_rate", 10);

    this->declare_parameter("shutdown_on_end", true);
    this->declare_parameter("wait_for_depth", false);

    int pub_rate = this->get_parameter("pub_rate").as_int();
    int tick_rate = this->get_parameter("tick_rate").as_int();

    localizer_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        this->get_parameter("localizer_topic").as_string(), 
        10, std::bind(&SimpleManager::localizer_callback, this, std::placeholders::_1));
    vision_sub_ = this->create_subscription<bur_msgs::msg::CVDetections>(
        this->get_parameter("vision_topic").as_string(),
        10, std::bind(&SimpleManager::vision_callback, this, std::placeholders::_1));

    goal_pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
        this->get_parameter("goal_topic").as_string(), 10);
    joy_pub_ = this->create_publisher<sensor_msgs::msg::Joy>(
        this->get_parameter("joy_topic").as_string(), 10);
    obstacle_pub_ = this->create_publisher<std_msgs::msg::Float32MultiArray>(
        "/obstacles", 10);

    pubTimer_ = this->create_wall_timer(
        std::chrono::milliseconds(1000 / pub_rate), 
        std::bind(&SimpleManager::publish_goal_pose, this));
    btTimer_ = this->create_wall_timer(
        std::chrono::milliseconds(1000 / tick_rate), 
        std::bind(&SimpleManager::tick_behavior, this));

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
}

void SimpleManager::initialize_targets() {
    this->declare_parameter("start_position.x", 0.0);
    this->declare_parameter("start_position.y", 0.0);
    this->declare_parameter("start_position.z", 0.0);

    this->declare_parameter("gate_position.x", 20.0);
    this->declare_parameter("gate_position.y", 0.0);
    this->declare_parameter("gate_position.z", 0.0);

    this->declare_parameter("buoy_position.x", 10.0);
    this->declare_parameter("buoy_position.y", 10.0);
    this->declare_parameter("buoy_position.z", 0.0);

    geometry_msgs::msg::Pose start_pose;
    start_pose.position.x = this->get_parameter("start_position.x").as_double();
    start_pose.position.y = this->get_parameter("start_position.y").as_double();
    start_pose.position.z = this->get_parameter("start_position.z").as_double();
    this->set_current_position(start_pose);

    VisionTarget gate_target(YOLO_GATE);
    gate_target.p_.position.x = this->get_parameter("gate_position.x").as_double();
    gate_target.p_.position.y = this->get_parameter("gate_position.y").as_double();
    gate_target.p_.position.z = this->get_parameter("gate_position.z").as_double();   

    VisionTarget buoy_target(YOLO_BUOY);
    buoy_target.p_.position.x = this->get_parameter("buoy_position.x").as_double();
    buoy_target.p_.position.y = this->get_parameter("buoy_position.y").as_double();
    buoy_target.p_.position.z = this->get_parameter("buoy_position.z").as_double();   

    this->vision_targets_.push_back(gate_target);
    this->vision_targets_.push_back(buoy_target);
}

void SimpleManager::initialize_tree(BT::BehaviorTreeFactory &factory) {
    factory.registerBehaviorTreeFromFile(this->get_parameter("behavior_tree").as_string());
    this->behavior_tree_ = factory.createTree("MainTree");
}

void SimpleManager::localizer_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    // try {
    //     this->baselink2Odom = tf_buffer_->lookupTransform("odom", "base_link", tf2::TimePointZero);
    // } catch (const tf2::TransformException& ex) {
    //     RCLCPP_INFO(this->get_logger(), "Could no transform base_link to odom: %s", ex.what());
    // }

    this->current_pos_ = msg->pose.pose;
    this->current_vel_ = msg->twist.twist;
}

void SimpleManager::vision_callback(const bur_msgs::msg::CVDetections::SharedPtr msg) {
    this->detected_ = msg->detected;
}

void SimpleManager::publish_goal_pose() {
    geometry_msgs::msg::PoseStamped msg;
    msg.pose = this->goal_pose_;
    this->goal_pose_pub_->publish(msg);

    std_msgs::msg::Float32MultiArray obstacle_msg;
    obstacle_msg.layout.dim.push_back(std_msgs::msg::MultiArrayDimension());
    obstacle_msg.layout.dim.push_back(std_msgs::msg::MultiArrayDimension());
    obstacle_msg.layout.dim[0].label = "height";
    obstacle_msg.layout.dim[0].size = 1;
    obstacle_msg.layout.dim[0].stride = 1*5;
    obstacle_msg.layout.dim[1].label = "width";
    obstacle_msg.layout.dim[1].size = 5;
    obstacle_msg.layout.dim[1].stride = 5;
    obstacle_msg.layout.data_offset = 0;

    std::vector<float> vec = { 1, 1, 1, 0.5, 0.5 };
    obstacle_msg.data = vec;
    // this->obstacle_pub_->publish(obstacle_msg);
}

void SimpleManager::tick_behavior() {
    if(depth > 0 && this->get_parameter("wait_for_depth").as_bool()) { return; }
    BT::NodeStatus status = this->behavior_tree_.tickOnce();

    if(status == BT::NodeStatus::SUCCESS &&
        this->get_parameter("shutdown_on_end").as_bool()) {
        rclcpp::shutdown();
    } else if (status == BT::NodeStatus::FAILURE) {
        RCLCPP_INFO(this->get_logger(), "Tree failed");
        rclcpp::shutdown();
    }
}

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);

    BT::BehaviorTreeFactory factory;

    auto manager = std::make_shared<SimpleManager>();

    factory.registerNodeType<GoToTarget>("GoToGate", manager, YOLO_GATE);
    factory.registerNodeType<GoToPose>("GoToStart", manager, std::make_shared<geometry_msgs::msg::Pose>(manager->start_position_));

    factory.registerNodeType<FireTorpedo>("FireTorpedo", manager);

    manager->initialize_tree(factory);
    manager->initialize_targets();

    rclcpp::spin(manager);

    return 0;
}
