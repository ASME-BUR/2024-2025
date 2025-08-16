#include <chrono>
#include <functional>
#include <memory>
#include <vector>

#include "behaviortree_cpp/action_node.h"
#include "behaviortree_cpp/behavior_tree.h"
#include "behaviortree_cpp/bt_factory.h"

#include "rclcpp/rclcpp.hpp"

#include "geometry_msgs/msg/wrench_stamped.hpp"


class SimpleManager : public rclcpp::Node
{
    public:
        SimpleManager() : rclcpp::Node::Node("simple_manager") {

            this->declare_parameter("wrench_topic", "/control_effort");

            this->declare_parameter("behavior_tree", "tree.xml");
            this->declare_parameter("tick_rate", 10);

            this->declare_parameter("auto_shutdown", true);
            this->declare_parameter("wait_for_depth", false);

            int tick_rate = this->get_parameter("tick_rate").as_int();

            wrench_pub_ = this->create_publisher<geometry_msgs::msg::WrenchStamped>(
                this->get_parameter("wrench_topic").as_string(), 10);

            btTimer_ = this->create_wall_timer(
                std::chrono::milliseconds(1000 / tick_rate), 
                std::bind(&SimpleManager::tick_behavior, this));

        }

        void initialize_tree(BT::BehaviorTreeFactory &factory) {
            factory.registerBehaviorTreeFromFile(this->get_parameter("behavior_tree").as_string());
            this->behavior_tree_ = factory.createTree("MainTree");
            this->tree_initialized_ = true;
        }

        void publish_wrench_msg(geometry_msgs::msg::WrenchStamped msg) {
            this->wrench_pub_->publish(msg);
        }

    private:
    
        BT::Tree behavior_tree_;
        bool tree_initialized_ {false};
        
        rclcpp::Publisher<geometry_msgs::msg::WrenchStamped>::SharedPtr wrench_pub_;
        rclcpp::TimerBase::SharedPtr btTimer_;

        void tick_behavior() {
            if (!this->tree_initialized_) return;
            BT::NodeStatus status = this->behavior_tree_.tickOnce();
            if(status == BT::NodeStatus::SUCCESS &&
                this->get_parameter("auto_shutdown").as_bool()) {
                rclcpp::shutdown();
            } else if (status == BT::NodeStatus::FAILURE) {
                RCLCPP_INFO(this->get_logger(), "Tree failed");
                rclcpp::shutdown();
            }
        }

};


class TimedJoyNode : public BT::StatefulActionNode
{
    public:
        TimedJoyNode(
                const std::string& name, const BT::NodeConfiguration& config,
                const std::shared_ptr<SimpleManager> node):
            BT::StatefulActionNode(name, config), node_(node) {}

        static BT::PortsList providedPorts() {
            return { 
                BT::InputPort<double>("duration"),
                BT::InputPort<double>("force_x"),
                BT::InputPort<double>("force_y"),
                BT::InputPort<double>("force_z"),
                BT::InputPort<double>("torque_x"),
                BT::InputPort<double>("torque_y"),
                BT::InputPort<double>("torque_z")
            };
        }

        BT::NodeStatus onStart() override   {
            this->initializeParams();
            begin_ = std::chrono::steady_clock::now();
            return this->checkTimer();
            
        }
        BT::NodeStatus onRunning() override { return this->checkTimer(); }
        void onHalted() override {}
    
    private:
        void initializeParams() {
            /*
                0: left joystick left/right (left/right)
                1: left joystick up/down (forward/back)
                3: right joystick left/right (yaw left/right)
                4: right joystick up/down (robot up/down)
            */
            this->msg_ = geometry_msgs::msg::WrenchStamped();

            this->duration_ = getInput<double>("duration") ? getInput<double>("duration").value() : 0;
            this->msg_.wrench.force.x = getInput<double>("force_x") ? getInput<double>("force_x").value() : 0;
            this->msg_.wrench.force.y = getInput<double>("force_y") ? getInput<double>("force_y").value() : 0;
            this->msg_.wrench.force.z = getInput<double>("force_z") ? getInput<double>("force_z").value() : 0;
            this->msg_.wrench.torque.x = getInput<double>("torque_x") ? getInput<double>("torque_x").value() : 0;
            this->msg_.wrench.torque.y = getInput<double>("torque_y") ? getInput<double>("torque_y").value() : 0;
            this->msg_.wrench.torque.z = getInput<double>("torque_z") ? getInput<double>("torque_z").value() : 0;
        }

        BT::NodeStatus checkTimer() {
            using namespace std;

            auto end = chrono::steady_clock::now();
            if(chrono::duration_cast<chrono::seconds>(end-this->begin_).count() > this->duration_) {
                return BT::NodeStatus::SUCCESS;
            }
            this->node_->publish_wrench_msg(this->msg_);
            return BT::NodeStatus::RUNNING;
        }

        const std::shared_ptr<SimpleManager> node_;
        geometry_msgs::msg::WrenchStamped msg_;

        std::chrono::steady_clock::time_point begin_;
        double duration_;
};


int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);

    BT::BehaviorTreeFactory factory;
    auto manager = std::make_shared<SimpleManager>();

    factory.registerNodeType<TimedJoyNode>("TimedJoyNode", manager);

    manager->initialize_tree(factory);
    rclcpp::spin(manager);

    return 0;

}