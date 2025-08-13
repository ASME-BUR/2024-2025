#include <chrono>
#include <functional>
#include <memory>
#include <vector>

#include "behaviortree_cpp/action_node.h"
#include "behaviortree_cpp/behavior_tree.h"
#include "behaviortree_cpp/bt_factory.h"

#include "rclcpp/rclcpp.hpp"

#include "sensor_msgs/msg/joy.hpp"


class SimpleManager : public rclcpp::Node
{
    public:
        SimpleManager() : rclcpp::Node::Node("simple_manager") {

            this->declare_parameter("joy_topic", "/joy");

            this->declare_parameter("behavior_tree", "tree.xml");
            this->declare_parameter("tick_rate", 10);

            this->declare_parameter("auto_shutdown", true);
            this->declare_parameter("wait_for_depth", false);

            int tick_rate = this->get_parameter("tick_rate").as_int();

            joy_pub_ = this->create_publisher<sensor_msgs::msg::Joy>(
                this->get_parameter("joy_topic").as_string(), 10);

            btTimer_ = this->create_wall_timer(
                std::chrono::milliseconds(1000 / tick_rate), 
                std::bind(&SimpleManager::tick_behavior, this));

        }

        void initialize_tree(BT::BehaviorTreeFactory &factory) {
            factory.registerBehaviorTreeFromFile(this->get_parameter("behavior_tree").as_string());
            this->behavior_tree_ = factory.createTree("MainTree");
            this->tree_initialized_ = true;
        }

        void publish_joy_msg(sensor_msgs::msg::Joy msg) {
            this->joy_pub_->publish(msg);
        }

    private:
    
        BT::Tree behavior_tree_;
        bool tree_initialized_ {false};
        
        rclcpp::Publisher<sensor_msgs::msg::Joy>::SharedPtr joy_pub_;
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
                BT::InputPort<int>("enable"),
                BT::InputPort<double>("duration"),
                BT::InputPort<double>("linear_x"),
                BT::InputPort<double>("linear_y"),
                BT::InputPort<double>("linear_z"),
                BT::InputPort<double>("angular_x"),
                BT::InputPort<double>("angular_y"),
                BT::InputPort<double>("angular_z")
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
            this->msg_ = sensor_msgs::msg::Joy();
            this->msg_.axes.clear();
            this->msg_.buttons.clear();

            std::vector<double> values(9, 0);
            std::vector<int> buttons(13, 0);

            this->duration_ = getInput<double>("duration") ? getInput<double>("duration").value() : 0;
            buttons[9] = getInput<int>("enable") ? getInput<int>("enable").value() : 0;
            values[1] = getInput<double>("linear_x") ? getInput<double>("linear_x").value() : 0;
            values[0] = getInput<double>("linear_y") ? getInput<double>("linear_y").value() : 0;
            values[4] = getInput<double>("linear_z") ? getInput<double>("linear_z").value() : 0;
            values[6] = getInput<double>("angular_x") ? getInput<double>("angular_x").value() : 0;
            values[7] = getInput<double>("angular_y") ? getInput<double>("angular_y").value() : 0;
            values[3] = getInput<double>("angular_z") ? getInput<double>("angular_z").value() : 0;

            for (size_t i = 0; i < values.size(); i++)
            {
                this->msg_.axes.push_back(values[i]);
            }
            for (size_t i = 0; i < buttons.size(); i++)
            {
                this->msg_.buttons.push_back(buttons[i]);
            }
        }

        BT::NodeStatus checkTimer() {
            using namespace std;

            auto end = chrono::steady_clock::now();
            if(chrono::duration_cast<chrono::seconds>(end-this->begin_).count() > this->duration_) {
                return BT::NodeStatus::SUCCESS;
            }
            this->node_->publish_joy_msg(this->msg_);
            return BT::NodeStatus::RUNNING;
        }

        const std::shared_ptr<SimpleManager> node_;
        sensor_msgs::msg::Joy msg_;

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