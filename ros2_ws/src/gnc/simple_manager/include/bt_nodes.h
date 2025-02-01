#ifndef BUR_NODES
#define BUR_NODES

#include <chrono>
#include <memory>

#include "behaviortree_cpp/action_node.h"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/quaternion.hpp"

#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"


#include "manager_node.h"

class GoToTarget : public BT::StatefulActionNode
{
    public:
        GoToTarget(const std::string& name, const BT::NodeConfiguration& config,
                   const std::shared_ptr<SimpleManager> ptr,
                   const int target_id):
            BT::StatefulActionNode(name, config),
            node_(ptr),
            target_id_(target_id_) {}

        static BT::PortsList providedPorts() { return {}; }

        BT::NodeStatus onStart() override   { return this->navigateToTarget(); }
        BT::NodeStatus onRunning() override { return this->navigateToTarget(); }

        void onHalted() override {}
    
    private:
        BT::NodeStatus navigateToTarget();

        std::shared_ptr<SimpleManager> node_;
        int target_id_;
        double thresh_dist_ = 1.0;
};


class FireTorpedo : public BT::SyncActionNode
{
    public:
        FireTorpedo(const std::string& name, const BT::NodeConfiguration& config,
                          const std::shared_ptr<SimpleManager> ptr):
            BT::SyncActionNode(name, config),
            node_(ptr) {}

        static BT::PortsList providedPorts() { return {}; }

        BT::NodeStatus tick() override;
    
    private:
        std::shared_ptr<SimpleManager> node_;
        std::string pub_topic_;
};

// Unused
class DriveForDuration : public BT::StatefulActionNode
{
    public:
        DriveForDuration(const std::string& name, const BT::NodeConfiguration& config,
                        const std::shared_ptr<SimpleManager> ptr,
                        const sensor_msgs::msg::Joy joy_msg,
                        const float duration):
            BT::StatefulActionNode(name, config),
            node_(ptr),
            joy_msg_(joy_msg) {
                begin_ = std::chrono::steady_clock::now();
            }

        static BT::PortsList providedPorts() { return {}; }

        BT::NodeStatus onStart() override   { return this->publish_joy(); }
        BT::NodeStatus onRunning() override { return this->publish_joy(); }

        void onHalted() override {}

        void setDuration(float duration) { this->duration_ = duration; }

    
    private:
        std::shared_ptr<SimpleManager> node_;
        float duration_;

        std::chrono::steady_clock::time_point begin_;

        sensor_msgs::msg::Joy joy_msg_;
        BT::NodeStatus publish_joy();
};


class SaveCurrentPoseToBlackboard : public BT::SyncActionNode
{
    public:
        SaveCurrentPoseToBlackboard(const std::string& name, const BT::NodeConfiguration& config,
                            const std::shared_ptr<SimpleManager> ptr, const std::string port):
            BT::SyncActionNode(name, config),
            node_(ptr) {}
        
        static BT::PortsList providedPorts() {
            return { BT::OutputPort<geometry_msgs::msg::Pose>("output") };
        }

        BT::NodeStatus tick() override {
            setOutput("output", this->node_->get_current_position());
            return BT::NodeStatus::SUCCESS;
        }
    
    private:
        std::shared_ptr<SimpleManager> node_;
        std::string port_;
};

#endif
