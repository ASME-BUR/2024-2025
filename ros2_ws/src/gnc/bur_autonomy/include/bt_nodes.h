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

class GoToPose : public BT::StatefulActionNode
{
    public:
        GoToPose(const std::string& name, const BT::NodeConfiguration& config,
                   const std::shared_ptr<SimpleManager> ptr,
                   const std::shared_ptr<geometry_msgs::msg::Pose> pose):
            BT::StatefulActionNode(name, config),
            node_(ptr),
            target_(pose) {}

        static BT::PortsList providedPorts() { return {}; }

        BT::NodeStatus onStart() override   { return this->navigateToTarget(); }
        BT::NodeStatus onRunning() override { return this->navigateToTarget(); }

        void onHalted() override {}
    
    private:
        BT::NodeStatus navigateToTarget();
        std::shared_ptr<SimpleManager> node_;
        std::shared_ptr<geometry_msgs::msg::Pose> target_;
        double thresh_dist_ = 1.0;
};


class GoToTarget : public BT::StatefulActionNode
{
    public:
        GoToTarget(const std::string& name, const BT::NodeConfiguration& config,
                   const std::shared_ptr<SimpleManager> ptr,
                   const int target_id):
            BT::StatefulActionNode(name, config),
            node_(ptr),
            target_id_(target_id) {}

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

#endif
