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

class BTNavigationNode : public BT::StatefulActionNode
{
    public:
        BTNavigationNode(const std::string& name, const BT::NodeConfiguration& config,
                   const std::shared_ptr<SimpleManager> ptr):
            BT::StatefulActionNode(name, config),
            node_(ptr) {}

        static BT::PortsList providedPorts() { return {}; }

        BT::NodeStatus onStart() override   { return this->navigateToTarget(); }
        BT::NodeStatus onRunning() override { return this->navigateToTarget(); }
        void onHalted() override {}

        std::shared_ptr<SimpleManager> getNode() { return this->node_; }
    
    private:
        virtual geometry_msgs::msg::Pose getTargetPosition() = 0;
        BT::NodeStatus navigateToTarget();

        std::shared_ptr<SimpleManager> node_;
        double thresh_dist_ = 1.0;
};

class GoToPose : public BTNavigationNode
{
    public:
        GoToPose(const std::string& name, const BT::NodeConfiguration& config,
                   const std::shared_ptr<SimpleManager> ptr,
                   const std::shared_ptr<geometry_msgs::msg::Pose> pose):
            BTNavigationNode(name, config, ptr),
            target_(pose) {}

    private:
        geometry_msgs::msg::Pose getTargetPosition() { return *target_; }
        std::shared_ptr<geometry_msgs::msg::Pose> target_;
};


class GoToTarget : public BTNavigationNode
{
    public:
        GoToTarget(const std::string& name, const BT::NodeConfiguration& config,
                   const std::shared_ptr<SimpleManager> ptr,
                   const int target_id):
            BTNavigationNode(name, config, ptr),
            target_id_(target_id) {}

    private:
        geometry_msgs::msg::Pose getTargetPosition();
        int target_id_;
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
