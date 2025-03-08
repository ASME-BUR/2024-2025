#include "manager_node.h"
#include "bt_nodes.h"

#include <chrono>
#include <cmath>
#include <iostream>
#include <thread>

#include "nav2_util/geometry_utils.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "tf2/LinearMath/Matrix3x3.h"
#include <tf2/utils.h>

BT::NodeStatus BTNavigationNode::navigateToTarget() {
    geometry_msgs::msg::Pose target = this->getTargetPosition();
    double dist = nav2_util::geometry_utils::euclidean_distance(target,
        this->node_->get_current_position());
    if(dist > this->thresh_dist_) {
        this->node_->set_goal_pose(target);
        return BT::NodeStatus::RUNNING;
    }
    return BT::NodeStatus::SUCCESS;
}

geometry_msgs::msg::Pose GoToTarget::getTargetPosition() {
    VisionTarget target;
    auto targets = this->getNode()->vision_targets_;
    for(int i = 0; i < targets.size(); i++) {
        if(targets[i].id_ == this->target_id_) {
            target = targets[i];
            break;
        }
    }
    return target.p_;
}


BT::NodeStatus FireTorpedo::tick() {
    sensor_msgs::msg::Joy msg;

    auto values = std::vector<double>(9, 0);
    auto buttons = std::vector<int>(10, 0);
    
    values[2] = -0.7;
    values[0] = 1.0;
    buttons[9] = 1;

    for (size_t i = 0; i < values.size(); i++) {
            msg.axes.push_back(values[i]);
    }
    for (size_t i = 0; i < buttons.size(); i++) {
            msg.buttons.push_back(buttons[i]);
    }

    this->node_->publish_joy_msg(msg);

    return BT::NodeStatus::SUCCESS;
}
