#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <vector>
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "nav_msgs/msg/odometry.hpp"
using namespace std::chrono_literals;


class MinimalPublisher : public rclcpp::Node
{
public:
    MinimalPublisher()
        : Node("minimal_publisher"), count_(0)
    {
        publisher_ = this->create_publisher<sensor_msgs::msg::Joy>("joy", 1);
        publisher2_ = this->create_publisher<nav_msgs::msg::Odometry>("next_waypoint", 1);
        depth_sub = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>("depth_sensor", 10, bind(&MinimalPublisher::depth_callback, this, std::placeholders::_1));
        timer_ = this->create_wall_timer(
            100ms, std::bind(&MinimalPublisher::timer_callback, this));
        init = this->now();
    }

private:
    void timer_callback()
    {

        /*
            0: left joystick left/right (left/right)
            1: left joystick up/down (forward/back)
            3: right joystick left/right (yaw left/right)
            4: right joystick up/down (robot up/down)
        */

        // if (depth < 0)
        // {
            auto message = sensor_msgs::msg::Joy();
            des_pose.pose.pose.position.z = -1.5;
            publisher2_->publish(des_pose);
            std::vector<double> values(9, 0);
            message.axes.clear();
            rclcpp::Duration diff = this->now() - init;
            auto num_of_s = diff.to_chrono<std::chrono::seconds>();
            double delta_t = num_of_s.count();
            message.header.stamp.sec = delta_t;
            message.buttons.clear();
            std::vector<int> buttons(13, 0);
            float start_time = 5.0;
            float t1 = 5;
            float t2 = 13;
            float t3 = 23;
            float t4 = 15;
            float t5 = 15;
            float t6 = 5;

            // Descend
            if (delta_t < start_time + t1 && delta_t >= start_time)
            {
                values[1] = 1.0;
                values[4] = 0.05;
                buttons[9] = 1;
            }
            // go forward
            else if (delta_t < start_time + t1 + t2 && delta_t >= start_time + t1)
            {
                values[1] = 1.0;
                values[4] = -0.15;
                buttons[9] = 1;
            }            
            // yaw
            else if (delta_t < start_time + t1 + t2 + t3 && delta_t >= start_time + t1 + t2)
            {
                values[3] = -0.5;
                values[4] = -0.05;
                buttons[9] = 1;
            }
            // roll and fire torpedo
            else if (delta_t < start_time + t1 + t2 + t3 + t4 && delta_t >= start_time + t1 + t2 + t3)
            {
                // values[3] = 5.0;
                buttons[9] = 1;
                buttons[4] = 1.0;
            }

            for (size_t i = 0; i < values.size(); i++)
            {
                message.axes.push_back(values[i]);
            }
            for (size_t i = 0; i < buttons.size(); i++)
            {
                message.buttons.push_back(buttons[i]);
            }
            publisher_->publish(message);
        // }
        // else
        // {
        //     init = this->now();
        // }
    }

    void depth_callback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
    {
        depth = msg->pose.pose.position.z;
    }

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<sensor_msgs::msg::Joy>::SharedPtr publisher_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr publisher2_;
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr depth_sub;
    size_t count_;
    double depth = 1000;
    rclcpp::Time init;
    nav_msgs::msg::Odometry des_pose;

};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MinimalPublisher>());
    rclcpp::shutdown();
    return 0;
}