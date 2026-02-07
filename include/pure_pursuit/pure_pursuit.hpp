#ifndef PURE_PURSUIT_HPP
#define PURE_PURSUIT_HPP

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/path.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"

namespace pure_pursuit{

    class PurePursuit : public rclcpp::Node {

        public:
            PurePursuit();
        
        private:

            rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
            rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_pub_;
            rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr look_ahead_pub_;
            rclcpp::TimerBase::SharedPtr control_loop_;

            std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
            std::unique_ptr<tf2_ros::Buffer> tf_buffer_;

            //data
            nav_msgs::msg::Path global_plan_;
            double look_ahead_dist_;
            double max_linear_vel_;
            double max_angular_vel_;

            void control_loop();
            void pathcallback(const nav_msgs::msg::Path::SharedPtr path);
            bool transform_plan(const std::string &frame);
            geometry_msgs::msg::PoseStamped get_look_ahead_pose(const geometry_msgs::msg::PoseStamped &bot_pose);
            double get_curvature(const geometry_msgs::msg::Pose &la_pose);
    };
}

#endif