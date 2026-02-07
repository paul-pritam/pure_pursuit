#include "../include/pure_pursuit/pure_pursuit.hpp"

#include <chrono>
#include <algorithm>

#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace pure_pursuit{

    PurePursuit::PurePursuit() : Node ("pure_pursuit_node"),
    look_ahead_dist_(0.5), max_linear_vel_(0.3),max_angular_vel_(1.0){

        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        declare_parameter<double>("look_ahead_dist",look_ahead_dist_);
        declare_parameter<double>("max_linear_vel",max_linear_vel_);
        declare_parameter<double>("max_angular_vel",max_angular_vel_);

        look_ahead_dist_ = get_parameter("look_ahead_dist").as_double();
        max_linear_vel_ = get_parameter("max_linear_vel").as_double();
        max_angular_vel_ = get_parameter("max_angular_vel").as_double();

        rclcpp::QoS path_qos(10);
        path_qos.transient_local();
        path_qos.reliable();

        path_sub_ = this->create_subscription<nav_msgs::msg::Path>("/astar/path", path_qos, std::bind(&PurePursuit::pathcallback,this,std::placeholders::_1));
        vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", path_qos);
        look_ahead_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/pure_pursuit/look_ahead_pose",path_qos);

        control_loop_ = create_wall_timer(std::chrono::milliseconds(100),std::bind(&PurePursuit::control_loop,this));
        
    }

    void PurePursuit::pathcallback(const nav_msgs::msg::Path::SharedPtr path){
        global_plan_ = *path;
    }

    void PurePursuit::control_loop(){

        if (global_plan_.poses.empty()){
            RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000, "Waiting for path...");
            return;
        }

        // current pose in /odom
        geometry_msgs::msg::TransformStamped bot_pose_tf;
        try{
            bot_pose_tf = tf_buffer_->lookupTransform("odom","base_link",tf2::TimePointZero);

        }catch(tf2::TransformException &ex){
            RCLCPP_WARN(get_logger(), "couldnt get transform: %s", ex.what());
            return;
        }

        if (!transform_plan(bot_pose_tf.header.frame_id)){
            RCLCPP_ERROR(get_logger(), "unable to transform global path plan in the bot_pose_tf");
            return;
        }
        
        //convert tranform_stamped to pose_stamped
        geometry_msgs::msg::PoseStamped bot_pose_stamped;
        bot_pose_stamped.pose.position.x = bot_pose_tf.transform.translation.x;
        bot_pose_stamped.pose.position.y = bot_pose_tf.transform.translation.y;
        bot_pose_stamped.pose.position.z = bot_pose_tf.transform.translation.z;
        bot_pose_stamped.pose.orientation = bot_pose_tf.transform.rotation;

        auto look_ahead_pose = get_look_ahead_pose(bot_pose_stamped);

        double dx = look_ahead_pose.pose.position.x - bot_pose_stamped.pose.position.x;
        double dy = look_ahead_pose.pose.position.y - bot_pose_stamped.pose.position.y;
        double dist = std::sqrt( dx * dx + dy * dy );

        if (dist <= 0.1){
            RCLCPP_INFO(get_logger(),"Goal Reached!!!!!!!!!!!!");
            global_plan_.poses.clear();
            //stop the bot
            geometry_msgs::msg::Twist stop_vel;
            stop_vel.linear.x = 0.0;
            stop_vel.linear.y = 0.0;
            stop_vel.linear.z = 0.0;
            stop_vel.angular.z = 0.0;

            vel_pub_->publish(stop_vel);
            return;
        }

        tf2::Transform bot_to_look_ahead_pose_tf, w_to_robot_tf, w_to_lookahead_pose_tf;
        tf2::fromMsg (bot_pose_stamped.pose, w_to_robot_tf);
        tf2::fromMsg (look_ahead_pose.pose, w_to_lookahead_pose_tf);

        bot_to_look_ahead_pose_tf = w_to_robot_tf.inverse() * w_to_lookahead_pose_tf;

        tf2::toMsg(bot_to_look_ahead_pose_tf, look_ahead_pose.pose);
        double curvature = get_curvature(look_ahead_pose.pose);

        geometry_msgs::msg::Twist vel_;
        vel_.linear.x = max_linear_vel_;
        vel_.angular.z = curvature * max_angular_vel_;
        vel_pub_->publish(vel_);
    }

    bool PurePursuit::transform_plan (const std::string &frame){

        if(global_plan_.header.frame_id == frame){
            return true;
        }
        geometry_msgs::msg::TransformStamped transform;
        try{
            transform = tf_buffer_->lookupTransform(frame, global_plan_.header.frame_id, tf2::TimePointZero);
        }catch(tf2::TransformException &ex) {
            RCLCPP_ERROR_STREAM(get_logger(), "Transform error:"<<ex.what());
            return false;
        }
        for(auto &pose_ : global_plan_.poses){
            tf2::doTransform(pose_, pose_, transform);
        }
        global_plan_.header.frame_id = frame;
        return true; 
    }

    geometry_msgs::msg::PoseStamped PurePursuit::get_look_ahead_pose(const geometry_msgs::msg::PoseStamped &bot_pose){

        geometry_msgs::msg::PoseStamped look_ahead_pose = global_plan_.poses.back();//sets look ahead pose to goal


        for (auto pose_i = global_plan_.poses.rbegin(); pose_i != global_plan_.poses.rend(); ++pose_i){

            double dx = pose_i->pose.position.x - bot_pose.pose.position.x;
            double dy = pose_i->pose.position.y - bot_pose.pose.position.y;
            double dist = std::sqrt( dx * dx + dy * dy );
            if (dist > look_ahead_dist_){
                look_ahead_pose = *pose_i;
            }
            else{
                break;
            }
        }
        return look_ahead_pose;
    }

    double PurePursuit::get_curvature(const geometry_msgs::msg::Pose &la_pose){
        
        double dist = la_pose.position.x * la_pose.position.x + la_pose.position.y * la_pose.position.y;

        if (dist > 0.001){
            return (2.0 * la_pose.position.y / dist);
        }
        else{
            return 0.0;
        }
    }
}
int main(int argc, char **argv){
    rclcpp::init(argc,argv);
    auto node = std::make_shared<pure_pursuit::PurePursuit>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}