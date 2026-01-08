#ifndef SIMPLE_TRAJECTORY_HPP
#define SIMPLE_TRAJECTORY_HPP

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>

class SimpleTrajectory : public rclcpp::Node{
    public: 
        SimpleTrajectory(const std::string & name);
    private:
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
        rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr trajectory_pub_;
        nav_msgs::msg::Path trajectory_path_;
        int max_trajectory_points_;

        void odomCallback(const nav_msgs::msg::Odometry & msg);
};

#endif