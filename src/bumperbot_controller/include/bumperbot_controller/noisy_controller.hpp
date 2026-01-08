#ifndef BUMPERBOT_CONTROLLER__NOISY_CONTROLLER_HPP_
#define BUMPERBOT_CONTROLLER__NOISY_CONTROLLER_HPP_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.h>

class NoisyController : public rclcpp::Node
{
public:
    NoisyController(const std::string & name);

private:
    void jointCallback(const sensor_msgs::msg::JointState & msg);

    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_sub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;

    double wheel_radius_;
    double wheel_separation_;

    double left_wheel_prev_position_;
    double right_wheel_prev_position_;
    rclcpp::Time prev_time;
    
    double x_ = 0.0;
    double y_ = 0.0;
    double theta_ = 0.0;

    bool first_joint_msg_;

    nav_msgs::msg::Odometry odom_msg_;

    std::unique_ptr<tf2_ros::TransformBroadcaster> transform_broadcaster_;
    geometry_msgs::msg::TransformStamped transform_stamped_;
};

#endif  // BUMPERBOT_CONTROLLER__NOISY_CONTROLLER_HPP_