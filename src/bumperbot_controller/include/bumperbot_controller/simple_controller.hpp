#ifndef BUMPERBOT_CONTROLLER__SIMPLE_CONTROLLER_HPP_
#define BUMPERBOT_CONTROLLER__SIMPLE_CONTROLLER_HPP_

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <Eigen/Dense>
#include <geometry_msgs/msg/transform_stamped.h>

class SimpleController : public rclcpp::Node
{
public:
    SimpleController(const std::string & name);

private:
    void VelocityCallback(const geometry_msgs::msg::TwistStamped & msg);
    void jointCallback(const sensor_msgs::msg::JointState & msg);

    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr wheel_cmd_pub_;
    rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr vel_sub_;
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

    Eigen::Matrix2d speed_conversion_;
    nav_msgs::msg::Odometry odom_msg_;

    std::unique_ptr<tf2_ros::TransformBroadcaster> transform_broadcaster_;
    geometry_msgs::msg::TransformStamped transform_stamped_;
};

#endif  // BUMPERBOT_CONTROLLER__SIMPLE_CONTROLLER_HPP_