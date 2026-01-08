#ifndef BUMPERBOT_LOCALIZATION_ODMETRY_MOTION_MODEL_HPP
#define BUMPERBOT_LOCALIZATION_ODMETRY_MOTION_MODEL_HPP

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose_array.hpp>


class OdometryMotionModel : public rclcpp::Node
{
public:
    OdometryMotionModel(const std::string & name);
    
private:
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr pose_array_pub_;

    geometry_msgs::msg::PoseArray samples_;
    
    double alpha1_;
    double alpha2_;
    double alpha3_;
    double alpha4_;
    int nr_samples;


    double last_odom_x;
    double last_odom_y;
    double last_odom_theta;

    bool is_first_odom;

    void odomCallback(const nav_msgs::msg::Odometry & odom);

};

#endif