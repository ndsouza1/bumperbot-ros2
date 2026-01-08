#ifndef SIMPLE_TF_KINEMATICS_HPP
#define SIMPLE_TF_KINEMATICS_HPP

#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <bumperbot_msgs/srv/get_transform.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include <cmath> 

#include <memory>

class SimpleTfKinematics : public rclcpp::Node
{
    public:
        SimpleTfKinematics(const std::string &name);

    private:
        std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_transform_broadcaster;
        std::shared_ptr<tf2_ros::TransformBroadcaster> dynamic_transform_broadcaster;
        geometry_msgs::msg::TransformStamped static_transform_stamped_;
        geometry_msgs::msg::TransformStamped dynamic_transform_stamped_;

        rclcpp::Service<bumperbot_msgs::srv::GetTransform>::SharedPtr get_transform_srv;
        
        rclcpp::TimerBase::SharedPtr timer_;

        std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
        std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};

        double x_increment_;
        double last_x;

        // Changed this to a double to store the total rotation angle
        double total_yaw_angle_;
        
        // This is now a double and will be used to increment the total_yaw_angle_
        double rotation_increment_;

        bool getTransformCallback(const std::shared_ptr<bumperbot_msgs::srv::GetTransform::Request> req,  
                            std::shared_ptr<bumperbot_msgs::srv::GetTransform::Response> res);

        

        void timerCallback();
};

#endif