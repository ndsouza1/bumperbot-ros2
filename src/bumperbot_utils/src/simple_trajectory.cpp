#include "bumperbot_utils/simple_trajectory.hpp"
#include <geometry_msgs/msg/pose_stamped.hpp>

using std::placeholders::_1;

SimpleTrajectory::SimpleTrajectory(const std::string & name)
    :Node(name)
{
   //parameters to customize topic name args 

    declare_parameter("odom_topic", "/bumperbot_controller/odom");
    declare_parameter("trajectory_topic", "/bumperbot_controller/trajetory");
    declare_parameter("max_trajectory_points", 1000);

    //get parameters
    std::string odom_topic = get_parameter("odom_topic").as_string();
    std::string trajectory_topic = get_parameter("trajectory_topic").as_string();
    max_trajectory_points_ = get_parameter("max_trajectory_points").as_int();

    odom_sub_ =create_subscription<nav_msgs::msg::Odometry>(odom_topic, 10,
                                std::bind(&SimpleTrajectory::odomCallback, this, _1));
    trajectory_pub_ = create_publisher<nav_msgs::msg::Path>(trajectory_topic, 10);

    RCLCPP_INFO_STREAM(get_logger(), "Starting rajectory Path node");
    RCLCPP_INFO_STREAM(get_logger(), "Subscribing to odom...." <<odom_topic);
    RCLCPP_INFO_STREAM(get_logger(), "Pubishing trajectory to rviz... "<<trajectory_topic);
    RCLCPP_INFO_STREAM(get_logger(), "Max trajectory points: "<< max_trajectory_points_);

    trajectory_path_.header.frame_id ="odom";
    trajectory_path_.poses.clear();
}

void SimpleTrajectory::odomCallback(const nav_msgs::msg::Odometry & msg){
    //create Pose from odometry msg
    geometry_msgs::msg::PoseStamped pose_stamped;
    pose_stamped.header = msg.header;
    pose_stamped.pose = msg.pose.pose;
    
    trajectory_path_.poses.push_back(pose_stamped);

    if (trajectory_path_.poses.size() > static_cast<size_t>(max_trajectory_points_))
    {
        trajectory_path_.poses.erase(trajectory_path_.poses.begin());
    }

    trajectory_path_.header.stamp = get_clock()->now();
    trajectory_path_.header.frame_id = msg.header.frame_id;   
    
    trajectory_pub_->publish(trajectory_path_);
        
    RCLCPP_DEBUG_STREAM(get_logger(), 
                    "Added pose to trajectory. Total points: " << trajectory_path_.poses.size());
}

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SimpleTrajectory>("simple_trajectory_node");
    
    RCLCPP_INFO_STREAM(node->get_logger(), "Simple Trajectory Node started");
    
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}