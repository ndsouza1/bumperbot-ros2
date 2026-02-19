#include <algorithm>

#include "bumperbot_motion/pure_pursuit.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

#include "nav2_util/node_utils.hpp"

namespace bumperbot_motion{
  void PurePursuit::configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string name, std::shared_ptr<tf2_ros::Buffer> tf_buffer,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
  {
    node_ = parent;
    auto node = node_.lock();

    costmap_ros_ = costmap_ros;
    tf_buffer_ = tf_buffer;
    plugin_name_ = name;
    logger_ = node->get_logger();
    clock_ = node->get_clock();

    // Use nav2_util to properly declare parameters WITH the prefix
    nav2_util::declare_parameter_if_not_declared(
      node, plugin_name_ + ".look_ahead_distance",
      rclcpp::ParameterValue(0.5));
    nav2_util::declare_parameter_if_not_declared(
      node, plugin_name_ + ".max_linear_velocity",
      rclcpp::ParameterValue(0.3));
    nav2_util::declare_parameter_if_not_declared(
      node, plugin_name_ + ".max_angular_velocity",
      rclcpp::ParameterValue(1.0));

    node->get_parameter(plugin_name_ + ".look_ahead_distance", look_ahead_distance_);
    node->get_parameter(plugin_name_ + ".max_linear_velocity", max_linear_velocity_);
    node->get_parameter(plugin_name_ + ".max_angular_velocity", max_angular_velocity_);

    carrot_pose_pub_ = node->create_publisher<geometry_msgs::msg::PoseStamped>("pure_pursuit/carrot_pose", 1);
  }                
  void PurePursuit::cleanup() {
    RCLCPP_INFO(logger_, "Cleaning up plugin PurePursuit");
    carrot_pose_pub_.reset();
  }
  void PurePursuit::activate() {
    RCLCPP_INFO(logger_, "Activating plugin PurePursuit");
    carrot_pose_pub_->on_activate();
    }
  void PurePursuit::deactivate() {
      RCLCPP_INFO(logger_, "Deactivating plugin PurePursuit");
      carrot_pose_pub_->on_deactivate();
  }

  void PurePursuit::setPlan(const nav_msgs::msg::Path & path)  //set a plan for current motion plan
  {
    global_plan_ = path;
  }
  void PurePursuit::setSpeedLimit(const double & , const bool & ){} //sspeed limit-set velocity limits for PDMotionPlanner // percentage to check if it indicates the robots current speed or the absolute at which u wanna remove the robot


geometry_msgs::msg::TwistStamped PurePursuit::computeVelocityCommands(
  const geometry_msgs::msg::PoseStamped & robot_pose,
  const geometry_msgs::msg::Twist &,
  nav2_core::GoalChecker *) 
{
  geometry_msgs::msg::TwistStamped cmd_vel;
  cmd_vel.header.frame_id = robot_pose.header.frame_id;
  
  if(global_plan_.poses.empty()){
    RCLCPP_ERROR(logger_, "Empty Plan!");
    return cmd_vel;
  }
  
  if (!transformPlan(robot_pose.header.frame_id)) {
    RCLCPP_ERROR_STREAM(logger_, "Unable to transform plan in robot's frame");
    return cmd_vel;
  }

  auto carrot_pose = getCarrotPose(robot_pose);
  carrot_pose_pub_->publish(carrot_pose);
        
  // Calculate the curvature to the look-ahead point
  tf2::Transform carrot_pose_robot_tf, robot_tf, carrot_pose_tf;
  tf2::fromMsg(robot_pose.pose, robot_tf);
  tf2::fromMsg(carrot_pose.pose, carrot_pose_tf);
  carrot_pose_robot_tf = robot_tf.inverse() * carrot_pose_tf;
  tf2::toMsg(carrot_pose_robot_tf, carrot_pose.pose);
  double curvature = getCurvature(carrot_pose.pose);
  
  // DEBUG OUTPUT
  RCLCPP_INFO_THROTTLE(logger_, *clock_, 500, 
    "Carrot: x=%.2f y=%.2f, Curvature: %.2f", 
    carrot_pose.pose.position.x, 
    carrot_pose.pose.position.y, 
    curvature);
        
  // Create and publish the velocity command
  cmd_vel.twist.linear.x = max_linear_velocity_;
  cmd_vel.twist.angular.z = curvature * max_angular_velocity_;
  
  // DEBUG OUTPUT
  RCLCPP_INFO_THROTTLE(logger_, *clock_, 500,
    "Cmd_vel: linear=%.2f angular=%.2f",
    cmd_vel.twist.linear.x,
    cmd_vel.twist.angular.z);

  return cmd_vel;
}

bool PurePursuit::transformPlan(const std::string & frame){
  if(global_plan_.header.frame_id == frame){
    return true;
  }
  geometry_msgs::msg::TransformStamped transform;
  try{
    transform = tf_buffer_->lookupTransform(frame, global_plan_.header.frame_id, tf2::TimePointZero);
  }
  catch(tf2::TransformException & ex){  // Catch all transform exceptions
    RCLCPP_ERROR_STREAM(logger_, "Couldn't transform plan from frame "<< global_plan_.header.frame_id<< " to " <<frame);
    return false;
  }
  for (auto & pose : global_plan_.poses){
    tf2::doTransform(pose, pose, transform);
  }

  global_plan_.header.frame_id = frame;
  return true;
}

geometry_msgs::msg::PoseStamped PurePursuit::getCarrotPose(const geometry_msgs::msg::PoseStamped & robot_pose)
{
  geometry_msgs::msg::PoseStamped carrot_pose = global_plan_.poses.back(); // Default to goal
  
  // Find first point on path >= look_ahead_distance away
  for (const auto& pose : global_plan_.poses) {
    double dx = pose.pose.position.x - robot_pose.pose.position.x;
    double dy = pose.pose.position.y - robot_pose.pose.position.y;
    double distance = std::sqrt(dx * dx + dy * dy);
    
    if(distance >= look_ahead_distance_){
      carrot_pose = pose;
      break; // Found it, stop searching
    }
  }
  
  return carrot_pose;
}

double PurePursuit::getCurvature(const geometry_msgs::msg::Pose & carrot_pose)
{
  //calculate the eucledian distance between the carrot and robot pose
  const double L = (carrot_pose.position.x * carrot_pose.position.x) + (carrot_pose.position.y * carrot_pose.position.y);
  if(L > 0.001){
    return 2.0 * carrot_pose.position.y / L;
  }

  else {
    return 0.0;
  }
}
}

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(bumperbot_motion::PurePursuit, nav2_core::Controller)