#include <algorithm>

#include "bumperbot_motion/pd_motion_planner.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

#include "nav2_util/lifecycle_node.hpp"

namespace bumperbot_motion{

void PDMotionPlanner::configure(
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

  auto param = plugin_name_ + ".kp";
  if (!node->has_parameter(param)) {
    node->declare_parameter(param, 2.0);
  }

  param = plugin_name_ + ".kd";
  if (!node->has_parameter(param)) {
    node->declare_parameter(param, 0.1);
  }

  param = plugin_name_ + ".max_linear_velocity";
  if (!node->has_parameter(param)) {
    node->declare_parameter(param, 0.3);
  }

  param = plugin_name_ + ".max_angular_velocity";
  if (!node->has_parameter(param)) {
    node->declare_parameter(param, 1.0);
  }

  param = plugin_name_ + ".step_size";
  if (!node->has_parameter(param)) {
    node->declare_parameter(param, 0.2);
  }

  node->get_parameter(plugin_name_ + ".kp", kp_);
  node->get_parameter(plugin_name_ + ".kd", kd_);
  node->get_parameter(plugin_name_ + ".max_linear_velocity", max_linear_velocity_);
  node->get_parameter(plugin_name_ + ".max_angular_velocity", max_angular_velocity_);
  node->get_parameter(plugin_name_ + ".step_size", step_size_);

  next_pose_pub_ = node->create_publisher<geometry_msgs::msg::PoseStamped>("pd/next_pose", 1);
}
                  
void PDMotionPlanner::cleanup() {
    RCLCPP_INFO(logger_, "Cleaning up plugin PDMotionPlanner");
    next_pose_pub_.reset();
  }
void PDMotionPlanner::activate()
  {
    auto node = node_.lock();
    RCLCPP_INFO(logger_, "Activating PDMotionPlanner");
    last_cycle_time_ = node->get_clock()->now();
  }
void PDMotionPlanner::deactivate()
  {
    RCLCPP_INFO(logger_, "Deactivating plugin PDMotionPlanner");
  }


void PDMotionPlanner::setPlan(const nav_msgs::msg::Path & path) {
  global_plan_ = path;
}
void PDMotionPlanner::setSpeedLimit(const double & speed_limit, const bool & ) {
  
}

geometry_msgs::msg::TwistStamped PDMotionPlanner::computeVelocityCommands(const geometry_msgs::msg::PoseStamped & robot_pose,
                                                        const geometry_msgs::msg::Twist & velocity,
                                                        nav2_core::GoalChecker *){
  geometry_msgs::msg::TwistStamped  cmd_vel;
  cmd_vel.header.frame_id = robot_pose.header.frame_id;


  if(global_plan_.poses.empty()){
    RCLCPP_ERROR(logger_, "Empty Plan!");
    return cmd_vel;
  }

  //RCLCPP_INFO(get_logger(), "frame_id Robot pose: %s", robot_poses_.header.frame_id.c_str());
  //RCLCPP_INFO(get_logger(), "frame_id Global Plan: %s", global_plan_.header.frame_id.c_str());
  if (!transformPlan(robot_pose.header.frame_id))
  {
    RCLCPP_ERROR_STREAM(logger_, "Unable to transform plan in robot's frame");
    return cmd_vel;
  } // path we recieve bu the algo which is in map is transformed to robot frame id


  auto next_pose = getNextPose(robot_pose);

  next_pose_pub_->publish(next_pose);
  tf2::Transform robot_tf, next_pose_tf, next_pose_robot_tf;
  tf2::fromMsg(robot_pose.pose, robot_tf);
  tf2::fromMsg(next_pose.pose, next_pose_tf);

  next_pose_robot_tf = robot_tf.inverse() * next_pose_tf;
  double linear_error = next_pose_robot_tf.getOrigin().getX(); // when there is error x we move robot linearly 
  double angular_error = next_pose_robot_tf.getOrigin().getY(); // when there is error y we move robot angular(in place rotation)  

  double dt = (clock_->now() - last_cycle_time_).seconds(); //time elapsed between two consecutive control loop solutions
  double linear_error_derivative =(linear_error - prev_linear_error_) / dt;
  double angular_error_derivative =(angular_error - prev_angular_error_) / dt;

  cmd_vel.twist.linear.x = std::clamp(kp_ * linear_error + kd_ * linear_error_derivative, -max_linear_velocity_, max_linear_velocity_);
  cmd_vel.twist.angular.z = std::clamp(kp_ * angular_error + kd_ * angular_error_derivative, -max_angular_velocity_, max_angular_velocity_);

  last_cycle_time_ = clock_->now();
  prev_linear_error_ = linear_error;
  prev_angular_error_ = angular_error;
  return cmd_vel;
}

bool PDMotionPlanner::transformPlan(const std::string & frame){
  if(global_plan_.header.frame_id == frame){
    return true;
  }
  geometry_msgs::msg::TransformStamped transform;
  try{
  transform = tf_buffer_->lookupTransform(frame, global_plan_.header.frame_id, tf2::TimePointZero);
  }
  catch(tf2::LookupException & ex){
    RCLCPP_ERROR_STREAM(logger_, "Couldn't transform plan from frame "<< global_plan_.header.frame_id<< " to " <<frame);
    return false;
  }
  for (auto & pose : global_plan_.poses){ //pose_stamped message geometry_msgs to tf2 objects for transformation
    tf2::doTransform(pose, pose, transform);
  }

  global_plan_.header.frame_id = frame;
  return true;
}
geometry_msgs::msg::PoseStamped PDMotionPlanner::getNextPose(const geometry_msgs::msg::PoseStamped & robot_pose){
  auto next_pose = global_plan_.poses.back();
  for(auto pose_it = global_plan_.poses.rbegin(); pose_it != global_plan_.poses.rend(); ++pose_it){
    double dx = pose_it -> pose.position.x - robot_pose.pose.position.x;
    double dy = pose_it -> pose.position.y - robot_pose.pose.position.y;
    double distance = std::sqrt(dx * dx + dy * dy);

    if(distance > step_size_){
      next_pose = *pose_it;
    }
    else{
      break;  // Same problem as before!
    }
  }
  return next_pose;
}
}
#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(bumperbot_motion::PDMotionPlanner, nav2_core::Controller)