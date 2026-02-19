#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include "nav2_core/controller.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"

namespace bumperbot_motion
{
  class PDMotionPlanner : public nav2_core::Controller
  {
    public:
      PDMotionPlanner() = default;
      ~PDMotionPlanner() =default;

      void configure(
            const rclcpp_lifecycle::LifecycleNode::WeakPtr &,
            std::string name, std::shared_ptr<tf2_ros::Buffer>,
            std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;
                          
      void cleanup() override;
      void activate() override;
      void deactivate() override;

      geometry_msgs::msg::TwistStamped computeVelocityCommands(
                            const geometry_msgs::msg::PoseStamped & robot_pose,
                            const geometry_msgs::msg::Twist & velocity,
                            nav2_core::GoalChecker * goal_checker) override;

      void setPlan(const nav_msgs::msg::Path & path) override; //set a plan for current motion plan
      void setSpeedLimit(const double & speed_limit, const bool & percentage) override; //sspeed limit-set velocity limits for PDMotionPlanner // percentage to check if it indicates the robots current speed or the absolute at which u wanna remove the robot

    private:
      rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr next_pose_pub_;

      rclcpp::TimerBase::SharedPtr control_loop_;
      double kp_;
      double kd_;
      double step_size_;

      double max_linear_velocity_;
      double max_angular_velocity_;
      nav_msgs::msg::Path global_plan_;

      double prev_angular_error_;
      double prev_linear_error_;
      rclcpp::Time last_cycle_time_;

      std::string plugin_name_;
      std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
      rclcpp::Logger logger_{rclcpp::get_logger("PDMotionPlanner")};
      rclcpp::Clock::SharedPtr clock_;
      rclcpp_lifecycle::LifecycleNode::WeakPtr node_;
      std::shared_ptr<tf2_ros::Buffer> tf_buffer_;

      bool transformPlan(const std::string & frame);

      geometry_msgs::msg::PoseStamped getNextPose(const geometry_msgs::msg::PoseStamped & robot_pose);
  };

}