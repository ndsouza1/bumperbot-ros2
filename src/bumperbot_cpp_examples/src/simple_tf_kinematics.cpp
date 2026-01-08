#include "bumperbot_cpp_examples/simple_tf_kinematics.hpp"

using namespace std::chrono_literals;
using namespace std::placeholders;

SimpleTfKinematics::SimpleTfKinematics(const std::string &name)
: Node(name),
x_increment_ (0.05),
last_x(0.0),
total_yaw_angle_(0.0), // Initializing the new double variable
rotation_increment_(0.05) // Initializing the new double variable
{
    static_transform_broadcaster =std::make_shared<tf2_ros::StaticTransformBroadcaster>(this); //static transform broadcaster
    dynamic_transform_broadcaster =std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    tf_buffer_= std::make_unique<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_, this);

    static_transform_stamped_.header.stamp = get_clock()->now();
    static_transform_stamped_.header.frame_id = "bumperbot_base";
    static_transform_stamped_.child_frame_id ="bumperbot_top";
    static_transform_stamped_.transform.translation.x = 0.0;
    static_transform_stamped_.transform.translation.y = 0.0;
    static_transform_stamped_.transform.translation.z = 0.3;

    static_transform_stamped_.transform.rotation.x = 0.0;
    static_transform_stamped_.transform.rotation.y = 0.0;
    static_transform_stamped_.transform.rotation.z = 0.0;
    static_transform_stamped_.transform.rotation.w = 1.0;

    static_transform_broadcaster->sendTransform(static_transform_stamped_);

    RCLCPP_INFO_STREAM(get_logger(), "Publishing static transform between "
                        << static_transform_stamped_.header.frame_id << " and "
                        << static_transform_stamped_.child_frame_id);
                        
    timer_ = create_wall_timer(0.1s, std::bind(&SimpleTfKinematics::timerCallback, this));

    get_transform_srv = create_service<bumperbot_msgs::srv::GetTransform>("get_transform", std::bind(&SimpleTfKinematics::getTransformCallback, this, _1, _2)); //_1 ,_2 two variables as input
}

void SimpleTfKinematics::timerCallback(){
    dynamic_transform_stamped_.header.stamp = get_clock()->now();
    dynamic_transform_stamped_.header.frame_id = "odom";
    dynamic_transform_stamped_.child_frame_id ="bumperbot_base";
    
    dynamic_transform_stamped_.transform.translation.x = last_x + x_increment_;
    dynamic_transform_stamped_.transform.translation.y = 0.0;
    dynamic_transform_stamped_.transform.translation.z = 0.0;

    total_yaw_angle_ += rotation_increment_;

    tf2::Quaternion q;
    q.setRPY(0, 0, total_yaw_angle_);

    // Use tf2::toMsg to convert the tf2 quaternion to a geometry_msgs/Quaternion
    dynamic_transform_stamped_.transform.rotation = tf2::toMsg(q);

    dynamic_transform_broadcaster->sendTransform(dynamic_transform_stamped_);

    last_x = dynamic_transform_stamped_.transform.translation.x;

    if (std::abs(total_yaw_angle_) >= 2.0 * M_PI){
        rotation_increment_ *= -1; // Reverse the direction
        total_yaw_angle_ = 0.0;    // Reset the total angle
    }
}

bool SimpleTfKinematics::getTransformCallback(const std::shared_ptr<bumperbot_msgs::srv::GetTransform::Request> req,  
                                            std::shared_ptr<bumperbot_msgs::srv::GetTransform::Response> res){
    RCLCPP_INFO_STREAM(get_logger(), "Requested transform between " <<req->frame_id<< " & "<< req->child_frame_id);
    geometry_msgs::msg::TransformStamped requested_transform;
    try{
        requested_transform = tf_buffer_->lookupTransform(req->frame_id, req->child_frame_id, tf2::TimePointZero);
    }
    catch(tf2::TransformException &ex){
        RCLCPP_ERROR_STREAM(get_logger(), "An error occured while transforming from" 
                                        <<req->frame_id<< " & "<< req->child_frame_id<< ex.what());
        res->success = false;
        return true;
    }

    res->transform_stamped = requested_transform;
    res->success = true;
    return true;
}

int main( int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SimpleTfKinematics>("simple_tf_kinematics");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}