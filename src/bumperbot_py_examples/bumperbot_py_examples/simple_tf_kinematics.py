import rclpy
from rclpy.node import Node
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
from tf2_ros import TransformBroadcaster, TransformException
from geometry_msgs.msg import TransformStamped
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from bumperbot_msgs.srv import GetTransform
from tf_transformations import quaternion_from_euler, quaternion_multiply, quaternion_inverse

class SimpleTfKinematics(Node):
    def __init__(self):
        super().__init__("simple_tf_kinematics")

        # Broadcasters
        self.static_tf_broadcaster_ = StaticTransformBroadcaster(self)
        self.dynamic_transform_broadcaster_ = TransformBroadcaster(self)

        # Messages
        self.static_transform_stamped_ = TransformStamped()
        self.dynamic_transform_stamped_ = TransformStamped()

        # Movement parameters
        self.x_increment_ = 0.05
        self.last_x_ = 0.0
        #rotation along yaw
        self.total_yaw_angle_ = 0.0
        self.rotation_increment_ = 0.05


        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Static transform setup
        self.static_transform_stamped_.header.stamp = self.get_clock().now().to_msg()
        self.static_transform_stamped_.header.frame_id = "bumperbot_base"
        self.static_transform_stamped_.child_frame_id = "bumperbot_top"
        self.static_transform_stamped_.transform.translation.x = 0.0
        self.static_transform_stamped_.transform.translation.y = 0.0
        self.static_transform_stamped_.transform.translation.z = 0.3
        self.static_transform_stamped_.transform.rotation.x = 0.0
        self.static_transform_stamped_.transform.rotation.y = 0.0
        self.static_transform_stamped_.transform.rotation.z = 0.0
        self.static_transform_stamped_.transform.rotation.w = 1.0

        self.static_tf_broadcaster_.sendTransform(self.static_transform_stamped_)

        self.get_logger().info(
            "Publishing the static transform between %s and %s" %
            (self.static_transform_stamped_.header.frame_id, self.static_transform_stamped_.child_frame_id)
        )

        # Timer for dynamic transform
        self.timer = self.create_timer(0.1, self.timerCallback)

        self.get_transform_srv_ = self.create_service(GetTransform, "get_transform", self.getTransformCallback)

    def timerCallback(self):
        # Fill header
        self.dynamic_transform_stamped_.header.stamp = self.get_clock().now().to_msg()
        self.dynamic_transform_stamped_.header.frame_id = "odom"
        self.dynamic_transform_stamped_.child_frame_id = "bumperbot_base"

        # Update translation
        self.dynamic_transform_stamped_.transform.translation.x = self.last_x_ + self.x_increment_
        self.dynamic_transform_stamped_.transform.translation.y = 0.0
        self.dynamic_transform_stamped_.transform.translation.z = 0.0

        #quaternions
        self.total_yaw_angle_ += self.rotation_increment_
        q = quaternion_from_euler(0, 0, self.total_yaw_angle_)

        # Rotation stays fixed
        self.dynamic_transform_stamped_.transform.rotation.x = q[0]
        self.dynamic_transform_stamped_.transform.rotation.y = q[1]
        self.dynamic_transform_stamped_.transform.rotation.z = q[2]
        self.dynamic_transform_stamped_.transform.rotation.w = q[3]

        # Broadcast transform
        self.dynamic_transform_broadcaster_.sendTransform(self.dynamic_transform_stamped_)
        
        self.last_x_ = self.dynamic_transform_stamped_.transform.translation.x

        # Check if the angle should reverse and reset the counter
        if abs(self.total_yaw_angle_) >= 2 * 3.14159:  # A full rotation
            self.rotation_increment_ *= -1
            self.total_yaw_angle_ = 0.0
    
    def getTransformCallback(self, req, res):
        self.get_logger().info("Requested transform between %s and %s " % (req.frame_id, req.child_frame_id))
        
        try:
            requested_transform = self.tf_buffer.lookup_transform(
                req.frame_id, req.child_frame_id, rclpy.time.Time())
            
            res.transform_stamped = requested_transform
            res.success = True
            
        except TransformException as e:
            self.get_logger().error(f"An error occurred while transforming {req.frame_id} and {req.child_frame_id}: {e}")
            res.success = False 
            
        return res

def main():
    rclpy.init()
    node = SimpleTfKinematics()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
