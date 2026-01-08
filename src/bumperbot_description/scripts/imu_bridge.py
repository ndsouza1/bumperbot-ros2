#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
import gz.msgs  # Gazebo messages
import gz.transport as gz

class ImuBridge(Node):
    def __init__(self):
        super().__init__('imu_bridge')
        self.pub = self.create_publisher(Imu, '/imu', 10)

        self.sub = gz.Subscriber('/imu', gz.msgs.IMU, self.callback)

    def callback(self, msg):
        imu_msg = Imu()
        imu_msg.header.stamp = self.get_clock().now().to_msg()
        imu_msg.header.frame_id = 'imu_link'

        # Copy orientation
        imu_msg.orientation.x = msg.orientation.x
        imu_msg.orientation.y = msg.orientation.y
        imu_msg.orientation.z = msg.orientation.z
        imu_msg.orientation.w = msg.orientation.w

        # Copy angular velocity
        imu_msg.angular_velocity.x = msg.angular_velocity.x
        imu_msg.angular_velocity.y = msg.angular_velocity.y
        imu_msg.angular_velocity.z = msg.angular_velocity.z

        # Copy linear acceleration
        imu_msg.linear_acceleration.x = msg.linear_acceleration.x
        imu_msg.linear_acceleration.y = msg.linear_acceleration.y
        imu_msg.linear_acceleration.z = msg.linear_acceleration.z

        self.pub.publish(imu_msg)

def main(args=None):
    rclpy.init(args=args)
    node = ImuBridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
