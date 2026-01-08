import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class TwistRelay(Node):
    def __init__(self):
        super().__init__('twist_relay')
        self.declare_parameter('input_topic', '/cmd_vel')
        self.declare_parameter('output_topic', '/diff_drive_controller/cmd_vel_unstamped')

        input_topic = self.get_parameter('input_topic').value
        output_topic = self.get_parameter('output_topic').value

        self.publisher_ = self.create_publisher(Twist, output_topic, 10)
        self.subscription_ = self.create_subscription(Twist, input_topic, self.relay_callback, 10)
        self.get_logger().info(f"Relaying from {input_topic} → {output_topic}")

    def relay_callback(self, msg):
        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = TwistRelay()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
