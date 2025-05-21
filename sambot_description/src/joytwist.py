import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist

class JoyToTwist(Node):
    def __init__(self):
        super().__init__('joytwist_node')

        self.declare_parameter('linear_axis', 1)
        self.declare_parameter('angular_axis', 3)
        self.declare_parameter('linear_scale', 1.0)
        self.declare_parameter('angular_scale', 4.0)

        self.linear_axis = self.get_parameter('linear_axis').value
        self.angular_axis = self.get_parameter('angular_axis').value
        self.linear_scale = self.get_parameter('linear_scale').value
        self.angular_scale = self.get_parameter('angular_scale').value

        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        self.subscription = self.create_subscription(
            Joy,
            '/joy',
            self.joy_callback,
            10
        )

        self.get_logger().info('Joy to Twist node started.')

    def joy_callback(self, msg):
        twist = Twist()
        twist.linear.x = msg.axes[self.linear_axis] * self.linear_scale
        twist.angular.z = msg.axes[self.angular_axis] * self.angular_scale
        self.publisher_.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = JoyToTwist()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
