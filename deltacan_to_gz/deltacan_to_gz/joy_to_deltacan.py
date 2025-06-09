import rclpy
from sensor_msgs.msg import Joy
from deltacan.msg import DeltaCan
from rclpy.node import Node
import numpy as np

class JoytoDeltacan(Node):
    def __init__(self):
        super().__init__('joy_to_deltacan')
        self.declare_parameter('joy_topic', '/joy')
        self.declare_parameter('deltacan_topic', '/deltacan')
        joy_topic = self.get_parameter('joy_topic').get_parameter_value().string_value
        deltan_topic = self.get_parameter('deltacan_topic').get_parameter_value().string_value
        self.joy_subscriber = self.create_subscription(Joy, joy_topic, self.joy_callback, 10)
        self.deltacan_publisher = self.create_publisher(DeltaCan, deltan_topic, 10)
        pass

    def joy_callback(self, msg: Joy):
        deltan_msg = DeltaCan()
        deltan_msg.header.stamp = self.get_clock().now().to_msg()
        # boom 1 arm 4 bucket 3
        deltan_msg.mboomcmd = np.around(msg.axes[1],2)
        deltan_msg.marmcmd = np.around(msg.axes[4],2)
        deltan_msg.mbucketcmd = np.around(msg.axes[3],2)

        self.deltacan_publisher.publish(deltan_msg)


def main():
    rclpy.init()
    joy_to_deltacan = JoytoDeltacan()
    rclpy.spin(joy_to_deltacan)
    rclpy.shutdown()

if __name__ == '__main__':
    main()