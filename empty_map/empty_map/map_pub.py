import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
import rclpy.parameter
import rclpy.qos
from std_msgs.msg import Header
import tf2_ros
import numpy as np
import geometry_msgs.msg
from geometry_msgs.msg import TransformStamped
from scipy.spatial.transform import Rotation as R


class EmptyMapAndTFPublisher(Node):
    def __init__(self):
        # Initialize the ROS2 Node
        super().__init__('empty_map_and_tf_publisher')

        qos_profile = 1
        # Create a Publisher for the empty map
        self.map_publisher = self.create_publisher(OccupancyGrid, '/map', qos_profile)
        
        # Create a Static Transform Broadcaster
        self.tf_broadcaster = tf2_ros.StaticTransformBroadcaster(self)

        # Set up a timer to publish at 1 Hz
        self.timer = self.create_timer(1, self.publish_empty_map_and_tf)

    def publish_empty_map_and_tf(self):
        # Create an empty map message
        empty_map = OccupancyGrid()
        empty_map.header = Header()
        empty_map.header.stamp = self.get_clock().now().to_msg()
        empty_map.header.frame_id = 'map'  # Map is the reference frame
        
        # Set the dimensions of the map (e.g., 10x10 grid)
        empty_map.info.width = 200
        empty_map.info.height = 200
        empty_map.info.resolution = 0.1  # 0.1 meter resolution
        empty_map.info.origin.position.x = -10.0
        empty_map.info.origin.position.y = -10.0
        
        # Fill the map with zeros (no obstacles)
        empty_map.data = [0] * (empty_map.info.width * empty_map.info.height)
        
        # Publish the empty map
        self.map_publisher.publish(empty_map)
        # self.get_logger().info("Empty map published.")

        # Create and send the static transform from map to odom
        transform = TransformStamped()
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = 'map'  # Start from 'map' frame
        transform.child_frame_id = 'odom'  # Map frame is the parent, odom is the child

        # Assuming the map frame is at the origin of the odom frame
        transform.transform.translation.x = 0.1
        transform.transform.translation.y = 0.1
        transform.transform.translation.z = 0.0
        rpy = (0,0,0)
        rpy = np.deg2rad(rpy)
        quat = R.from_euler('xyz', rpy).as_quat()
        transform.transform.rotation.x = quat[0]
        transform.transform.rotation.y = quat[1]
        transform.transform.rotation.z = quat[2]
        transform.transform.rotation.w = quat[3]

        # Broadcast the static transform
        self.tf_broadcaster.sendTransform(transform)
        # self.get_logger().info("Static transform from map to odom broadcasted.")


def main(args=None):
    rclpy.init(args=args)
    
    # Create and spin the node
    node = EmptyMapAndTFPublisher()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Clean up and shut down
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
