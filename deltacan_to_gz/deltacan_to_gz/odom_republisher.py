import rclpy
from rclpy.node import Node

from tf2_msgs.msg import TFMessage
from nav_msgs.msg import Odometry
from transforms3d.euler import euler2quat, quat2euler

class OracleOdomPublisher(Node):
    def __init__(self):
        super().__init__('odom_republisher')

        # Replace with your actual controller topic name
        self.publisher_ = self.create_publisher(
            Odometry,
            '/odom',
            10
        )

        self.create_subscription(
            TFMessage,
            '/robot_pose',
            self.tf_callback,
            10
        )

        self.prev_tf = None
        self.curr_tf = None


    def tf_callback(self, msg:TFMessage):
        self.latest_tf = msg
        self.prev_tf = self.curr_tf
        self.curr_tf = msg.transforms[11]
        if self.prev_tf is not None and self.curr_tf is not None:
            odom_msg = Odometry()
            odom_msg.header = self.curr_tf.header
            odom_msg.child_frame_id = self.curr_tf.child_frame_id
            odom_msg.pose.pose.position.x = self.curr_tf.transform.translation.x
            odom_msg.pose.pose.position.y = self.curr_tf.transform.translation.y
            odom_msg.pose.pose.position.z = self.curr_tf.transform.translation.z
            odom_msg.pose.pose.orientation = self.curr_tf.transform.rotation

            dt = (self.curr_tf.header.stamp.sec + self.curr_tf.header.stamp.nanosec * 1e-9) - (self.prev_tf.header.stamp.sec + self.prev_tf.header.stamp.nanosec * 1e-9)
            if dt > 0:
                odom_msg.twist.twist.linear.x = (self.curr_tf.transform.translation.x - self.prev_tf.transform.translation.x) / dt
                odom_msg.twist.twist.linear.y = (self.curr_tf.transform.translation.y - self.prev_tf.transform.translation.y) / dt
                odom_msg.twist.twist.linear.z = (self.curr_tf.transform.translation.z - self.prev_tf.transform.translation.z) / dt
               
                q1 = self.prev_tf.transform.rotation
                q2 = self.curr_tf.transform.rotation
                e1 = quat2euler([q1.x, q1.y, q1.z, q1.w])
                e2 = quat2euler([q2.x, q2.y, q2.z, q2.w])
                odom_msg.twist.twist.angular.x = (e2[0] - e1[0]) / dt
                odom_msg.twist.twist.angular.y = (e2[1] - e1[1]) / dt
                odom_msg.twist.twist.angular.z = (e2[2] - e1[2]) / dt

            self.publisher_.publish(odom_msg)
        

def main(args=None):
    rclpy.init(args=args)
    node = OracleOdomPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
