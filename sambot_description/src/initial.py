import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

class TrajectoryPublisher(Node):
    def __init__(self):
        super().__init__('trajectory_publisher')

        # Replace with your actual controller topic name
        self.publisher_ = self.create_publisher(
            JointTrajectory,
            '/joint_trajectory_controller/joint_trajectory',
            10
        )

        # Timer to publish once after 1 second
        self.timer = self.create_timer(1.0, self.publish_trajectory)

    def publish_trajectory(self):
        self.get_logger().info('Publishing trajectory point...')
        msg = JointTrajectory()
        
        # Joint names
        msg.joint_names = [
            'excavator_slew',
            'arm_to_bucket',
            'plow_to_base',
            'boom_to_arm',
            'swing_to_boom',
            'cabin_to_swing'
        ]

        # Create a single trajectory point
        point = JointTrajectoryPoint()
        point.positions = [
            -0.00039181548393951715,
            -0.01158286028037736,
            0.009413709438744533,
            -0.008706551191894896,
            0.007019542682725977,
            0.022634932995336143
        ]
        point.velocities = [
            0.0020333712219766796,
            -0.04149319183448896,
            0.04995732743865372,
            -0.016638111777325673,
            0.025199998696727635,
            -0.0654129095620147
        ]


        msg.points.append(point)
        print(msg)

        self.publisher_.publish(msg)
        self.get_logger().info('Published trajectory point')

def main(args=None):
    rclpy.init(args=args)
    node = TrajectoryPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
