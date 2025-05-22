import rclpy
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformException
import numpy as np
from tf_transformations import quaternion_matrix
import pickle

class FrameTransformator(Node):
    def __init__(self):
        super().__init__('frame_transformator')
        
        # Initialize TF2 listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # Wait for the TF tree to be populated
        self.get_logger().info("Waiting for TF tree to be available...")
        
        # Create a matrix to store transforms (initialized to None)
        self.transforms = None
        
        # Call function to get all transforms after getting frames dynamically
        self.get_all_transforms()

    def get_all_transforms(self):
        # Wait for some time to ensure the TF tree is populated
        rclpy.spin_once(self)
        
        # Get the list of all available frames in the TF tree
        frames = self.tf_buffer.all_frames_as_string()
        print(frames)
        frame_list = frames.splitlines()
        
        # Clean up the frame list (remove empty strings, etc.)
        frame_list = [frame.strip().split()[1] for frame in frame_list]

        print(f"Frame list: {frame_list}")
        input()
        # Set the number of frames
        num_frames = len(frame_list)
        self.transforms = np.zeros((num_frames, num_frames, 4, 4))
        
        # Log the available frames
        self.get_logger().info(f"Available frames: {frame_list}")

        self.transform_dict = {}
        
        # Iterate through each pair of frames
        for i, frame1 in enumerate(frame_list):
            for j, frame2 in enumerate(frame_list):
                if frame1 == frame2:
                    continue
                if i != j:
                    self.get_transform(frame1, frame2, i, j)
                else:
                    # Identity matrix for self-transform
                    self.transforms[i, j] = np.eye(4)
                    self.transform_dict[(frame1, frame2)] = [np.eye(4)]
    def get_transform(self, source_frame, target_frame, i, j):
        try:
            # Get transform from source_frame to target_frame
            transform: TransformStamped = self.tf_buffer.lookup_transform(target_frame, source_frame, rclpy.time.Time())
            
            # Extract translation and rotation
            translation = transform.transform.translation
            rotation = transform.transform.rotation
            
            # Convert to a 4x4 transformation matrix
            transform_matrix = np.eye(4)
            transform_matrix[:3, 3] = [translation.x, translation.y, translation.z]
            quaternion = [rotation.x, rotation.y, rotation.z, rotation.w]
            transform_matrix[:3, :3] = quaternion_matrix(quaternion)[:3, :3]
            
            # Store the result
            self.transforms[i, j] = transform_matrix

            self.transform_dict[(source_frame, target_frame)] = [transform_matrix]
            
            # Format the output as requested
            output = f"""\n
                        link: {source_frame}
                        parent: {target_frame}
                        T: {transform_matrix}
                        --------------------------------------\n
                    """

            with open('/home/christoa/nav2_ws/src/nav2_dev/empty_map/empty_map/transforms.txt', 'a') as f:
                f.write(output)

            with open('/home/christoa/nav2_ws/src/nav2_dev/empty_map/empty_map/transforms.pkl', 'wb') as f:
                pickle.dump(self.transform_dict, f)

            self.get_logger().info(output.strip())  # Print the formatted result
        
        except TransformException as e:
            self.get_logger().error(f"Could not get transform from {source_frame} to {target_frame}: {str(e)}")


def main(args=None):
    rclpy.init(args=args)
    
    frame_transformator = FrameTransformator()

    # Keep the node running to continue receiving transforms
    rclpy.spin(frame_transformator)
    
    rclpy.shutdown()

if __name__ == '__main__':
    main()
