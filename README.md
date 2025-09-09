# nav2_dev

To build first install Gazebo Classic+Ignition: https://gazebosim.org/docs/latest/ros_installation/ <br>
Install Gazebo ROS (Humble): https://index.ros.org/p/gazebo_ros/ <br>
Install the control packages (Humble): https://github.com/ros-controls/gz_ros2_control
sudo apt install ros-${ROS_DISTRO}-ros-ign-bridge
ign topic -t "/cmd_vel" -m ignition.msgs.Twist -p "linear: {x: -0.5}, angular: {z: 0.05}"
Launch the sim using `ros2 launch earth_gz_ign velocity_control.launch.py`