# Quickstart Guide: Module 3 - The AI-Robot Brain (NVIDIA Isaac)

## Prerequisites

1. **System Requirements**:
   - Ubuntu 22.04 (recommended) or equivalent Linux distribution
   - NVIDIA GPU with CUDA support (RTX series recommended)
   - At least 16GB RAM (32GB recommended for Isaac Sim)
   - 50GB free disk space
   - Python 3.8 or higher

2. **Install ROS 2 Humble Hawksbill** (if not already installed):
   ```bash
   # Add the ROS 2 apt repository
   sudo apt update && sudo apt install -y curl gnupg lsb-release
   sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

   echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(source /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

   sudo apt update
   sudo apt install ros-humble-desktop
   sudo apt install ros-dev-tools
   ```

3. **Install NVIDIA Isaac Sim**:
   - Download Isaac Sim from NVIDIA Developer website
   - Follow installation instructions for your platform
   - Ensure CUDA drivers are properly installed

4. **Install Isaac ROS packages**:
   ```bash
   sudo apt update
   sudo apt install ros-humble-isaac-ros-*
   ```

## Setting Up Your AI Robot Workspace

1. **Create a ROS workspace for AI robotics**:
   ```bash
   mkdir -p ~/ai_robot_ws/src
   cd ~/ai_robot_ws
   colcon build
   source install/setup.bash
   ```

2. **Create an AI robotics package**:
   ```bash
   cd ~/ai_robot_ws/src
   ros2 pkg create --build-type ament_python ai_robot_examples
   ```

3. **Verify Installation**:
   ```bash
   # Check that Isaac ROS packages are available
   ros2 pkg list | grep isaac_ros

   # Check ROS 2 setup
   ros2 topic list
   ```

## Running Your First AI Perception Example

1. **Create a basic perception node** (`~/ai_robot_ws/src/ai_robot_examples/ai_robot_examples/perception_node.py`):
   ```python
   import rclpy
   from rclpy.node import Node
   from sensor_msgs.msg import Image
   from cv_bridge import CvBridge
   import cv2
   import numpy as np

   class PerceptionNode(Node):
       def __init__(self):
           super().__init__('perception_node')
           self.bridge = CvBridge()

           # Create subscription for camera input
           self.subscription = self.create_subscription(
               Image,
               '/camera/image_raw',
               self.image_callback,
               10
           )

           # Create publisher for processed image
           self.publisher = self.create_publisher(
               Image,
               '/camera/processed_image',
               10
           )

           self.get_logger().info('Perception node initialized')

       def image_callback(self, msg):
           # Convert ROS Image message to OpenCV image
           cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

           # Simple processing example: edge detection
           gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
           edges = cv2.Canny(gray, 50, 150)

           # Convert back to ROS Image message
           processed_msg = self.bridge.cv2_to_imgmsg(edges, encoding='mono8')
           processed_msg.header = msg.header

           # Publish processed image
           self.publisher.publish(processed_msg)

           self.get_logger().info('Processed image and published result')

   def main(args=None):
       rclpy.init(args=args)
       perception_node = PerceptionNode()

       try:
           rclpy.spin(perception_node)
       except KeyboardInterrupt:
           pass
       finally:
           perception_node.destroy_node()
           rclpy.shutdown()

   if __name__ == '__main__':
       main()
   ```

2. **Update setup.py** (`~/ai_robot_ws/src/ai_robot_examples/setup.py`):
   ```python
   from setuptools import find_packages, setup

   package_name = 'ai_robot_examples'

   setup(
       name=package_name,
       version='0.0.0',
       packages=find_packages(exclude=['test']),
       data_files=[
           ('share/ament_index/resource_index/packages',
               ['resource/' + package_name]),
           ('share/' + package_name, ['package.xml']),
       ],
       install_requires=['setuptools'],
       zip_safe=True,
       maintainer='Your Name',
       maintainer_email='your.email@example.com',
       description='AI robotics examples for ROS 2',
       license='Apache-2.0',
       tests_require=['pytest'],
       entry_points={
           'console_scripts': [
               'perception_node = ai_robot_examples.perception_node:main',
           ],
       },
   )
   ```

3. **Build and run the perception node**:
   ```bash
   cd ~/ai_robot_ws
   colcon build --packages-select ai_robot_examples
   source install/setup.bash

   # Run the perception node
   ros2 run ai_robot_examples perception_node
   ```

## Setting up Isaac Sim for VSLAM

1. **Launch Isaac Sim**:
   ```bash
   # Navigate to your Isaac Sim installation directory
   cd /path/to/isaac-sim
   ./isaac-sim.sh
   ```

2. **Create a simple VSLAM scene** in Isaac Sim:
   - Open Isaac Sim
   - Create a new stage
   - Add a robot with RGB-D camera sensors
   - Configure the camera settings for VSLAM
   - Add environmental features for visual landmarks

3. **Connect Isaac Sim to ROS 2**:
   - Use Isaac Sim's ROS bridge to publish camera data
   - Verify topics are available: `ros2 topic list | grep camera`

## Nav2 Path Planning Setup

1. **Create a Nav2 configuration** (`~/ai_robot_ws/src/ai_robot_examples/config/nav2_config.yaml`):
   ```yaml
   amcl:
     ros__parameters:
       use_sim_time: True
       alpha1: 0.2
       alpha2: 0.2
       alpha3: 0.2
       alpha4: 0.2
       alpha5: 0.2
       base_frame_id: "base_footprint"
       beam_skip_distance: 0.5
       beam_skip_error_threshold: 0.9
       beam_skip_threshold: 0.3
       do_beamskip: false
       global_frame_id: "map"
       lambda_short: 0.1
       laser_likelihood_max_dist: 2.0
       laser_max_range: 100.0
       laser_min_range: -1.0
       laser_model_type: "likelihood_field"
       max_beams: 60
       max_particles: 2000
       min_particles: 500
       odom_frame_id: "odom"
       pf_err: 0.05
       pf_z: 0.99
       recovery_alpha_fast: 0.0
       recovery_alpha_slow: 0.0
       resample_interval: 1
       robot_model_type: "nav2_amcl::DifferentialMotionModel"
       save_pose_rate: 0.5
       sigma_hit: 0.2
       tf_broadcast: true
       transform_tolerance: 1.0
       update_min_a: 0.2
       update_min_d: 0.2
       z_hit: 0.5
       z_max: 0.05
       z_rand: 0.5
       z_short: 0.05

   bt_navigator:
     ros__parameters:
       use_sim_time: True
       global_frame: map
       robot_base_frame: base_link
       odom_topic: /odom
       bt_loop_duration: 10
       default_server_timeout: 20
       enable_groot_monitoring: True
       groot_zmq_publisher_port: 1666
       groot_zmq_server_port: 1667
       navigate_through_poses: False
       navigate_to_pose: True
       action_server_result_timeout: 900.0
       bt_xml_filename: "navigate_w_replanning_and_recovery.xml"
       default_nav_through_poses_bt_xml: "navigate_w_replanning_and_recovery.xml"
       default_nav_to_pose_bt_xml: "navigate_w_replanning_and_recovery.xml"
       plugin_lib_names:
       - nav2_compute_path_to_pose_action_bt_node
       - nav2_compute_path_through_poses_action_bt_node
       - nav2_smooth_path_action_bt_node
       - nav2_follow_path_action_bt_node
       - nav2_spin_action_bt_node
       - nav2_wait_action_bt_node
       - nav2_assisted_teleop_action_bt_node
       - nav2_back_up_action_bt_node
       - nav2_drive_on_heading_bt_node
       - nav2_clear_costmap_service_bt_node
       - nav2_is_stuck_condition_bt_node
       - nav2_goal_reached_condition_bt_node
       - nav2_goal_updated_condition_bt_node
       - nav2_globally_consistent_localization_condition_bt_node
       - nav2_is_path_valid_condition_bt_node
       - nav2_initial_pose_received_condition_bt_node
       - nav2_reinitialize_global_localization_on_amcl_reset_bt_node
       - nav2_rate_controller_bt_node
       - nav2_distance_controller_bt_node
       - nav2_speed_controller_bt_node
       - nav2_truncate_path_action_bt_node
       - nav2_truncate_path_local_action_bt_node
       - nav2_goal_updater_node_bt_node
       - nav2_recovery_node_bt_node
       - nav2_pipeline_sequence_bt_node
       - nav2_round_robin_node_bt_node
       - nav2_transform_available_condition_bt_node
       - nav2_time_expired_condition_bt_node
       - nav2_path_expiring_timer_condition
       - nav2_distance_traveled_condition_bt_node
       - nav2_single_trigger_bt_node
       - nav2_is_battery_low_condition_bt_node
       - nav2_navigate_through_poses_action_bt_node
       - nav2_navigate_to_pose_action_bt_node
       - nav2_remove_passed_goals_action_bt_node
       - nav2_planner_selector_bt_node
       - nav2_controller_selector_bt_node
       - nav2_goal_checker_selector_bt_node
       - nav2_controller_cancel_bt_node
       - nav2_path_longer_on_approach_bt_node
       - nav2_wait_cancel_bt_node
       - nav2_spin_cancel_bt_node
       - nav2_back_up_cancel_bt_node
       - nav2_assisted_teleop_cancel_bt_node
       - nav2_drive_on_heading_cancel_bt_node

   controller_server:
     ros__parameters:
       use_sim_time: True
       controller_frequency: 20.0
       min_x_velocity_threshold: 0.001
       min_y_velocity_threshold: 0.5
       min_theta_velocity_threshold: 0.001
       progress_checker_plugin: "progress_checker"
       goal_checker_plugin: "goal_checker"
       controller_plugins: ["FollowPath"]

       # Progress checker parameters
       progress_checker:
         plugin: "nav2_controller::SimpleProgressChecker"
         required_movement_radius: 0.5
         movement_time_allowance: 10.0

       # Goal checker parameters
       goal_checker:
         plugin: "nav2_controller::SimpleGoalChecker"
         xy_goal_tolerance: 0.25
         yaw_goal_tolerance: 0.25
         stateful: True

       # Controller parameters
       FollowPath:
         plugin: "nav2_rotation_shim::RotationShimController"
         rotational_scaler: 1.5
         linear_scaler: 1.0
         low_speed_linear_scaler: 1.0
         max_angular_accel: 1.0
         max_angular_vel: 0.75
         min_angular_vel: 0.1
         xy_goal_tolerance: 0.1
         trans_stopped_velocity: 0.1
         short_circuit_trajectory_validation: true
         use_cost_regulated_linear_velocity_scaling: false
         cost_scaling_dist: 0.6
         cost_scaling_gain: 1.0
         inflation_cost_scaling_factor: 3.0
         differential_controller:
           plugin: "nav2_controller::SimpleDWBController"
           desired_linear_vel: 0.5
           max_linear_accel: 2.5
           max_linear_decel: 2.5
           max_linear_jerk: 5.0
           max_linear_vel: 0.5
           min_linear_vel: -0.5
           desired_angular_vel: 1.0
           max_angular_accel: 3.2
           max_angular_decel: 3.2
           max_angular_jerk: 6.4
           max_angular_vel: 1.0
           min_angular_vel: -1.0
           min_turn_radius: 0.0
           wheel_separation: 0.4
           cmd_vel_timeout: 0.5
           linear_x_interpolation_size: 4
           use_vel_table: false
           velocity_samples: 10
           vx_samples: 10
           vy_samples: 5
           vtheta_samples: 20
           sim_time: 1.7
           time_step: 0.05
           max_iterations: 10
           tolerance: 0.0
           use_dwa: false
           motion_model_for_prediction: "DiffDrive"
   ```

## Troubleshooting

1. **If Isaac Sim doesn't start**:
   ```bash
   # Check if GPU is available and CUDA is properly configured
   nvidia-smi
   ```

2. **If Nav2 packages are not found**:
   ```bash
   sudo apt update
   sudo apt install ros-humble-navigation2 ros-humble-nav2-bringup
   ```

3. **If ROS commands are not found**:
   ```bash
   source /opt/ros/humble/setup.bash
   source ~/ai_robot_ws/install/setup.bash
   ```

4. **If perception node fails**:
   ```bash
   # Check available topics
   ros2 topic list

   # Check topic info
   ros2 topic info /camera/image_raw
   ```

## Next Steps

1. Complete the full 3-week module following the structured content
2. Practice with the hands-on exercises after each section
3. Experiment with modifying the example code to deepen understanding
4. Explore advanced features in Isaac Sim for more complex AI perception
5. Integrate perception and navigation components for the capstone project