# Quickstart Guide: Module 2 - The Digital Twin (Gazebo & Unity)

## Prerequisites

1. **System Requirements**:
   - Ubuntu 22.04 (recommended) or equivalent Linux distribution
   - At least 8GB RAM (16GB recommended for Unity)
   - 30GB free disk space
   - Python 3.8 or higher
   - Graphics card supporting OpenGL 3.3+ (for Unity)

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

3. **Install Gazebo**:
   ```bash
   sudo apt install ros-humble-gazebo-*
   ```

4. **Install Unity Hub and Unity Editor**:
   - Download Unity Hub from https://unity.com/download
   - Install Unity Hub and use it to install Unity 2022.3 LTS or newer

## Setting Up Your Digital Twin Workspace

1. **Create a ROS workspace for digital twin projects**:
   ```bash
   mkdir -p ~/digital_twin_ws/src
   cd ~/digital_twin_ws
   colcon build
   source install/setup.bash
   ```

2. **Create a digital twin package**:
   ```bash
   cd ~/digital_twin_ws/src
   ros2 pkg create --build-type ament_python digital_twin_examples
   ```

3. **Verify Installation**:
   ```bash
   # Check that Gazebo is available
   gazebo --version

   # Check ROS 2 setup
   ros2 topic list
   ```

## Running Your First Digital Twin Example

1. **Create a simple humanoid robot model** (`~/digital_twin_ws/src/digital_twin_examples/digital_twin_examples/simple_humanoid.urdf`):
   ```xml
   <?xml version="1.0"?>
   <robot name="simple_humanoid">
     <!-- Base Link -->
     <link name="base_link">
       <visual>
         <geometry>
           <box size="0.3 0.1 0.3"/>
         </geometry>
         <material name="blue">
           <color rgba="0 0 1 1"/>
         </material>
       </visual>
       <collision>
         <geometry>
           <box size="0.3 0.1 0.3"/>
         </geometry>
       </collision>
       <inertial>
         <mass value="1"/>
         <inertia ixx="1" ixy="0" ixz="0" iyy="1" iyz="0" izz="1"/>
       </inertial>
     </link>

     <!-- Torso -->
     <link name="torso">
       <visual>
         <geometry>
           <box size="0.2 0.4 0.15"/>
         </geometry>
         <material name="red">
           <color rgba="1 0 0 1"/>
         </material>
       </visual>
       <collision>
         <geometry>
           <box size="0.2 0.4 0.15"/>
         </geometry>
       </collision>
       <inertial>
         <mass value="2"/>
         <inertia ixx="1" ixy="0" ixz="0" iyy="1" iyz="0" izz="1"/>
       </inertial>
     </link>

     <!-- Head -->
     <link name="head">
       <visual>
         <geometry>
           <sphere radius="0.1"/>
         </geometry>
         <material name="white">
           <color rgba="1 1 1 1"/>
         </material>
       </visual>
       <collision>
         <geometry>
           <sphere radius="0.1"/>
         </geometry>
       </collision>
       <inertial>
         <mass value="0.5"/>
         <inertia ixx="1" ixy="0" ixz="0" iyy="1" iyz="0" izz="1"/>
       </inertial>
     </link>

     <!-- Left Leg -->
     <link name="left_leg">
       <visual>
         <geometry>
           <box size="0.1 0.4 0.1"/>
         </geometry>
         <material name="green">
           <color rgba="0 1 0 1"/>
         </material>
       </visual>
       <collision>
         <geometry>
           <box size="0.1 0.4 0.1"/>
         </geometry>
       </collision>
       <inertial>
         <mass value="1"/>
         <inertia ixx="1" ixy="0" ixz="0" iyy="1" iyz="0" izz="1"/>
       </inertial>
     </link>

     <!-- Right Leg -->
     <link name="right_leg">
       <visual>
         <geometry>
           <box size="0.1 0.4 0.1"/>
         </geometry>
         <material name="green">
           <color rgba="0 1 0 1"/>
         </material>
       </visual>
       <collision>
         <geometry>
           <box size="0.1 0.4 0.1"/>
         </geometry>
       </collision>
       <inertial>
         <mass value="1"/>
         <inertia ixx="1" ixy="0" ixz="0" iyy="1" iyz="0" izz="1"/>
       </inertial>
     </link>

     <!-- Joints -->
     <joint name="base_to_torso" type="fixed">
       <parent link="base_link"/>
       <child link="torso"/>
       <origin xyz="0 0 0.25"/>
     </joint>

     <joint name="torso_to_head" type="fixed">
       <parent link="torso"/>
       <child link="head"/>
       <origin xyz="0 0 0.3"/>
     </joint>

     <joint name="base_to_left_leg" type="fixed">
       <parent link="base_link"/>
       <child link="left_leg"/>
       <origin xyz="-0.1 0 -0.3"/>
     </joint>

     <joint name="base_to_right_leg" type="fixed">
       <parent link="base_link"/>
       <child link="right_leg"/>
       <origin xyz="0.1 0 -0.3"/>
     </joint>
   </robot>
   ```

2. **Create a simple sensor simulation** (`~/digital_twin_ws/src/digital_twin_examples/digital_twin_examples/sensor_simulator.py`):
   ```python
   import rclpy
   from rclpy.node import Node
   from sensor_msgs.msg import LaserScan, Image, Imu
   import math
   import random

   class SensorSimulator(Node):
       def __init__(self):
           super().__init__('sensor_simulator')

           # Create publishers for different sensor types
           self.lidar_publisher = self.create_publisher(LaserScan, 'scan', 10)
           self.camera_publisher = self.create_publisher(Image, 'camera/image_raw', 10)
           self.imu_publisher = self.create_publisher(Imu, 'imu/data', 10)

           # Timer for sensor data publishing
           timer_period = 0.1  # seconds
           self.timer = self.create_timer(timer_period, self.publish_sensor_data)

       def publish_sensor_data(self):
           # Publish simulated LiDAR data
           lidar_msg = LaserScan()
           lidar_msg.header.stamp = self.get_clock().now().to_msg()
           lidar_msg.header.frame_id = 'laser_frame'
           lidar_msg.angle_min = -math.pi / 2
           lidar_msg.angle_max = math.pi / 2
           lidar_msg.angle_increment = math.pi / 180  # 1 degree
           lidar_msg.range_min = 0.1
           lidar_msg.range_max = 10.0

           # Generate simulated ranges (simple obstacle detection)
           ranges = []
           for i in range(181):  # 181 points for 180 degrees
               # Simulate an obstacle at 2 meters in front
               if 80 <= i <= 100:  # Front-facing sensors
                   ranges.append(2.0 + random.uniform(-0.1, 0.1))
               else:
                   ranges.append(8.0 + random.uniform(-0.5, 0.5))

           lidar_msg.ranges = ranges
           self.lidar_publisher.publish(lidar_msg)

           # Publish simulated IMU data
           imu_msg = Imu()
           imu_msg.header.stamp = self.get_clock().now().to_msg()
           imu_msg.header.frame_id = 'imu_frame'
           # Simulate slight orientation changes
           imu_msg.orientation.x = random.uniform(-0.01, 0.01)
           imu_msg.orientation.y = random.uniform(-0.01, 0.01)
           imu_msg.orientation.z = random.uniform(-0.01, 0.01)
           imu_msg.orientation.w = 1.0  # Simplified
           # Simulate small accelerations
           imu_msg.linear_acceleration.x = random.uniform(-0.1, 0.1)
           imu_msg.linear_acceleration.y = random.uniform(-0.1, 0.1)
           imu_msg.linear_acceleration.z = 9.8 + random.uniform(-0.1, 0.1)
           self.imu_publisher.publish(imu_msg)

           self.get_logger().info('Published simulated sensor data')

   def main(args=None):
       rclpy.init(args=args)
       sensor_simulator = SensorSimulator()

       try:
           rclpy.spin(sensor_simulator)
       except KeyboardInterrupt:
           pass
       finally:
           sensor_simulator.destroy_node()
           rclpy.shutdown()

   if __name__ == '__main__':
       main()
   ```

3. **Update setup.py** (`~/digital_twin_ws/src/digital_twin_examples/setup.py`):
   ```python
   from setuptools import find_packages, setup

   package_name = 'digital_twin_examples'

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
       description='Digital twin examples for ROS 2',
       license='Apache-2.0',
       tests_require=['pytest'],
       entry_points={
           'console_scripts': [
               'sensor_simulator = digital_twin_examples.sensor_simulator:main',
           ],
       },
   )
   ```

4. **Build and run the sensor simulation**:
   ```bash
   cd ~/digital_twin_ws
   colcon build --packages-select digital_twin_examples
   source install/setup.bash

   # Run the sensor simulator
   ros2 run digital_twin_examples sensor_simulator
   ```

## Setting up Gazebo Simulation

1. **Create a simple Gazebo world** (`~/digital_twin_ws/src/digital_twin_examples/worlds/simple_world.world`):
   ```xml
   <?xml version="1.0" ?>
   <sdf version="1.6">
     <world name="simple_world">
       <include>
         <uri>model://ground_plane</uri>
       </include>
       <include>
         <uri>model://sun</uri>
       </include>

       <!-- Simple box obstacle -->
       <model name="box_obstacle">
         <pose>2 0 0.5 0 0 0</pose>
         <link name="link">
           <collision name="collision">
             <geometry>
               <box>
                 <size>1 1 1</size>
               </box>
             </geometry>
           </collision>
           <visual name="visual">
             <geometry>
               <box>
                 <size>1 1 1</size>
               </box>
             </geometry>
             <material>
               <ambient>0.5 0.5 0.5 1</ambient>
               <diffuse>0.5 0.5 0.5 1</diffuse>
             </material>
           </visual>
         </link>
       </model>
     </world>
   </sdf>
   ```

2. **Launch Gazebo with your robot**:
   ```bash
   # Terminal 1: Start Gazebo with the simple world
   gazebo ~/digital_twin_ws/src/digital_twin_examples/worlds/simple_world.world

   # Terminal 2: Spawn your robot model
   ros2 run gazebo_ros spawn_entity.py -entity simple_humanoid -file ~/digital_twin_ws/src/digital_twin_examples/digital_twin_examples/simple_humanoid.urdf -x 0 -y 0 -z 1
   ```

## Unity Setup for Digital Twin Visualization

1. **Create a new Unity project**:
   - Open Unity Hub
   - Create a new 3D project named "DigitalTwinVisualization"
   - Import the robot model (convert URDF to FBX format if needed)

2. **Set up basic scene**:
   - Add a plane as the ground
   - Import your humanoid robot model
   - Set up lighting and camera for visualization

3. **For connecting Unity with ROS 2**:
   - Install ROS# Unity package from the Unity Asset Store
   - Configure the ROS bridge to connect Unity with your ROS 2 system

## Troubleshooting

1. **If Gazebo doesn't start**:
   ```bash
   # Check if GPU is available
   glxinfo | grep "OpenGL renderer"

   # Try running with software rendering
   export LIBGL_ALWAYS_SOFTWARE=1
   gazebo
   ```

2. **If Unity won't start**:
   - Ensure your graphics drivers are up to date
   - Check that your system meets Unity's minimum requirements
   - Try running Unity in safe mode if experiencing crashes

3. **If ROS commands are not found**:
   ```bash
   source /opt/ros/humble/setup.bash
   source ~/digital_twin_ws/install/setup.bash
   ```

4. **If sensor simulation is not publishing data**:
   ```bash
   # Check available topics
   ros2 topic list

   # Check topic info
   ros2 topic info /scan
   ```

## Next Steps

1. Complete the full 5-week module following the structured content
2. Practice with the hands-on exercises after each section
3. Experiment with modifying the example code to deepen understanding
4. Explore advanced features in Gazebo and Unity for more complex simulations
5. Learn about connecting Gazebo and Unity through ROS 2 bridges