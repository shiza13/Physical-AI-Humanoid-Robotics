# Quickstart Guide: ROS 2 Robotic Nervous System

## Prerequisites

1. **System Requirements**:
   - Ubuntu 22.04 (recommended for ROS 2 Humble)
   - At least 4GB RAM
   - 20GB free disk space
   - Python 3.8 or higher

2. **Install ROS 2 Humble Hawksbill**:
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

4. **Setup ROS Environment**:
   ```bash
   source /opt/ros/humble/setup.bash
   ```

## Setting Up Your Workspace

1. **Create a ROS workspace**:
   ```bash
   mkdir -p ~/ros2_ws/src
   cd ~/ros2_ws
   colcon build
   source install/setup.bash
   ```

2. **Verify Installation**:
   ```bash
   ros2 topic list
   ```

## Running Your First Example

1. **Create a simple publisher/subscriber example**:
   ```bash
   cd ~/ros2_ws/src
   ros2 pkg create --build-type ament_python beginner_tutorials
   ```

2. **Create a simple publisher node** (`~/ros2_ws/src/beginner_tutorials/beginner_tutorials/publisher_member_function.py`):
   ```python
   import rclpy
   from rclpy.node import Node
   from std_msgs.msg import String

   class MinimalPublisher(Node):
       def __init__(self):
           super().__init__('minimal_publisher')
           self.publisher_ = self.create_publisher(String, 'topic', 10)
           timer_period = 0.5  # seconds
           self.timer = self.create_timer(timer_period, self.timer_callback)
           self.i = 0

       def timer_callback(self):
           msg = String()
           msg.data = f'Hello World: {self.i}'
           self.publisher_.publish(msg)
           self.get_logger().info(f'Publishing: "{msg.data}"')
           self.i += 1

   def main(args=None):
       rclpy.init(args=args)
       minimal_publisher = MinimalPublisher()
       rclpy.spin(minimal_publisher)
       minimal_publisher.destroy_node()
       rclpy.shutdown()

   if __name__ == '__main__':
       main()
   ```

3. **Create a simple subscriber node** (`~/ros2_ws/src/beginner_tutorials/beginner_tutorials/subscriber_member_function.py`):
   ```python
   import rclpy
   from rclpy.node import Node
   from std_msgs.msg import String

   class MinimalSubscriber(Node):
       def __init__(self):
           super().__init__('minimal_subscriber')
           self.subscription = self.create_subscription(
               String,
               'topic',
               self.listener_callback,
               10)
           self.subscription  # prevent unused variable warning

       def listener_callback(self, msg):
           self.get_logger().info(f'I heard: "{msg.data}"')

   def main(args=None):
       rclpy.init(args=args)
       minimal_subscriber = MinimalSubscriber()
       rclpy.spin(minimal_subscriber)
       minimal_subscriber.destroy_node()
       rclpy.shutdown()

   if __name__ == '__main__':
       main()
   ```

4. **Update setup.py** (`~/ros2_ws/src/beginner_tutorials/setup.py`):
   ```python
   from setuptools import find_packages, setup

   package_name = 'beginner_tutorials'

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
       description='Beginner tutorials for ROS 2',
       license='Apache-2.0',
       tests_require=['pytest'],
       entry_points={
           'console_scripts': [
               'talker = beginner_tutorials.publisher_member_function:main',
               'listener = beginner_tutorials.subscriber_member_function:main',
           ],
       },
   )
   ```

5. **Build and run the example**:
   ```bash
   cd ~/ros2_ws
   colcon build --packages-select beginner_tutorials
   source install/setup.bash

   # Terminal 1: Run the publisher
   ros2 run beginner_tutorials talker

   # Terminal 2: Run the subscriber
   ros2 run beginner_tutorials listener
   ```

## Simple 6-DOF Humanoid URDF

1. **Create URDF file** (`~/ros2_ws/src/beginner_tutorials/urdf/simple_humanoid.urdf`):
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

2. **Load the URDF in RViz**:
   ```bash
   # Terminal 1: Start RViz
   rviz2

   # Terminal 2: Publish the robot description
   ros2 run robot_state_publisher robot_state_publisher --ros-args -p robot_description:=$(cat ~/ros2_ws/src/beginner_tutorials/urdf/simple_humanoid.urdf)
   ```

## Launch File Example

1. **Create a launch file** (`~/ros2_ws/src/beginner_tutorials/launch/simple_launch.py`):
   ```python
   from launch import LaunchDescription
   from launch_ros.actions import Node
   from ament_index_python.packages import get_package_share_directory
   import os

   def generate_launch_description():
       return LaunchDescription([
           Node(
               package='beginner_tutorials',
               executable='talker',
               name='talker',
               parameters=[
                   {'param_name': 'param_value'}
               ],
               output='screen'
           ),
           Node(
               package='beginner_tutorials',
               executable='listener',
               name='listener',
               output='screen'
           )
       ])
   ```

2. **Run the launch file**:
   ```bash
   ros2 launch beginner_tutorials simple_launch.py
   ```

## Troubleshooting

1. **If ROS commands are not found**:
   ```bash
   source /opt/ros/humble/setup.bash
   ```

2. **If Python modules are not found**:
   ```bash
   source ~/ros2_ws/install/setup.bash
   ```

3. **If Gazebo doesn't start**:
   - Check that you have a display available
   - Try running with `gazebo --verbose` to see detailed errors

## Next Steps

1. Complete the full 5-week module following the structured content
2. Practice with the hands-on exercises after each section
3. Experiment with modifying the example code to deepen understanding
4. Explore additional ROS 2 tutorials and documentation