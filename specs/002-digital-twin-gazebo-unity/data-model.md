# Data Model: Module 2 - The Digital Twin (Gazebo & Unity)

## Overview
This document defines the key data structures and models used in the Digital Twin module for robotics simulation. It covers the data formats for robot models, sensor data, simulation states, and communication protocols between Gazebo and Unity environments.

## Core Data Structures

### 1. Robot Model Data

#### 1.1 URDF/SDF Robot Definition
```xml
<robot name="humanoid_robot">
  <!-- Links define rigid bodies -->
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

  <!-- Joints connect links -->
  <joint name="joint_name" type="revolute">
    <parent link="parent_link"/>
    <child link="child_link"/>
    <origin xyz="0 0 0.25" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
    <limit lower="-1.57" upper="1.57" effort="100" velocity="1"/>
  </joint>
</robot>
```

#### 1.2 Robot State Message (ROS 2)
```yaml
# std_msgs/Header header
  stamp:  # time
    sec: 123456
    nanosec: 789012345
  frame_id: "base_link"
# string[] name
  - "joint1"
  - "joint2"
  - "joint3"
# float64[] position
  - 0.5
  - -0.3
  - 1.2
# float64[] velocity
  - 0.1
  - -0.05
  - 0.2
# float64[] effort
  - 5.0
  - 3.2
  - 7.8
```

### 2. Sensor Data Models

#### 2.1 LiDAR Scan Data (sensor_msgs/LaserScan)
```yaml
# std_msgs/Header header
  stamp:  # time
    sec: 123456
    nanosec: 789012345
  frame_id: "laser_frame"
# float32 angle_min: -1.5708  # -90 degrees
# float32 angle_max: 1.5708   # 90 degrees
# float32 angle_increment: 0.01745  # 1 degree
# float32 time_increment: 0.0
# float32 scan_time: 0.0
# float32 range_min: 0.1
# float32 range_max: 10.0
# float32[] ranges: [2.0, 2.1, 2.05, ...]  # distances in meters
# float32[] intensities: [100.0, 95.0, 102.0, ...]  # optional
```

#### 2.2 Depth Camera Data (sensor_msgs/Image + sensor_msgs/CameraInfo)
```yaml
# Image message for depth data
# std_msgs/Header header
  stamp:  # time
    sec: 123456
    nanosec: 789012345
  frame_id: "camera_frame"
# string encoding: "16UC1"  # 16-bit unsigned integers for depth
# uint8[] data: [4500, 4600, 4700, ...]  # depth values in mm
# uint32 height: 480
# uint32 width: 640
# bool is_bigendian: false
# uint32 step: 1280  # bytes per row

# CameraInfo for intrinsic parameters
# std_msgs/Header header
  stamp:  # time
    sec: 123456
    nanosec: 789012345
  frame_id: "camera_frame"
# uint32 height: 480
# uint32 width: 640
# string distortion_model: "plumb_bob"
# float64[] d: [0.0, 0.0, 0.0, 0.0, 0.0]  # distortion coefficients
# float64[9] k: [525.0, 0.0, 319.5, 0.0, 525.0, 239.5, 0.0, 0.0, 1.0]  # intrinsic matrix
# float64[9] r: [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]  # rectification matrix
# float64[12] p: [525.0, 0.0, 319.5, 0.0, 0.0, 525.0, 239.5, 0.0, 0.0, 0.0, 1.0, 0.0]  # projection matrix
```

#### 2.3 IMU Data (sensor_msgs/Imu)
```yaml
# std_msgs/Header header
  stamp:  # time
    sec: 123456
    nanosec: 789012345
  frame_id: "imu_frame"
# geometry_msgs/Quaternion orientation
  x: 0.0
  y: 0.0
  z: 0.0
  w: 1.0
# float64[9] orientation_covariance: [-1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
# geometry_msgs/Vector3 angular_velocity
  x: 0.01
  y: -0.02
  z: 0.03
# float64[9] angular_velocity_covariance: [-1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
# geometry_msgs/Vector3 linear_acceleration
  x: 0.1
  y: 0.2
  z: 9.8
# float64[9] linear_acceleration_covariance: [-1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
```

### 3. Simulation State Models

#### 3.1 Gazebo Model State
```yaml
# string model_name: "simple_humanoid"
# geometry_msgs/Pose pose
  position:
    x: 1.0
    y: 2.0
    z: 0.5
  orientation:
    x: 0.0
    y: 0.0
    z: 0.0
    w: 1.0
# geometry_msgs/Twist twist
  linear:
    x: 0.1
    y: 0.0
    z: 0.0
  angular:
    x: 0.0
    y: 0.0
    z: 0.1
# string reference_frame: "world"
```

#### 3.2 Simulation Parameters
```yaml
# Physics parameters
physics:
  type: "ode"  # or "bullet", "simbody"
  max_step_size: 0.001
  real_time_factor: 1.0
  max_contacts: 20
  gravity:
    x: 0.0
    y: 0.0
    z: -9.8

# Performance parameters
performance:
  update_rate: 1000.0  # Hz
  threads: 4
  solver:
    type: "quick"
    iters: 100
    sor: 1.3

# Visualization parameters
visualization:
  paused: false
  step_time: 0.01
  record_video: false
```

### 4. Bridge Communication Models

#### 4.1 Bridge Configuration
```yaml
# Bridge connection parameters
bridge_config:
  gazebo_to_unity:
    enabled: true
    frequency: 60  # Hz
    data_types: ["pose", "sensor_data", "joint_state"]
    compression: "none"  # or "lz4", "zstd"

  unity_to_gazebo:
    enabled: true
    frequency: 60  # Hz
    data_types: ["control_commands", "user_input"]

  # Network settings
  network:
    protocol: "tcp"
    address: "127.0.0.1"
    port: 9090
    timeout: 5.0  # seconds
```

#### 4.2 Synchronization Messages
```yaml
# Synchronization message structure
synchronization:
  timestamp:  # ROS time
    sec: 123456
    nanosec: 789012345
  simulation_time: 123.456  # seconds in simulation
  frame_number: 7410  # Unity frame number
  sequence_id: 12345  # message sequence
  environment_state:
    robot_pose:
      position: {x: 1.0, y: 2.0, z: 0.5}
      orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}
    robot_twist:
      linear: {x: 0.1, y: 0.0, z: 0.0}
      angular: {x: 0.0, y: 0.0, z: 0.1}
    sensor_data:
      lidar_ranges: [2.0, 2.1, 2.05, ...]
      imu_orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}
      camera_depth: [4500, 4600, 4700, ...]
```

### 5. Unity-Specific Data Models

#### 5.1 Unity Robot Representation
```csharp
// Robot representation in Unity
public class RobotModel
{
    public string robotName;
    public List<Joint> joints;
    public List<Sensor> sensors;
    public Transform baseTransform;
    public Dictionary<string, Transform> linkTransforms;
    public Dictionary<string, float> jointPositions;
    public Dictionary<string, float> jointVelocities;
    public Dictionary<string, float> jointEfforts;
}

// Joint data structure
public class Joint
{
    public string name;
    public JointType type;  // REVOLUTE, PRISMATIC, FIXED, etc.
    public Transform transform;
    public float position;
    public float velocity;
    public float effort;
    public float lowerLimit;
    public float upperLimit;
    public float maxEffort;
    public float maxVelocity;
}

// Sensor data structure
public class Sensor
{
    public string name;
    public SensorType type;  // LIDAR, CAMERA, IMU, etc.
    public Transform transform;
    public string rosTopic;
    public bool isActive;
    public float updateRate;
}
```

#### 5.2 Unity Scene Configuration
```yaml
# Unity scene configuration
scene_config:
  name: "digital_twin_scene"
  lighting:
    ambient_intensity: 1.0
    skybox: "default_skybox"
    shadows: "soft"
  camera:
    position: {x: 5.0, y: 5.0, z: 5.0}
    target: {x: 0.0, y: 0.0, z: 0.0}
    field_of_view: 60
  rendering:
    quality: "high"
    antialiasing: "fxaa"
    post_processing: true
  physics:
    gravity: {x: 0.0, y: -9.81, z: 0.0}
    fixed_timestep: 0.02
```

### 6. Educational Data Models

#### 6.1 Learning Progress Tracking
```yaml
# Student progress data
student_progress:
  student_id: "student_001"
  module: "digital_twin"
  current_section: "gazebo_basics"
  completion_percentage: 25.0
  exercises_completed:
    - "setup_gazebo_environment"
    - "load_robot_model"
  exercises_pending:
    - "simulate_physics"
    - "integrate_sensors"
  assessment_scores:
    - exercise: "setup_gazebo_environment"
      score: 100.0
      timestamp: 123456.789
    - exercise: "load_robot_model"
      score: 95.0
      timestamp: 123457.123
```

#### 6.2 Exercise Configuration
```yaml
# Exercise configuration data
exercise:
  id: "exercise_001"
  title: "Setup Gazebo Environment"
  description: "Install and configure Gazebo simulation environment"
  objectives:
    - "Install Gazebo"
    - "Verify installation"
    - "Launch basic simulation"
  steps:
    - command: "sudo apt install ros-humble-gazebo-*"
      description: "Install Gazebo packages"
      expected_output: "Successfully installed"
    - command: "gazebo --version"
      description: "Check Gazebo version"
      expected_output: "Gazebo X.X.X"
  validation_criteria:
    - "Gazebo window opens successfully"
    - "Simulation runs in real-time"
  difficulty: "beginner"
  estimated_time: 45  # minutes
```

## Data Flow Patterns

### 1. Sensor Data Flow
```
Physical Robot (Real/Emulated)
    ↓
Gazebo Simulation
    ↓
ROS 2 Topics (sensor_msgs/*)
    ↓
Unity Visualization (via bridge)
    ↓
Student Interface
```

### 2. Control Command Flow
```
Student Input (Keyboard/Script)
    ↓
Unity Interface
    ↓
ROS 2 Topics (geometry_msgs/Twist, etc.)
    ↓
Gazebo Simulation
    ↓
Robot Actuation
```

### 3. State Synchronization Flow
```
Gazebo Physics Engine
    ↓
Model States (gazebo_msgs/ModelState)
    ↓
Bridge Node
    ↓
Unity Transform Updates
    ↓
Synchronized Visualization
```

## Validation Criteria

### 1. Data Integrity
- All sensor messages must conform to ROS message standards
- Robot state data must maintain consistency across environments
- Time synchronization must be maintained within acceptable thresholds
- Data types must match expected formats in both Gazebo and Unity

### 2. Performance Requirements
- Sensor data must be published at required frequencies
- State updates must occur within latency constraints
- Bridge communication must maintain stable connection
- Visualization must maintain target frame rates

### 3. Educational Validity
- Data structures must support learning objectives
- Messages must be accessible to students with appropriate documentation
- Validation mechanisms must provide clear feedback
- Progress tracking must accurately reflect student achievement