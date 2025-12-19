# API Contract: Module 2 - The Digital Twin (Gazebo & Unity)

## Overview
This contract defines the interfaces, data structures, and communication protocols for the Digital Twin module. It specifies the APIs for robot simulation, sensor integration, and visualization between Gazebo and Unity environments.

## 1. Core Simulation APIs

### 1.1 Robot Control API
**Purpose**: Interface for controlling simulated robots in Gazebo environment

#### Endpoints:
```
POST /robot/{robot_name}/cmd_vel
```
- **Description**: Send velocity commands to a robot
- **Request Body**:
  ```json
  {
    "linear": {
      "x": 0.5,
      "y": 0.0,
      "z": 0.0
    },
    "angular": {
      "x": 0.0,
      "y": 0.0,
      "z": 0.2
    }
  }
  ```
- **Response**: 200 OK on successful command execution
- **Error Responses**: 404 if robot not found, 400 for invalid command format

```
GET /robot/{robot_name}/state
```
- **Description**: Get current state of the robot
- **Response**:
  ```json
  {
    "name": "simple_humanoid",
    "position": {
      "x": 1.0,
      "y": 2.0,
      "z": 0.5
    },
    "orientation": {
      "x": 0.0,
      "y": 0.0,
      "z": 0.0,
      "w": 1.0
    },
    "velocity": {
      "linear": {"x": 0.1, "y": 0.0, "z": 0.0},
      "angular": {"x": 0.0, "y": 0.0, "z": 0.1}
    }
  }
  ```
- **Error Responses**: 404 if robot not found

### 1.2 Joint Control API
**Purpose**: Interface for controlling individual robot joints

#### Endpoints:
```
POST /robot/{robot_name}/joint/{joint_name}/position
```
- **Description**: Set target position for a specific joint
- **Request Body**:
  ```json
  {
    "position": 1.57,
    "velocity": 0.5,
    "effort": 10.0
  }
  ```
- **Response**: 200 OK on successful command execution

```
GET /robot/{robot_name}/joints
```
- **Description**: Get all joint states for a robot
- **Response**:
  ```json
  {
    "name": ["joint1", "joint2", "joint3"],
    "position": [0.5, -0.3, 1.2],
    "velocity": [0.1, -0.05, 0.2],
    "effort": [5.0, 3.2, 7.8]
  }
  ```

## 2. Sensor Simulation APIs

### 2.1 LiDAR Sensor API
**Purpose**: Interface for simulated LiDAR sensor data

#### ROS 2 Topic: `/scan`
- **Message Type**: `sensor_msgs/LaserScan`
- **Description**: Publishes laser scan data from simulated LiDAR
- **Message Format**:
  ```yaml
  header:
    stamp: {sec: 123456, nanosec: 789012345}
    frame_id: "laser_frame"
  angle_min: -1.5708  # -90 degrees
  angle_max: 1.5708   # 90 degrees
  angle_increment: 0.01745  # 1 degree
  range_min: 0.1
  range_max: 10.0
  ranges: [2.0, 2.1, 2.05, ...]  # distances in meters
  ```

### 2.2 Camera Sensor API
**Purpose**: Interface for simulated camera sensor data

#### ROS 2 Topics:
- `/camera/image_raw` - Raw RGB image data
- `/camera/depth/image_raw` - Depth image data
- `/camera/camera_info` - Camera intrinsic parameters

**Message Type**: `sensor_msgs/Image`
- **Description**: Publishes RGB and depth image data from simulated camera
- **Message Format**:
  ```yaml
  header:
    stamp: {sec: 123456, nanosec: 789012345}
    frame_id: "camera_frame"
  height: 480
  width: 640
  encoding: "rgb8"  # or "16UC1" for depth
  data: [byte array of image data]
  ```

### 2.3 IMU Sensor API
**Purpose**: Interface for simulated IMU sensor data

#### ROS 2 Topic: `/imu/data`
- **Message Type**: `sensor_msgs/Imu`
- **Description**: Publishes inertial measurement unit data
- **Message Format**:
  ```yaml
  header:
    stamp: {sec: 123456, nanosec: 789012345}
    frame_id: "imu_frame"
  orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}
  angular_velocity: {x: 0.01, y: -0.02, z: 0.03}
  linear_acceleration: {x: 0.1, y: 0.2, z: 9.8}
  ```

## 3. Gazebo-Unity Bridge APIs

### 3.1 Bridge Configuration API
**Purpose**: Interface for configuring and controlling the Gazebo-Unity bridge

#### Endpoints:
```
POST /bridge/configure
```
- **Description**: Configure bridge parameters
- **Request Body**:
  ```json
  {
    "frequency": 60,
    "data_types": ["pose", "sensor_data", "joint_state"],
    "compression": "none",
    "network": {
      "protocol": "tcp",
      "address": "127.0.0.1",
      "port": 9090
    }
  }
  ```
- **Response**: 200 OK on successful configuration

```
POST /bridge/start
```
- **Description**: Start the bridge connection
- **Response**: 200 OK if bridge started successfully
- **Error Responses**: 500 if connection fails

```
POST /bridge/stop
```
- **Description**: Stop the bridge connection
- **Response**: 200 OK if bridge stopped successfully

### 3.2 Synchronization API
**Purpose**: Interface for maintaining synchronization between Gazebo and Unity

#### Endpoints:
```
GET /synchronization/status
```
- **Description**: Get synchronization status between environments
- **Response**:
  ```json
  {
    "gazebo_time": 123.456,
    "unity_time": 123.454,
    "latency": 0.002,
    "sync_status": "synchronized"
  }
  ```

```
POST /synchronization/sync
```
- **Description**: Force synchronization between environments
- **Response**: 200 OK if synchronization successful

## 4. Visualization APIs

### 4.1 Unity Scene Management API
**Purpose**: Interface for managing Unity scenes and visualization

#### Endpoints:
```
POST /unity/scene/load
```
- **Description**: Load a Unity scene
- **Request Body**:
  ```json
  {
    "scene_name": "digital_twin_scene",
    "parameters": {
      "robot_model": "simple_humanoid.urdf",
      "environment": "simple_world.world"
    }
  }
  ```
- **Response**: 200 OK if scene loaded successfully

```
POST /unity/camera/configure
```
- **Description**: Configure Unity camera settings
- **Request Body**:
  ```json
  {
    "position": {"x": 5.0, "y": 5.0, "z": 5.0},
    "target": {"x": 0.0, "y": 0.0, "z": 0.0},
    "field_of_view": 60
  }
  ```

## 5. Educational APIs

### 5.1 Exercise Management API
**Purpose**: Interface for managing educational exercises and student progress

#### Endpoints:
```
GET /exercise/{exercise_id}/status
```
- **Description**: Get status of a specific exercise
- **Response**:
  ```json
  {
    "exercise_id": "exercise_001",
    "title": "Setup Gazebo Environment",
    "description": "Install and configure Gazebo simulation environment",
    "status": "in_progress",
    "completion_percentage": 75.0,
    "steps_completed": 3,
    "total_steps": 4
  }
  ```

```
POST /exercise/{exercise_id}/validate
```
- **Description**: Validate completion of an exercise
- **Response**:
  ```json
  {
    "exercise_id": "exercise_001",
    "valid": true,
    "score": 100.0,
    "feedback": "All validation criteria met successfully"
  }
  ```

```
GET /student/{student_id}/progress
```
- **Description**: Get overall progress for a student
- **Response**:
  ```json
  {
    "student_id": "student_001",
    "module": "digital_twin",
    "current_section": "gazebo_basics",
    "completion_percentage": 25.0,
    "exercises_completed": 2,
    "total_exercises": 8,
    "last_activity": "2025-12-16T10:30:00Z"
  }
  ```

### 5.2 Assessment API
**Purpose**: Interface for educational assessments and evaluations

#### Endpoints:
```
POST /assessment/{assessment_id}/submit
```
- **Description**: Submit an assessment for evaluation
- **Request Body**:
  ```json
  {
    "student_id": "student_001",
    "answers": [
      {
        "question_id": "q1",
        "answer": "Digital twin is a virtual representation..."
      }
    ]
  }
  ```
- **Response**:
  ```json
  {
    "assessment_id": "assessment_001",
    "student_id": "student_001",
    "score": 85.0,
    "grade": "B",
    "feedback": "Good understanding of core concepts"
  }
  ```

## 6. Error Handling and Status Codes

### 6.1 Standard Error Response Format
```json
{
  "error": {
    "code": "ERROR_CODE",
    "message": "Descriptive error message",
    "details": "Additional error details",
    "timestamp": "2025-12-16T10:30:00Z"
  }
}
```

### 6.2 HTTP Status Codes
- **200 OK**: Request successful
- **201 Created**: Resource created successfully
- **400 Bad Request**: Invalid request format or parameters
- **401 Unauthorized**: Authentication required
- **403 Forbidden**: Insufficient permissions
- **404 Not Found**: Resource not found
- **409 Conflict**: Resource conflict (e.g., duplicate names)
- **500 Internal Server Error**: Server error
- **503 Service Unavailable**: Service temporarily unavailable

### 6.3 ROS 2 Error Handling
- **Timeouts**: Implement appropriate timeouts for all ROS 2 service calls
- **Retry Logic**: Include retry mechanisms for transient failures
- **Connection Monitoring**: Monitor ROS 2 node connections and report failures

## 7. Performance Requirements

### 7.1 API Response Times
- **Robot Control Commands**: < 50ms
- **State Queries**: < 20ms
- **Sensor Data Access**: < 10ms
- **Bridge Operations**: < 100ms

### 7.2 Data Throughput
- **Sensor Data Rate**: Match real-world sensor frequencies
  - LiDAR: 5-20 Hz
  - Cameras: 15-30 Hz
  - IMU: 50-100 Hz
- **State Updates**: 60 Hz for smooth visualization

### 7.3 Resource Usage
- **Memory**: < 2GB for basic simulation
- **CPU**: < 75% utilization on 4-core system
- **Network**: < 100 Mbps for bridge communication

## 8. Security Considerations

### 8.1 Authentication
- APIs should implement authentication for multi-user scenarios
- Use ROS 2 security features for sensitive operations
- Validate user permissions for educational access

### 8.2 Data Protection
- Protect student progress and assessment data
- Implement proper data access controls
- Encrypt sensitive communication channels

### 8.3 Simulation Security
- Prevent unauthorized access to simulation controls
- Implement sandboxing for student code execution
- Validate all input data to prevent simulation corruption

## 9. Validation Criteria

### 9.1 API Compliance
- All endpoints must follow RESTful principles where applicable
- All ROS 2 interfaces must use standard message types
- All responses must conform to specified schemas
- Error handling must be consistent across all endpoints

### 9.2 Educational Validity
- APIs must support learning objectives defined in the specification
- Progress tracking must accurately reflect student achievement
- Assessment interfaces must provide meaningful feedback
- Validation mechanisms must confirm learning outcomes

### 9.3 Technical Validity
- All interfaces must be tested and validated
- Performance requirements must be met
- Security requirements must be implemented
- Documentation must be complete and accurate