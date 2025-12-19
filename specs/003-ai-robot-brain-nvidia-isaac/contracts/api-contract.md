# API Contract: Module 3 - The AI-Robot Brain (NVIDIA Isaac)

## Overview
This contract defines the interfaces, data structures, and communication protocols for the AI-Robot Brain module. It specifies the APIs for perception, navigation, and simulation using the NVIDIA Isaac platform.

## 1. Perception APIs

### 1.1 Camera Data API
**Purpose**: Interface for accessing camera data from Isaac Sim

#### ROS 2 Topic: `/camera/image_raw`
- **Message Type**: `sensor_msgs/Image`
- **Description**: Publishes raw RGB image data from simulated camera
- **Message Format**:
  ```yaml
  header:
    stamp: {sec: 123456, nanosec: 789012345}
    frame_id: "camera_frame"
  height: 480
  width: 640
  encoding: "rgb8"
  data: [byte array of image data]
  ```

#### ROS 2 Topic: `/camera/depth/image_raw`
- **Message Type**: `sensor_msgs/Image`
- **Description**: Publishes depth image data from simulated depth camera
- **Message Format**:
  ```yaml
  header:
    stamp: {sec: 123456, nanosec: 789012345}
    frame_id: "depth_camera_frame"
  height: 480
  width: 640
  encoding: "16UC1"  # 16-bit unsigned integers for depth
  data: [4500, 4600, 4700, ...]  # depth values in mm
  ```

### 1.2 Object Detection API
**Purpose**: Interface for AI-powered object detection in simulation

#### ROS 2 Topic: `/perception/detections`
- **Message Type**: `vision_msgs/Detection2DArray`
- **Description**: Publishes detected objects with bounding boxes
- **Message Format**:
  ```yaml
  header:
    stamp: {sec: 123456, nanosec: 789012345}
    frame_id: "camera_frame"
  detections:
    - results:
        - hypothesis: "person"
          score: 0.95
      bbox:
        center:
          x: 320
          y: 240
          theta: 0.0
        size_x: 100
        size_y: 200
      id: "detection_001"
  ```

### 1.3 Semantic Segmentation API
**Purpose**: Interface for pixel-level scene understanding

#### ROS 2 Topic: `/perception/semantic_segmentation`
- **Message Type**: `sensor_msgs/Image`
- **Description**: Publishes semantic segmentation mask
- **Message Format**:
  ```yaml
  header:
    stamp: {sec: 123456, nanosec: 789012345}
    frame_id: "camera_frame"
  height: 480
  width: 640
  encoding: "32SC1"  # 32-bit signed integers for class IDs
  data: [1, 1, 2, 3, 1, ...]  # class ID for each pixel
  ```

## 2. Navigation APIs

### 2.1 Navigation Goal API
**Purpose**: Interface for setting navigation goals for the robot

#### ROS 2 Action: `/navigate_to_pose`
- **Action Type**: `nav2_msgs/action/NavigateToPose`
- **Description**: Navigate robot to specified pose
- **Goal Format**:
  ```yaml
  pose:
    header:
      stamp: {sec: 0, nanosec: 0}
      frame_id: "map"
    pose:
      position: {x: 5.0, y: 3.0, z: 0.0}
      orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}
  ```

#### ROS 2 Service: `/clear_costmap`
- **Service Type**: `std_srvs/srv/Empty`
- **Description**: Clear navigation costmap
- **Request**: Empty
- **Response**: Empty

### 2.2 Path Planning API
**Purpose**: Interface for path planning and execution

#### ROS 2 Topic: `/plan`
- **Message Type**: `nav_msgs/Path`
- **Description**: Publishes computed navigation path
- **Message Format**:
  ```yaml
  header:
    stamp: {sec: 123456, nanosec: 789012345}
    frame_id: "map"
  poses:
    - header:
        stamp: {sec: 123456, nanosec: 789012345}
        frame_id: "map"
      pose:
        position: {x: 0.0, y: 0.0, z: 0.0}
        orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}
    - header:
        stamp: {sec: 123456, nanosec: 789022345}
        frame_id: "map"
      pose:
        position: {x: 1.0, y: 0.5, z: 0.0}
        orientation: {x: 0.0, y: 0.0, z: 0.1, w: 0.99}
  ```

#### ROS 2 Topic: `/local_costmap/costmap`
- **Message Type**: `nav_msgs/OccupancyGrid`
- **Description**: Publishes local costmap for obstacle avoidance
- **Message Format**:
  ```yaml
  header:
    stamp: {sec: 123456, nanosec: 789012345}
    frame_id: "map"
  info:
    map_load_time: {sec: 123456, nanosec: 0}
    resolution: 0.05  # meters per pixel
    width: 200  # pixels
    height: 200  # pixels
    origin:
      position: {x: -5.0, y: -5.0, z: 0.0}
      orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}
  data: [0, 0, 0, 100, -1, 0, ...]  # 0=free, 100=occupied, -1=unknown
  ```

## 3. VSLAM APIs

### 3.1 Localization API
**Purpose**: Interface for robot localization using visual SLAM

#### ROS 2 Topic: `/amcl_pose`
- **Message Type**: `geometry_msgs/PoseWithCovarianceStamped`
- **Description**: Publishes estimated robot pose with uncertainty
- **Message Format**:
  ```yaml
  header:
    stamp: {sec: 123456, nanosec: 789012345}
    frame_id: "map"
  pose:
    pose:
      position: {x: 2.5, y: 1.8, z: 0.0}
      orientation: {x: 0.0, y: 0.0, z: 0.1, w: 0.99}
    covariance: [0.1, 0.0, 0.0, 0.0, 0.0, 0.0,  # 36-element covariance matrix
                 0.0, 0.1, 0.0, 0.0, 0.0, 0.0,
                 0.0, 0.0, 0.1, 0.0, 0.0, 0.0,
                 0.0, 0.0, 0.0, 0.1, 0.0, 0.0,
                 0.0, 0.0, 0.0, 0.0, 0.1, 0.0,
                 0.0, 0.0, 0.0, 0.0, 0.0, 0.1]
  ```

#### ROS 2 Topic: `/tf` and `/tf_static`
- **Message Type**: `tf2_msgs/TFMessage`
- **Description**: Publishes coordinate frame transformations
- **Message Format**:
  ```yaml
  transforms:
    - header:
        stamp: {sec: 123456, nanosec: 789012345}
        frame_id: "odom"
      child_frame_id: "base_link"
      transform:
        translation: {x: 2.5, y: 1.8, z: 0.0}
        rotation: {x: 0.0, y: 0.0, z: 0.1, w: 0.99}
  ```

### 3.2 Map API
**Purpose**: Interface for map management in SLAM

#### ROS 2 Service: `/map_server/load_map`
- **Service Type**: `nav2_msgs/srv/LoadMap`
- **Description**: Load a pre-built map
- **Request**:
  ```yaml
  map_url: "/path/to/map.yaml"
  ```
- **Response**:
  ```yaml
  result: 1  # SUCCESS
  ```

#### ROS 2 Topic: `/map`
- **Message Type**: `nav_msgs/OccupancyGrid`
- **Description**: Publishes global map
- **Message Format**: Same as costmap but for global map

## 4. Isaac Sim APIs

### 4.1 Simulation Control API
**Purpose**: Interface for controlling Isaac Sim simulation

#### ROS 2 Service: `/reset_simulation`
- **Service Type**: `std_srvs/srv/Empty`
- **Description**: Reset the simulation to initial state
- **Request**: Empty
- **Response**: Empty

#### ROS 2 Topic: `/clock`
- **Message Type**: `rosgraph_msgs/Clock`
- **Description**: Publishes simulation time
- **Message Format**:
  ```yaml
  clock:
    sec: 123456
    nanosec: 789012345
  ```

### 4.2 Synthetic Data Generation API
**Purpose**: Interface for synthetic dataset creation

#### ROS 2 Service: `/synthetic_data/generate_dataset`
- **Service Type**: `std_srvs/srv/Trigger`
- **Description**: Generate synthetic training dataset
- **Request**: Empty
- **Response**:
  ```yaml
  success: true
  message: "Dataset generated successfully"
  ```

#### ROS 2 Topic: `/synthetic_data/metadata`
- **Message Type**: Custom message (isaac_robot_msgs/Metadata)
- **Description**: Publishes metadata for generated synthetic data
- **Message Format**:
  ```yaml
  dataset_name: "isaac_sim_perception_dataset"
  version: "1.0"
  creation_date: "2025-12-17"
  scene_config:
    lighting: "outdoor_day"
    weather: "clear"
    objects: ["person", "box", "table"]
  annotation_types:
    - "bounding_box"
    - "semantic_segmentation"
    - "depth"
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
    "title": "Introduction to Isaac Platform",
    "description": "Learn about NVIDIA Isaac Sim and Isaac ROS capabilities",
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
    "module": "ai_robot_brain",
    "current_section": "vslam_navigation",
    "completion_percentage": 40.0,
    "exercises_completed": 2,
    "total_exercises": 5,
    "last_activity": "2025-12-17T10:30:00Z"
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
        "answer": "VSLAM combines visual input with sensor data to estimate robot pose..."
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
    "feedback": "Good understanding of VSLAM concepts"
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
    "timestamp": "2025-12-17T10:30:00Z"
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
- **Perception Requests**: < 100ms for processing
- **Navigation Commands**: < 50ms for command acceptance
- **State Queries**: < 20ms for pose/position requests
- **Simulation Control**: < 10ms for basic commands

### 7.2 Data Throughput
- **Camera Data Rate**: 15-30 Hz for RGB images
- **Depth Data Rate**: 15-30 Hz for depth images
- **Navigation Updates**: 10-20 Hz for path planning
- **Localization Updates**: 50-100 Hz for pose estimation

### 7.3 Resource Usage
- **Memory**: < 4GB for basic perception pipeline
- **GPU Memory**: < 6GB for Isaac Sim with perception
- **CPU**: < 75% utilization on 8-core system
- **Network**: < 50 Mbps for simulation data

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