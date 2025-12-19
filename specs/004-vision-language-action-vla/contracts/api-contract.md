# API Contract: Module 4 - Vision-Language-Action (VLA)

## Overview

This contract defines the interfaces, data structures, and communication protocols for the Vision-Language-Action (VLA) module. It specifies the APIs for voice processing, computer vision, cognitive planning, and integration with robotics systems using Large Language Models.

## 1. Voice Processing APIs

### 1.1 Voice Recognition API

**Purpose**: Interface for speech-to-text conversion using OpenAI Whisper

#### ROS 2 Service: `/voice_recognition/process_audio`
- **Service Type**: Custom service (vla_msgs/srv/ProcessAudio)
- **Description**: Process audio input and return transcribed text
- **Request Format**:
  ```yaml
  audio_data: [byte array of audio data]
  language: "en"  # optional language code
  ```
- **Response Format**:
  ```yaml
  success: true
  transcription: "Move the red ball to the left of the table"
  confidence: 0.92
  language: "en"
  duration: 3.2  # seconds
  ```

#### ROS 2 Topic: `/voice_commands`
- **Message Type**: `std_msgs/String`
- **Description**: Publishes recognized voice commands
- **Message Format**:
  ```yaml
  data: "Move the red ball to the left of the table"
  ```

### 1.2 Command Parsing API

**Purpose**: Interface for parsing natural language commands into structured actions

#### ROS 2 Service: `/command_parser/parse_command`
- **Service Type**: Custom service (vla_msgs/srv/ParseCommand)
- **Description**: Parse natural language command into structured format
- **Request Format**:
  ```yaml
  command: "Move the red ball to the left of the table"
  context: "indoor_environment"  # optional context
  ```
- **Response Format**:
  ```yaml
  success: true
  intent: "move_object"
  action: "move"
  target_object:
    name: "ball"
    color: "red"
    confidence: 0.95
  destination:
    reference_object: "table"
    spatial_relationship: "left_of"
    confidence: 0.85
  parameters:
    speed: "normal"
    precision: "high"
  ```

#### ROS 2 Topic: `/parsed_commands`
- **Message Type**: Custom message (vla_msgs/ParsedCommand)
- **Description**: Publishes structured command representations
- **Message Format**:
  ```yaml
  command_id: "cmd_001"
  command_type: "navigation"
  entities:
    - type: "object"
      value: "red ball"
      confidence: 0.95
      reference_id: "obj_001"
    - type: "location"
      value: "left of table"
      confidence: 0.85
      reference_id: "loc_001"
  relations:
    - type: "spatial"
      from: "obj_001"
      to: "loc_001"
      relation: "left_of"
  ```

## 2. Computer Vision APIs

### 2.1 Object Detection API

**Purpose**: Interface for detecting and recognizing objects in visual input

#### ROS 2 Topic: `/object_detections`
- **Message Type**: `vision_msgs/Detection2DArray`
- **Description**: Publishes detected objects with bounding boxes
- **Message Format**:
  ```yaml
  header:
    stamp: {sec: 123456, nanosec: 789012345}
    frame_id: "camera_frame"
  detections:
    - results:
        - hypothesis: "ball"
          score: 0.95
      bbox:
        center:
          x: 320
          y: 240
          theta: 0.0
        size_x: 50
        size_y: 50
      id: "object_001"
  ```

#### ROS 2 Topic: `/camera/image_raw`
- **Message Type**: `sensor_msgs/Image`
- **Description**: Publishes raw RGB image data from camera
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

### 2.2 Visual Grounding API

**Purpose**: Interface for connecting language descriptions to visual entities

#### ROS 2 Service: `/visual_grounding/find_object`
- **Service Type**: Custom service (vla_msgs/srv/FindObject)
- **Description**: Find visual object based on language description
- **Request Format**:
  ```yaml
  description: "red ball"
  search_area: "current_view"  # or specific coordinates
  ```
- **Response Format**:
  ```yaml
  success: true
  object_found: true
  object_id: "obj_001"
  position:
    x: 1.5
    y: 2.3
    z: 0.0
  confidence: 0.95
  bounding_box:
    x: 100
    y: 150
    width: 50
    height: 50
  ```

## 3. Cognitive Planning APIs

### 3.1 Task Planning API

**Purpose**: Interface for generating action sequences from parsed commands

#### ROS 2 Service: `/task_planner/generate_plan`
- **Service Type**: Custom service (vla_msgs/srv/GeneratePlan)
- **Description**: Generate executable task plan from parsed command
- **Request Format**:
  ```yaml
  parsed_command:  # ParsedCommand message
    command_id: "cmd_001"
    command_type: "navigation"
    entities:
      - type: "object"
        value: "red ball"
        confidence: 0.95
        reference_id: "obj_001"
  current_robot_state:  # Robot state for context
    pose:
      position: {x: 0.0, y: 0.0, z: 0.0}
      orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}
  ```
- **Response Format**:
  ```yaml
  success: true
  plan:
    plan_id: "plan_001"
    start_time: 123456.789
    estimated_duration: 30.0
    confidence: 0.85
    tasks:
      - task_id: "task_001"
        action: "detect_object"
        parameters:
          object_type: "ball"
          color: "red"
        required_objects: []
        produced_objects: ["ball_red_001"]
        estimated_time: 5.0
        dependencies: []
  ```

#### ROS 2 Topic: `/execution_plan`
- **Message Type**: Custom message (vla_msgs/TaskPlan)
- **Description**: Publishes generated task plans for execution
- **Message Format**: See data-model.md for complete TaskPlan structure

### 3.2 Action Execution API

**Purpose**: Interface for executing planned actions on the robot

#### ROS 2 Action: `/action_executor/execute_task`
- **Action Type**: Custom action (vla_msgs/action/ExecuteTask)
- **Description**: Execute a specific task from the plan
- **Goal Format**:
  ```yaml
  task_id: "task_001"
  action: "approach_object"
  parameters:
    object_id: "ball_red_001"
  ```
- **Feedback Format**:
  ```yaml
  task_id: "task_001"
  status: "executing"
  progress: 0.6
  current_step: "navigating_to_object"
  ```
- **Result Format**:
  ```yaml
  success: true
  task_id: "task_001"
  completion_time: 8.5
  error_message: ""
  ```

## 4. VLA Integration APIs

### 4.1 Multi-Modal Fusion API

**Purpose**: Interface for combining voice, vision, and action data

#### ROS 2 Topic: `/fused_perception`
- **Message Type**: Custom message (vla_msgs/FusedPerception)
- **Description**: Publishes combined vision and language understanding
- **Message Format**:
  ```yaml
  timestamp: 123456.789
  visual_objects:
    - id: "obj_001"
      type: "ball"
      color: "red"
      position: {x: 1.5, y: 2.3, z: 0.0}
      confidence: 0.95
      bounding_box:
        x: 100
        y: 150
        width: 50
        height: 50
    - id: "obj_002"
      type: "table"
      color: "brown"
      position: {x: 3.0, y: 1.8, z: 0.0}
      confidence: 0.88
      bounding_box:
        x: 200
        y: 100
        width: 200
        height: 100
  language_entities:
    - id: "lang_001"
      type: "object"
      value: "red ball"
      confidence: 0.92
      reference: "obj_001"
    - id: "lang_002"
      type: "location"
      value: "table"
      confidence: 0.85
      reference: "obj_002"
  spatial_relationships:
    - from: "obj_001"
      to: "obj_002"
      relationship: "left_of"
      confidence: 0.78
  grounding_confidence: 0.85
  ```

### 4.2 VLA System State API

**Purpose**: Interface for monitoring and controlling the VLA system

#### ROS 2 Topic: `/vla_system_state`
- **Message Type**: Custom message (vla_msgs/VlaSystemState)
- **Description**: Publishes current state of the VLA system
- **Message Format**:
  ```yaml
  timestamp: 123456.789
  current_mode: "active"  # idle, active, paused, error
  current_task: "approach_object"
  task_progress: 0.6
  recognized_command: "cmd_001"
  active_objects: ["ball_red_001"]
  execution_plan: "plan_001"
  system_health: "ok"  # ok, warning, error
  voice_confidence: 0.92
  vision_confidence: 0.88
  planning_confidence: 0.85
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
    "title": "Voice-to-Action with OpenAI Whisper",
    "description": "Learn to recognize and parse natural language commands using Whisper",
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
    "module": "vla_integration",
    "current_section": "cognitive_planning",
    "completion_percentage": 60.0,
    "exercises_completed": 3,
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
        "answer": "VLA systems integrate vision, language, and action modalities..."
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
    "feedback": "Good understanding of VLA architecture concepts"
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
- **Voice Recognition**: < 2 seconds for processing
- **Command Parsing**: < 1 second for natural language understanding
- **Object Detection**: < 0.1 seconds for 15+ FPS operation
- **Task Planning**: < 5 seconds for complex command decomposition
- **Action Execution**: < 0.1 seconds for command acceptance

### 7.2 Data Throughput
- **Voice Data Rate**: 1 Hz for continuous command processing
- **Camera Data Rate**: 15-30 Hz for real-time vision
- **Plan Updates**: 1 Hz for dynamic replanning
- **State Updates**: 10 Hz for monitoring

### 7.3 Resource Usage
- **Memory**: < 4GB for VLA processing pipeline
- **GPU Memory**: < 2GB for AI model processing
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

### 8.3 System Security
- Prevent unauthorized access to robot control systems
- Implement sandboxing for student code execution
- Validate all input data to prevent system corruption

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