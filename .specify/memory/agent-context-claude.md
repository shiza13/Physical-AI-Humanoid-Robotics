# Agent Context: Vision-Language-Action (VLA) Module

## Overview
This file contains context information for Claude Code agents working on the Vision-Language-Action (VLA) module. It includes key technical details, architecture patterns, and implementation guidelines specific to the VLA system.

## VLA System Components

### 1. Voice Processing Module
- **Technology**: OpenAI Whisper for speech-to-text conversion
- **ROS 2 Interface**: `/voice_commands` topic (std_msgs/String)
- **Performance Target**: <2 seconds processing time, 85%+ accuracy
- **Key Functions**: Audio input, speech recognition, command parsing

### 2. Vision System
- **Technology**: YOLOv5/Opencv for object detection
- **ROS 2 Interface**: `/object_detections` topic (vision_msgs/Detection2DArray)
- **Performance Target**: 15+ FPS, 80%+ accuracy
- **Key Functions**: Object detection, visual grounding, spatial reasoning

### 3. Cognitive Planner
- **Function**: Translate natural language commands into action sequences
- **Performance Target**: <5 seconds for complex command decomposition, 90%+ accuracy
- **Key Functions**: Task decomposition, action sequencing, plan generation

### 4. Action Executor
- **Technology**: ROS 2 action clients for robot control
- **Key Functions**: Execute ROS 2 actions on robotic platforms

## ROS 2 Communication Patterns

### Topics
- `/voice_commands` - Voice commands from speech recognition
- `/object_detections` - Object detection results
- `/parsed_commands` - Structured command representations
- `/execution_plan` - Generated task plans
- `/vla_system_state` - System state monitoring
- `/fused_perception` - Combined vision and language understanding

### Services
- `/voice_recognition/process_audio` - Process audio input
- `/command_parser/parse_command` - Parse natural language commands
- `/visual_grounding/find_object` - Find visual objects by description

### Actions
- `/action_executor/execute_task` - Execute specific tasks from plans

## Data Structures

### Voice Processing Data
- **Speech Recognition Result**: transcription, confidence, duration, timestamp
- **Voice Command**: text command string with metadata

### Vision Data
- **Object Detection**: bounding box, confidence scores, object IDs
- **Image Data**: sensor_msgs/Image format

### Planning Data
- **Parsed Command**: intent, action, target objects, spatial relationships
- **Task Plan**: sequence of executable tasks with dependencies

## Performance Requirements
- Voice recognition: <2 seconds
- Task planning: <5 seconds
- Computer vision: 15+ FPS
- End-to-end response: <3 seconds
- System memory: <4GB
- CPU utilization: <75% on 8-core system

## Educational Components
- Learning modules following 5 user stories
- Hands-on exercises after each section
- Validation checkpoints for student progress
- Troubleshooting guides for common issues

## Implementation Guidelines
- Use Python 3.8+ for ROS 2 Humble compatibility
- Implement beginner-friendly error handling
- Include comprehensive logging and debugging tools
- Follow ROS 2 best practices for node design
- Ensure simulation-first approach with optional real-robot deployment

## Dependencies
- ROS 2 Humble Hawksbill
- OpenAI Whisper
- YOLOv5/Ultralytics
- OpenCV
- Ubuntu 22.04 LTS
- NVIDIA GPU with 4GB+ VRAM (recommended)

## Architecture Patterns
- Modular design with separate components for voice, vision, and planning
- Publisher-subscriber pattern for real-time data flow
- Service-based communication for request-response interactions
- Action-based communication for long-running tasks
- Multi-modal fusion for combining voice, vision, and action data