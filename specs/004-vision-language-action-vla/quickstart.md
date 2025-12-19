# Quickstart Guide: Module 4 - Vision-Language-Action (VLA)

## Prerequisites

1. **System Requirements**:
   - Ubuntu 22.04 (recommended) or equivalent Linux distribution
   - At least 8GB RAM (16GB recommended for AI processing)
   - 30GB free disk space
   - Python 3.8 or higher
   - GPU with CUDA support recommended for AI processing

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

3. **Install OpenAI Whisper**:
   ```bash
   pip install openai-whisper
   # Or for GPU acceleration:
   pip install torch torchvision torchaudio --index-url https://download.pytorch.org/whl/cu118
   pip install openai-whisper
   ```

4. **Install Computer Vision Libraries**:
   ```bash
   pip install opencv-python
   pip install ultralytics  # For YOLO object detection
   pip install transformers  # For language processing
   ```

## Setting Up Your VLA Workspace

1. **Create a ROS workspace for VLA**:
   ```bash
   mkdir -p ~/vla_ws/src
   cd ~/vla_ws
   colcon build
   source install/setup.bash
   ```

2. **Create a VLA package**:
   ```bash
   cd ~/vla_ws/src
   ros2 pkg create --build-type ament_python vla_examples
   ```

3. **Verify Installation**:
   ```bash
   # Check that Whisper is available
   python3 -c "import whisper; print('Whisper available')"

   # Check ROS 2 setup
   ros2 topic list
   ```

## Running Your First Voice-to-Action Example

1. **Create a basic voice processing node** (`~/vla_ws/src/vla_examples/vla_examples/voice_processor.py`):
   ```python
   import rclpy
   from rclpy.node import Node
   import whisper
   import speech_recognition as sr
   from std_msgs.msg import String
   from geometry_msgs.msg import Twist

   class VoiceProcessorNode(Node):
       def __init__(self):
           super().__init__('voice_processor')

           # Create publisher for voice commands
           self.voice_pub = self.create_publisher(String, 'voice_commands', 10)

           # Create publisher for robot actions
           self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

           # Initialize Whisper model
           self.model = whisper.load_model("base")

           # Initialize speech recognizer
           self.recognizer = sr.Recognizer()
           self.microphone = sr.Microphone()

           # Set up voice processing timer
           self.timer = self.create_timer(5.0, self.process_voice)

           self.get_logger().info('Voice processor node initialized')

       def process_voice(self):
           try:
               with self.microphone as source:
                   self.recognizer.adjust_for_ambient_noise(source)
                   audio = self.recognizer.listen(source, timeout=3.0)

                   # Use Whisper for speech-to-text
                   result = self.model.transcribe(audio.get_wav_data())
                   text = result['text']

                   # Publish the recognized text
                   msg = String()
                   msg.data = text
                   self.voice_pub.publish(msg)

                   self.get_logger().info(f'Recognized: {text}')

                   # Process the command and execute action
                   self.execute_command(text)

           except sr.WaitTimeoutError:
               self.get_logger().info('No speech detected')
           except Exception as e:
               self.get_logger().error(f'Error processing voice: {e}')

       def execute_command(self, command):
           # Simple command processing example
           cmd_vel = Twist()

           if 'forward' in command.lower():
               cmd_vel.linear.x = 0.5
               self.cmd_vel_pub.publish(cmd_vel)
               self.get_logger().info('Moving forward')
           elif 'backward' in command.lower():
               cmd_vel.linear.x = -0.5
               self.cmd_vel_pub.publish(cmd_vel)
               self.get_logger().info('Moving backward')
           elif 'stop' in command.lower():
               cmd_vel.linear.x = 0.0
               cmd_vel.angular.z = 0.0
               self.cmd_vel_pub.publish(cmd_vel)
               self.get_logger().info('Stopping')

   def main(args=None):
       rclpy.init(args=args)
       voice_processor = VoiceProcessorNode()

       try:
           rclpy.spin(voice_processor)
       except KeyboardInterrupt:
           pass
       finally:
           voice_processor.destroy_node()
           rclpy.shutdown()

   if __name__ == '__main__':
       main()
   ```

2. **Update setup.py** (`~/vla_ws/src/vla_examples/setup.py`):
   ```python
   from setuptools import find_packages, setup

   package_name = 'vla_examples'

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
       description='Vision-Language-Action examples for ROS 2',
       license='Apache-2.0',
       tests_require=['pytest'],
       entry_points={
           'console_scripts': [
               'voice_processor = vla_examples.voice_processor:main',
           ],
       },
   )
   ```

3. **Build and run the voice processor**:
   ```bash
   cd ~/vla_ws
   colcon build --packages-select vla_examples
   source install/setup.bash

   # Run the voice processor
   ros2 run vla_examples voice_processor
   ```

## Setting up Computer Vision Integration

1. **Create a basic object detection node** (`~/vla_ws/src/vla_examples/vla_examples/object_detector.py`):
   ```python
   import rclpy
   from rclpy.node import Node
   import cv2
   from ultralytics import YOLO
   from sensor_msgs.msg import Image
   from cv_bridge import CvBridge
   import numpy as np

   class ObjectDetectorNode(Node):
       def __init__(self):
           super().__init__('object_detector')
           self.bridge = CvBridge()

           # Load YOLO model
           self.model = YOLO('yolov8n.pt')  # You may need to download this first

           # Create subscription for camera input
           self.subscription = self.create_subscription(
               Image,
               '/camera/image_raw',
               self.image_callback,
               10
           )

           # Create publisher for detection results
           self.detection_pub = self.create_publisher(Image, '/detection_result', 10)

           self.get_logger().info('Object detector node initialized')

       def image_callback(self, msg):
           # Convert ROS Image message to OpenCV image
           cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

           # Run object detection
           results = self.model(cv_image)

           # Draw detection results on image
           annotated_frame = results[0].plot()

           # Convert back to ROS Image message
           result_msg = self.bridge.cv2_to_imgmsg(annotated_frame, encoding='bgr8')
           result_msg.header = msg.header

           # Publish detection results
           self.detection_pub.publish(result_msg)

           # Log detected objects
           names = results[0].names
           for box in results[0].boxes:
               cls = int(box.cls[0])
               conf = float(box.conf[0])
               self.get_logger().info(f'Detected {names[cls]} with confidence {conf:.2f}')

   def main(args=None):
       rclpy.init(args=args)
       object_detector = ObjectDetectorNode()

       try:
           rclpy.spin(object_detector)
       except KeyboardInterrupt:
           pass
       finally:
           object_detector.destroy_node()
           rclpy.shutdown()

   if __name__ == '__main__':
       main()
   ```

## Cognitive Planning Example

1. **Create a basic command parser** (`~/vla_ws/src/vla_examples/vla_examples/command_parser.py`):
   ```python
   import rclpy
   from rclpy.node import Node
   from std_msgs.msg import String
   from geometry_msgs.msg import Twist
   from vla_examples.object_detector import ObjectDetectorNode  # Import for object detection

   class CommandParserNode(Node):
       def __init__(self):
           super().__init__('command_parser')

           # Create subscription for voice commands
           self.command_sub = self.create_subscription(
               String,
               'voice_commands',
               self.command_callback,
               10
           )

           # Create publisher for robot actions
           self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

           # Simple command history
           self.command_history = []

           self.get_logger().info('Command parser node initialized')

       def command_callback(self, msg):
           command = msg.data.lower()
           self.get_logger().info(f'Received command: {command}')

           # Add to command history
           self.command_history.append(command)

           # Parse and execute command
           self.parse_and_execute(command)

       def parse_and_execute(self, command):
           cmd_vel = Twist()

           # Simple command parsing
           if 'move forward' in command or 'go forward' in command:
               cmd_vel.linear.x = 0.5
               self.cmd_vel_pub.publish(cmd_vel)
               self.get_logger().info('Executing: Move forward')
           elif 'move backward' in command or 'go backward' in command:
               cmd_vel.linear.x = -0.5
               self.cmd_vel_pub.publish(cmd_vel)
               self.get_logger().info('Executing: Move backward')
           elif 'turn left' in command:
               cmd_vel.angular.z = 0.5
               self.cmd_vel_pub.publish(cmd_vel)
               self.get_logger().info('Executing: Turn left')
           elif 'turn right' in command:
               cmd_vel.angular.z = -0.5
               self.cmd_vel_pub.publish(cmd_vel)
               self.get_logger().info('Executing: Turn right')
           elif 'stop' in command:
               cmd_vel.linear.x = 0.0
               cmd_vel.angular.z = 0.0
               self.cmd_vel_pub.publish(cmd_vel)
               self.get_logger().info('Executing: Stop')
           else:
               self.get_logger().info(f'Unknown command: {command}')

   def main(args=None):
       rclpy.init(args=args)
       command_parser = CommandParserNode()

       try:
           rclpy.spin(command_parser)
       except KeyboardInterrupt:
           pass
       finally:
           command_parser.destroy_node()
           rclpy.shutdown()

   if __name__ == '__main__':
       main()
   ```

## Troubleshooting

1. **If Whisper doesn't work**:
   ```bash
   # Check if Whisper is properly installed
   python3 -c "import whisper; print(whisper.__version__)"
   ```

2. **If ROS commands are not found**:
   ```bash
   source /opt/ros/humble/setup.bash
   source ~/vla_ws/install/setup.bash
   ```

3. **If audio recognition fails**:
   ```bash
   # Check available audio devices
   arecord -l
   ```

4. **If computer vision node fails**:
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
4. Integrate voice, vision, and action components for the capstone project
5. Deploy a working simulation or edge-kit implementation for the final demo