# Data Model: ROS 2 Robotic Nervous System

## Core Entities

### ROS 2 Node
- **Description**: A process that performs computation and communicates with other nodes
- **Key attributes**:
  - Node name (string)
  - Node namespace (string, optional)
  - Published topics (list of topic names)
  - Subscribed topics (list of topic names)
  - Services provided (list of service names)
  - Services called (list of service names)
  - Actions provided (list of action names)
  - Actions called (list of action names)
- **Relationships**: Communicates via topics, services, and actions

### Topic
- **Description**: A named bus over which nodes exchange messages in a publish/subscribe pattern
- **Key attributes**:
  - Topic name (string)
  - Message type (string, e.g., std_msgs/String)
  - Publishers (list of nodes)
  - Subscribers (list of nodes)
  - QoS settings (Quality of Service configuration)
- **Relationships**: Connected to publisher and subscriber nodes

### Service
- **Description**: A synchronous request/response communication pattern between nodes
- **Key attributes**:
  - Service name (string)
  - Service type (string, e.g., std_srvs/SetBool)
  - Service server (node providing the service)
  - Service clients (list of nodes calling the service)
- **Relationships**: Connected to service server and client nodes

### Action
- **Description**: An asynchronous goal-based communication pattern for long-running tasks
- **Key attributes**:
  - Action name (string)
  - Action type (string, e.g., example_interfaces/Fibonacci)
  - Action server (node providing the action)
  - Action clients (list of nodes calling the action)
- **Relationships**: Connected to action server and client nodes

### URDF Robot Model
- **Description**: An XML description of a robot's physical structure
- **Key attributes**:
  - Robot name (string)
  - Links (list of link entities)
  - Joints (list of joint entities)
  - Materials (list of material definitions)
  - Transmissions (list of transmission definitions)
- **Relationships**: Contains multiple links and joints that form the robot structure

### Link
- **Description**: A rigid body element of a robot
- **Key attributes**:
  - Link name (string)
  - Visual properties (geometry, material, origin)
  - Collision properties (geometry, origin)
  - Inertial properties (mass, inertia matrix)
- **Relationships**: Connected to other links via joints

### Joint
- **Description**: A connection between two links that allows relative motion
- **Key attributes**:
  - Joint name (string)
  - Joint type (string: revolute, continuous, prismatic, fixed, etc.)
  - Parent link (string)
  - Child link (string)
  - Joint limits (position, velocity, effort for revolute joints)
  - Origin transform (position and orientation)
- **Relationships**: Connects parent and child links

### Launch File
- **Description**: A configuration file that defines how to start multiple nodes with specific parameters
- **Key attributes**:
  - Launch file name (string)
  - Node descriptions (list of node configurations)
  - Parameters (key-value pairs)
  - Namespaces (string)
  - Remappings (topic/service remappings)
- **Relationships**: References nodes to be launched

### Parameter
- **Description**: Configuration values that can be set for nodes at runtime
- **Key attributes**:
  - Parameter name (string)
  - Parameter value (any type: string, integer, float, boolean, etc.)
  - Parameter type (string: string, int, double, bool, etc.)
  - Default value (any type)
- **Relationships**: Associated with specific nodes or globally

## State Transitions

### Node Lifecycle
1. Unconfigured → Inactive: After node creation
2. Inactive → Active: After activation
3. Active → Inactive: After deactivation
4. Inactive → Unconfigured: After cleanup
5. Any state → Finalized: Before destruction

### Action Client States
1. IDLE → SEND_GOAL: When sending a goal
2. SEND_GOAL → PENDING: While waiting for server acknowledgment
3. PENDING → ACTIVE: When goal is being processed
4. ACTIVE → FEEDBACK: While processing with feedback
5. FEEDBACK → SUCCEEDED/ABORTED/CANCELED: Goal completion states

## Validation Rules

1. **Node Naming**: Node names must be unique within a namespace
2. **Topic Connection**: Publishers and subscribers must use compatible message types
3. **Service Matching**: Service clients must use the same service type as the server
4. **Action Matching**: Action clients must use the same action type as the server
5. **URDF Validity**: All joint parent and child links must exist in the robot model
6. **Joint Limits**: Joint positions must be within defined limits
7. **Parameter Types**: Parameters must match the expected type for the node
8. **Namespace Scoping**: Namespaced elements must follow proper naming conventions

## Relationships Overview

- **Nodes** communicate through **Topics**, **Services**, and **Actions**
- **URDF Robot Model** contains **Links** and **Joints** that define the robot's structure
- **Launch Files** configure **Nodes** and their **Parameters**
- **Parameters** provide runtime configuration for **Nodes**
- **Joints** connect **Links** to form the robot structure