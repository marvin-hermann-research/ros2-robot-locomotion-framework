# AETHER I - Bipedal Robot Locomotion Framework: A Modular Behaviour Tree Architecture for ROS2

## Table of Contents

- [Project Description](#project-description)
- [Motivation](#motivation)
- [Project Context](#project-context)
- [Architecture Overview](#architecture-overview)
  - [behaviour_tree/](#behaviour_tree)
  - [ros_nodes/](#ros_nodes)
- [System Launch and Initialisation](#system-launch-and-initialisation)
- [Data Flow and Component Interaction](#data-flow-and-component-interaction)
- [Motion Pattern Management](#motion-pattern-management)
  - [Structure](#structure)
- [Structured Scientific Logging](#structured-scientific-logging)
  - [Core Design](#logging-core-design)
  - [Usage Example](#usage-example)
- [Use Cases](#use-cases)
  - [Idle](#idle)
  - [Walk Forward](#walk-forward)
  - [Use Case Visualisation](#use-case-visualisation)
- [Installation & Usage](#installation--usage)
  - [Setup](#setup)
    - [1. System requirements](#1-system-requirements)
    - [2. Clone repository](#2-clone-repository)
    - [3. Install Python dependencies](#3-install-python-dependencies)
    - [Build the workspace](#build-the-workspace)
    - [Run the application](#run-the-application)
- [Extensibility](#extensibility)
- [Future Work](#future-work)
- [License](#license)
- [Author](#author)

## Project Description

As a part of the [AETHER Framework Series](https://github.com/marvin-hermann-research/aether-framework-overview.git), this repository presents a **research-oriented framework** for robotic locomotion in ROS2.  
It integrates **Behaviour Trees (BTs)** with a **distributed sensor–actuator architecture**, enabling structured decision-making and extensible, modular control of humanoid robots.

## Motivation

ROS2 has become the standard middleware for robotics and embodied AI research, offering robust communication between sensors, actuators, and higher-level logic.
Behaviour Trees enhance this by introducing **structured, hierarchical decision-making**, offering greater flexibility and transparency than finite-state machines, particularly in locomotion tasks.

This framework has been developed as a **research foundation to explore modular robotic architectures** in simulation.  
It emphasizes **scientific conventions** such as reproducibility, extensibility, and clarity, while maintaining compatibility with real hardware through a **plug-and-play design**.  
As such, it serves both as a reproducible baseline for scientific experimentation and as a portfolio contribution towards ongoing research in **embodied AI and behaviour-based control**.

## Project Context

The framework implements a modular and extensible architecture for robotic locomotion in ROS2.
The design follows current research conventions and emphasizes scalability, making it suitable as a foundation for future experimental projects.

**Scope**  
- Provides a realistic locomotion framework without relying on physical sensors or actuators.
- Uses simulated ROS2 nodes for sensing and actuation, while maintaining a plug-and-play design that supports future hardware integration.  

**References**
Development was guided by the official documentation of:

- [ROS2 Documentation](https://docs.ros.org/)
- [py_trees](https://py-trees.readthedocs.io/)
- [py_trees_ros](https://py-trees-ros-tutorials.readthedocs.io/)
- [python-json-logger](https://pypi.org/project/python-json-logger/)
  
Additional resources are available in the [ROS2 Masterpost](https://github.com/marvin-hermann-research/masterposts/blob/main/ros2_engineering_notes.md), providing detailed insights into ROS2 concepts and architectural decisions.

## Architecture Overview

```bash
bipedal_robot_pkg/
├── setup.py
├── package.xml
├── requirements.txt
├── __init__.py
├── behaviour_tree/
│   ├── patterns/
│   │   ├── all_patterns.yaml
│   │   ├── idle_pattern.yaml
│   │   └── walk_forward_pattern.yaml
│   ├── nodes/
│   │   ├── tree_factory.py
│   │   ├── actions/
│   │   │   ├── idle_behaviour.py
│   │   │   └── walk_forward_behaviour.py
│   │   └── conditions/
│   │       ├── can_walk.py
│   │       └── must_walk.py
│   └── launch/
│       └── bipedal_robot_application.py
├── ros_nodes/
│   ├── actuators/
│   │   ├── left_leg_node.py
│   │   └── right_leg_node.py
│   ├── controller/
│   │   └── movement_controller_node.py
│   ├── evaluators/
│   │   ├── can_walk_evaluator.py
│   │   └── must_walk_evaluator.py
│   ├── logger/
│   │   └── logging_factory.py
│   ├── sensors/
│   │   ├── battery_monitor_node.py
│   │   ├── imu_sensor_node.py
│   │   └── laser_sensor_node.py
│   └── action_publishers/
│       ├── idle_publisher.py
│       └── walk_forward_publisher.py
```

The project consists of two primary modules beyond the standard ROS2 components (`setup.py`, `package.xml`, `requirements.txt`):

### behaviour_tree/

Implements the behaviour tree components, stores locomotion patterns, and handles robot initialisation. Contains:

- **patterns/**: YAML files defining locomotion sequences.
- **nodes/**: The `TreeFactory` constructs BTs and includes action and condition nodes.
- **launch/**: Launch file initialising the behaviour tree and all ROS2 nodes.

### ros_nodes/

Implements ROS2 components, manages structured scientific logging, and bridges to the behaviour tree. Contains:

- **actuators/**: Nodes for robot actuators (e.g., legs), receiving instructions from the controller.
- **controller/**: `MovementControllerNode` executes locomotion patterns based on BT actions.
- **evaluators/**: Subscriber nodes that evaluate sensor data and update Blackboard keys.
- **logger/**: Provides `LoggingFactory` for structured JSON logging of experiments.
- **sensors/**: Sensor nodes publishing simulated data to ROS2 topics.
- **action_publishers/**: Bridge BT actions to ROS2 topics, used by action nodes to command the controller.

### Architecture Diagrams

<p align="center">
  <img src="docs/images/package_diagram.svg" width="800"/>
</p>

*Figure 1: Package Diagram.*

<p align="center">
  <img src="docs/images/rosgraph.svg" width="800"/>
</p>

*Figure 2: ROS2 nodes and topics during runtime.*

## System Launch and Initialisation

The application launches via a ROS 2 entrypoint (`ros2 run`), calling the **BipedalRobotApplication** constructor. The initialisation sequence is as follows:

1. **ROS 2 Runtime Initialisation**  
   - The `rclpy` runtime is started and the main application node (`bipedal_robot_application`) is created.  

2. **Node Instantiation**  
  -  Instantiate and register all subsystem nodes:
    - **Action Publishers**: `IdlePublisher`, `WalkForwardPublisher`
	- **Actuators**: `LeftLegNode`, `RightLegNode`
	- **Controller**: `MovementControllerNode`
	- **Evaluators**: `CanWalkEvaluator`, `MustWalkEvaluator`
	- **Sensors**: `BatteryMonitorNode`, `ImuSensorNode`, `LaserSensorNode`

3. **Behaviour Tree Construction**  
   - A factory pattern (`TreeFactory`) builds the locomotion behaviour tree.  
   - Each **BT Condition Node** is linked to exactly one **Evaluator Node**,  
     allowing it to use the ROS 2 logging framework.  
   - Each **BT Action Node** is linked to exactly one **Action Publisher Node**,  
     enabling it to signal locomotion commands through ROS 2 topics.  

   - The locomotion tree follows a **priority-based structure**:  

     **Root Node (Selector: "Locomotion Root")**  
     ├── **Sequence: "Walk Forward Sequence"**  
     │   ├── Condition: `Can Walk?`  
     │   ├── Condition: `Must Walk?`  
     │   └── Action: `Walk Forward`  
     └── **Action: Idle**  

   - The tree is implemented in `TreeFactory` and integrated with ROS 2 via `py_trees_ros.BehaviourTree`.  

<p align="center">
  <img src="docs/images/tree_visualisation.gif" width="800"/>
</p>

*Figure 3: Behaviour Tree visualisation.*


4. **Executor Setup**  
   - All nodes, including the behaviour tree, are registered under a **multi-threaded executor**.  
   - This ensures parallel handling of sensor input, condition evaluation, decision logic, and actuation.  

5. **Run Phase**  
   - Periodic BT ticking (default 100 ms) while the executor handles callbacks.

6. **Shutdown**  
   - On interrupt or termination, all nodes are gracefully destroyed and the ROS 2 runtime is properly shut down to ensure reproducibility and resource safety.  

### Project Startup

<p align="center">
   <img src="docs/images/project_startup.gif" width="600"/>
   <img src="docs/images/startup_intialisation_output.png" width="600"/>
</p>

*Figure 4: a) Project startup sequence; b) Initialisation sequence.*

## Data Flow and Component Interaction

### ROS2, Behaviour Tree Communication Pipeline

<p align="center">
   <img src="docs/images/tree_ros_pipeline.svg" width="600"/>
</p>

*Figure 5: Communication pipeline architecture.*

The system follows a modular **ROS 2 ↔ Behaviour Tree integration** pattern.  
Each component fulfills a distinct role in sensing, decision-making, and actuation.  

1. **Sensor Nodes**  
   - Publish raw data (e.g., IMU, battery status, laser range) on dedicated ROS 2 topics.  

<p align="center">
   <img src="docs/images/laser_topic_stream.gif" width="600"/>
</p>

*Figure 6: Laser sensor topic stream.*

2. **Evaluators (ROS 2 Subscriber Nodes)**  
   - Subscribe to one or multiple sensor topics.  
   - Internally process the data according to predefined logic.  
   - Each evaluator is uniquely bound to a **single Blackboard key** (writer role).  
   - Depending on the evaluation, they set the Blackboard entry to *True/False* (or other structured values).  

3. **Behaviour Tree Condition Nodes**  
   - Each condition node is linked to a Blackboard key.  
   - On each tick, the condition checks the Blackboard entry.  
   - Returns **SUCCESS** or **FAILURE**, thereby influencing the execution path of the BT.  

4. **Behaviour Tree Action Nodes**  
   - When activated, an action node uses its associated **Action Publisher Node**.  
   - The action node calls the publisher’s `.publish()` method, emitting the required control message on a topic.  

5. **Action Publishers → Controller**  
   - Action Publishers publish locomotion commands on dedicated topics.  
   - The **Movement Controller Node** subscribes to these topics.  
   - On receiving a command, the controller loads the corresponding **locomotion pattern** (from YAML files).  
   - It then distributes the motion primitives to the actuator nodes.  

6. **Actuator Nodes**  
   - Represent robot limbs (e.g., left/right legs).  
   - Execute the received motion commands to produce locomotion.  

### Active Nodes and Topics

<p align="center">
  <img src="docs/images/node_list_command.png" width="400"/>
  <img src="docs/images/topic_list_command.png" width="400"/>
</p>

*Figure 7: a) Node list; b) Topic listing during runtime.*

## Motion Pattern Management

Locomotion sequences are abstracted into **YAML-based motion patterns**.  
This design separates high-level behaviours from low-level actuation details, allowing researchers to extend or modify robot gaits without touching controller code.

### Structure
1. **Pattern Index (`all_patterns.yaml`)**  
   - Maps symbolic names (e.g., `idle`, `walk_forward`) to file paths of specific motion patterns.
   - Provides a single point of reference for available gaits.

2. **Pattern Files (`*_pattern.yaml`)**  
   - Contain time-stamped joint configurations.  
   - Each entry specifies hip and knee angles for both legs relative to upright stance.  
   - Designed for continuous looping.

<p align="center">
  <img src="docs/images/pattern_example.png" width="600"/>
</p>

*Figure 8: Pattern example. Showcase of walk cycle step.*

3. **Runtime Loading**  
   - The `MovementControllerNode` loads the requested pattern via the index.  
   - Pattern steps are executed in real time by comparing elapsed time against step timestamps.  
   - Completed patterns are reset, enabling repetition or seamless switching.

## Structured Scientific Logging

The framework provides a **dedicated logging subsystem**, ensuring that every relevant event in the robot’s lifecycle can be captured, stored, and later analyzed in a reproducible manner.

### Logging Core Design
- **Logging Factory**: Each ROS 2 node can instantiate a `LoggingFactory`, which wraps around Python’s `logging` and `python-json-logger`.  
- **JSON Schema**: All logs follow a fixed structure:  

  ```json
  {
    "timestamp": "...",
    "node": "...",
    "level": "...",
    "event": "...",
    "data": {...}
  }
  ```

- **Storage**: Logs are written to rotating JSON files (default: `robot_log.json`), preventing uncontrolled growth while preserving history.
- **Decoupled Operation**: Logging is orthogonal to decision-making, failures in logging do not affect control flow.

### Usage Example

```python
self._json_logger.log("INFO", "Pattern Loaded", {"pattern": pattern_name})
self._json_logger.log("ERROR", "Pattern Load Failed", {"pattern": pattern_name, "error": str(e)})
```

<p align="center">
  <img src="docs/images/json_log.png" width="800"/>
</p>

*Figure 9: Log example.*

## Use Cases

### Idle

- **Trigger**: Executed when either BT condition node `CanWalk` or `MustWalk` return `FAILURE`.
- **Flow**:
  1. BT action node `IdleBehaviour` publishes an `idle_status` topic.
  2. `MovementControllerNode` receives this message.
  3. The controller loads the `idle` pattern from YAML and streams actuator commands.
- **Result**: The robot enters a subtle, stationary stance, simulating realistic idle posture.

### Walk Forward

- **Trigger**: Executed when both BT condition nodes `CanWalk` and `MustWalk` return `SUCCESS`.
- **Special Note**: `CanWalk` depends on sensor logic; `MustWalk` is externally triggerable via CLI *(Command Line Interface)* input, simulating an explicit researcher command.
- **Flow**:
  1. BT action node `WalkForwardBehaviour` publishes a `walk_forward` topic.
  2. `MovementControllerNode` subscribes and reacts by loading the `walk_forward` YAML pattern.
  3. The controller streams the walking motion sequence step-by-step to actuators.
- **Result**: The robot performs a cyclic forward walking gait.

### Use Case Visualisation

<p align="center">
  <img src="docs/images/use_case.gif" width="800"/>
</p>

*Figure 10: Use case visualisation. Command line interface induced switch of action state.*


## Installation & Usage

**Requirements**: ROS2 Humble, Python ≥ 3.10, dependencies from `requirements.txt`.

### Setup

#### 1. System requirements

This project is built for **ROS 2 Humble (Ubuntu 22.04)**.  
Make sure ROS 2 Humble is installed following the [official installation guide](https://docs.ros.org/en/humble/Installation.html).  
Also install the colcon build tools:

```bash
sudo apt install python3-colcon-common-extensions
```

#### 2. Clone repository

```bash
git clone https://github.com/marvin-hermann-research/ros2-architecture-lab.git
cd bipedal_robot_pkg
```
#### 3. Install Python dependencies

```bash
pip install -r requirements.txt
```

#### Build the workspace

```bash
colcon build
source install/setup.bash
```

#### Run the application

```bash
ros2 run bipedal_robot_pkg robot_application
```
  
## Extensibility

The framework is designed for modular expansion:
- Integration of additional sensors or actuators.
- New locomotion patterns (AI-generated).
- Alternative or more complex BT structures (e.g. task switching).
- Extension of the logging system (e.g. Grafana dashboards, external monitoring).
- Planned future work: **hardware integration** via a dedicated companion [repository].

## Future Work

A dedicated hardware project building upon this framework is currently planned.
The goal is to control a simple, stationary robot arm using simulated neuromorphic sensors, implemented on conventional hardware (e.g., ESP32) but emulating event-driven behaviour.

This work serves as a bridge towards the next iterations in the [AETHER Series](https://github.com/marvin-hermann-research/aether-framework-overview.git), validating the software stack on physical hardware and preparing for more complex, multi-agent and AI-driven control systems in future AETHER versions.

## License

This project is licensed under the MIT License. See the [LICENSE](LICENSE) file for details.

## Author

Marvin Hermann (@marvin-hermann-research)
