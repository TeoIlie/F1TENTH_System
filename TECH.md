# F1TENTH System Technical Architecture

## Overview

The F1TENTH system is a ROS2 Humble-based autonomous racing car driver stack that provides comprehensive low-level control, sensor integration, and teleoperation capabilities for F1TENTH race cars. This system bridges high-level autonomous navigation commands with hardware-level motor control and sensor data acquisition.

## System Architecture

### Control Flow Pipeline

The system implements a hierarchical control architecture:

```
Autonomous Commands (/drive) OR Joystick Input
                    ↓
              ackermann_mux (safety multiplexer)
                    ↓
           ackermann_to_vesc_node (command conversion)
                    ↓
              vesc_driver_node (hardware interface)
                    ↓
               Motor Hardware
```

### Sensor Data Pipeline

Sensor data flows through dedicated pathways:

```
LiDAR Hardware → urg_node → /scan (LaserScan)
VESC Hardware → vesc_driver_node → /sensors/core (VescStateStamped)
                                 → /sensors/imu/raw (Imu)
               → vesc_to_odom_node → /odom (Odometry)
```

## Core Node Architecture

### Input Processing Nodes

- **joy_node**: Captures raw joystick input from hardware device
- **joy_teleop_node**: Processes joystick commands with configurable button/axis mappings and deadman switch logic (LB=teleoperation, RB=navigation mode)

### Command Processing Nodes

- **ackermann_mux**: Safety-critical multiplexer that arbitrates between autonomous navigation commands (`/drive`) and manual teleoperation input. Implements deadman switch safety mechanisms.
- **ackermann_to_vesc_node**: Converts high-level Ackermann drive commands (steering angle + speed) to low-level VESC motor commands using configurable gain and offset parameters.

### Hardware Interface Nodes

- **vesc_driver_node**: Direct interface to VESC motor controller hardware. Handles duty cycle, current, brake, speed, and servo position commands while publishing comprehensive telemetry data.

### Sensor Nodes

- **urg_node**: Hokuyo URG series LiDAR driver that publishes LaserScan messages on `/scan` topic
- **vesc_to_odom_node**: Computes wheel odometry from VESC encoder data, publishing Odometry messages and optionally broadcasting TF transforms

### Transform Management

- **static_transform_publisher**: Publishes static transform from `base_link` to `laser` frame (translation: x=0.27m, z=0.11m) for sensor fusion and SLAM compatibility

## Critical Topics

### Input Topics
- `/drive`: AckermannDriveStamped messages for autonomous navigation commands

### Output Topics
- `/scan`: LaserScan messages from LiDAR sensor
- `/odom`: Odometry messages from wheel encoders
- `/sensors/imu/raw`: Raw IMU data from VESC
- `/sensors/core`: Comprehensive VESC telemetry including motor state, battery voltage, and fault conditions

## Safety Systems

### Deadman Switch Implementation
- **LB Button**: Enables teleoperation mode - must be held for manual control
- **RB Button**: Enables navigation mode - must be held for autonomous operation
- **Release Behavior**: Immediate motor stop when deadman switches are released

### Control Arbitration
The `ackermann_mux` node provides safety-critical arbitration between control sources with configurable priority levels and timeout mechanisms.

## Configuration Management

System configuration is centralized in `f1tenth_stack/config/`:
- `vesc.yaml`: Motor controller parameters, calibration constants, and safety limits
- `sensors.yaml`: LiDAR configuration and sensor parameters
- `joy_teleop.yaml`: Joystick button/axis mappings and deadman switch configuration
- `mux.yaml`: Control multiplexer priority and timeout settings

## Launch Configurations

The system supports multiple deployment configurations:
- **Standard Bringup**: Full system with Hokuyo URG LiDAR
- **No LiDAR**: Motor control and teleoperation without laser sensor
- **SICK LiDAR**: Alternative configuration for SICK TiM-5xx series sensors

Each configuration maintains identical node architecture while adapting sensor-specific parameters and launch sequences.