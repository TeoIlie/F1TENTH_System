# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

This is the F1TENTH autonomous racing car driver stack for ROS2 Humble. The system provides low-level drivers and teleoperation for F1TENTH race cars, including VESC motor control, LiDAR integration, and joystick teleoperation.

**Important:** This is a personal fork of the main F1TENTH system with modifications for a specific car setup.

## Architecture

The system consists of 11 ROS2 packages organized into functional groups:

### Core Packages
- **f1tenth_stack**: Main bringup package with launch files and configuration
- **vesc/**: VESC motor controller drivers (4 packages: vesc, vesc_driver, vesc_ackermann, vesc_msgs)
- **ackermann_mux**: Message multiplexer for autonomous/manual control switching
- **teleop_tools/**: Teleoperation packages (joy_teleop, key_teleop, mouse_teleop, teleop_tools_msgs, teleop_tools)

### Key System Components
1. **Control Flow**: `/drive` (autonomous) or joystick → ackermann_mux → vesc_ackermann → vesc_driver → motor
2. **Sensor Data**: LiDAR → `/scan`, VESC → `/odom` + `/sensors/imu/raw` + `/sensors/core`
3. **Safety**: Deadman switches on joystick (LB for teleop, RB for navigation)

## Essential Commands

### System Bringup
```bash
# Full system with LiDAR
ros2 launch f1tenth_stack bringup_launch.py

# System without LiDAR  
ros2 launch f1tenth_stack no_lidar_bringup_launch.py

# System with SICK LiDAR
ros2 launch f1tenth_stack sick_bringup_launch.py
```

### Submodules Management
```bash
# Initial clone with submodules
git submodule update --init --recursive --remote

# Update submodules to latest
git submodule update --recursive --remote
```

### Testing
Use standard ROS2 testing commands:
```bash
colcon test
colcon test-result --verbose
```

Individual packages support pytest, flake8, and copyright tests.

## Key Configuration Files

All configuration is in `f1tenth_stack/config/`:
- `vesc.yaml`: Motor controller parameters and calibration
- `sensors.yaml`: LiDAR and sensor configuration  
- `joy_teleop.yaml`: Joystick button/axis mappings
- `mux.yaml`: Control multiplexer settings

## Important Topics

### Input Topics (Subscribed)
- `/drive`: AckermannDriveStamped - autonomous navigation commands

### Output Topics (Published)  
- `/scan`: LaserScan - LiDAR data
- `/odom`: Odometry - wheel odometry from VESC
- `/sensors/imu/raw`: Imu - IMU data from VESC
- `/sensors/core`: VescStateStamped - VESC telemetry

## LiDAR Setup Notes

### SICK TiM-5xx LiDAR
- Requires `ros-humble-sick-scan-xd` package
- Use the modified launch file: `f1tenth_stack/launch/sick_tim_5xx.launch` 
- This version sets correct frame_id ("laser") and tf_base_frame_id ("base_link") for SLAM compatibility
- Set LiDAR IP address in launch file before use

### Finding SICK LiDAR IP
```bash
# Scan network for LiDAR
nmap -sn [network_range]
# or use SICK SOPAS ET on Windows
```

## Development Notes

- **Build System**: Uses colcon (ROS2 standard)
- **Python Packages**: Use setuptools with entry_points for ROS2 nodes
- **Launch Files**: Python launch files (.py) are preferred over XML
- **Frame Convention**: base_link → laser, odom → base_link transforms
- **Safety Critical**: Always test joystick deadman switches before autonomous operation