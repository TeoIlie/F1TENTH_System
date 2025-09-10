#!/bin/bash
# Run this script using "/opt/f1tenth/battery_monitor.sh" (no "sudo")
# Source ROS2 and workspace
source /opt/ros/humble/setup.bash
echo "Sourced ROS2 Humble"

if [ -f /home/jetson/f1tenth_ws/install/setup.bash ]; then
  source /home/jetson/f1tenth_ws/install/setup.bash
  echo "Sourced ROS2 workspace from f1tenth_ws"
fi

# Set ROS_DOMAIN_ID
export ROS_DOMAIN_ID=42
echo "Setting custom ROS_DOMAIN_ID=42 to prevent shared network interference"

# TEST ros2 command by uncommenting below
# ros2 topic echo /sensors/core --field state.voltage_input --once

VOLTAGE_THRESHOLD=9.5
TOPIC="/sensors/core"
PASSWORD="Jetson123"

echo "Starting voltage monitor (threshold: ${VOLTAGE_THRESHOLD}V)"
echo "Monitoring topic: ${TOPIC}"
echo "Press Ctrl+C to stop"

# Monitor the ROS2 topic and process each voltage reading
ros2 topic echo "$TOPIC" --field state.voltage_input | while read -r voltage; do

    # TEST shutdown works by uncommenting the hardcoded low voltage velow
    # voltage="9.0"

    # Skip empty lines and non-numeric values
    if [[ "$voltage" =~ ^[0-9]+\.?[0-9]*$ ]]; then
        echo "Current voltage: ${voltage}V"
        
        # Compare voltage with threshold using bc for floating point comparison
        if (( $(echo "$voltage < $VOLTAGE_THRESHOLD" | bc -l) )); then
            echo "WARNING: Voltage ${voltage}V is below threshold ${VOLTAGE_THRESHOLD}V"
            echo "Initiating clean shutdown in 5 seconds..."
            sleep 5
            echo "Shutting down system..."
            echo "$PASSWORD" | sudo -S shutdown -h now
            break
        fi
    fi
done
