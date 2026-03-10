#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from vesc_msgs.msg import VescStateStamped
import os
import signal


class VescBatteryMonitor(Node):
    """
    Monitor VESC battery voltage and shut down system when voltage drops below threshold.
    Monitors /sensors/core topic and reads state.voltage_input field.
    """

    def __init__(self):
        super().__init__("vesc_battery_monitor")

        # Declare parameters - actual values should be set in vesc.yaml
        # These are just fallback defaults if YAML is not provided
        self.declare_parameter("voltage_threshold", 0.0)
        self.declare_parameter("check_duration", 1.0)
        self.declare_parameter("warning_threshold", 0.0)

        # Get parameters (loaded from vesc.yaml via launch file)
        self.voltage_threshold = (
            self.get_parameter("voltage_threshold").get_parameter_value().double_value
        )
        self.check_duration = (
            self.get_parameter("check_duration").get_parameter_value().double_value
        )
        self.warning_threshold = (
            self.get_parameter("warning_threshold").get_parameter_value().double_value
        )

        # State tracking
        self.low_voltage_start_time = None
        self.last_voltage = None
        self.warning_issued = False
        self.shutdown_initiated = False

        # Subscribe to VESC state
        self.vesc_state_sub = self.create_subscription(
            VescStateStamped, "/sensors/core", self.vesc_state_callback, 10
        )

        self.get_logger().info(
            f"Starting voltage monitor (threshold: {self.voltage_threshold}V)"
        )
        self.get_logger().info(f"Monitoring topic: /sensors/core")
        self.get_logger().info(f"Check duration: {self.check_duration}s")
        self.get_logger().info("Press Ctrl+C to stop")

    def vesc_state_callback(self, msg):
        """Process VESC state messages and monitor voltage."""
        if self.shutdown_initiated:
            return

        voltage = msg.state.voltage_input
        self.last_voltage = voltage
        current_time = self.get_clock().now()

        # Log current voltage reading
        # self.get_logger().info(f'Current voltage: {voltage:.2f}V')

        if voltage < self.warning_threshold and voltage >= self.voltage_threshold:
            if not self.warning_issued:
                self.get_logger().warn(
                    f"WARNING: Voltage {voltage:.2f}V is below warning threshold {self.warning_threshold}V"
                )
                self.warning_issued = True
        elif voltage >= self.warning_threshold:
            self.warning_issued = False

        if voltage < self.voltage_threshold:
            # Start tracking low voltage duration
            if self.low_voltage_start_time is None:
                self.low_voltage_start_time = current_time
                self.get_logger().warn(
                    f"Low voltage detected: {voltage:.2f}V (threshold: {self.voltage_threshold}V)"
                )
            else:
                # Check if voltage has been low for check_duration
                duration = (
                    current_time - self.low_voltage_start_time
                ).nanoseconds / 1e9
                if duration >= self.check_duration:
                    self.get_logger().error(
                        f"Voltage critically low for {duration:.1f}s. Shutting down system..."
                    )
                    self.shutdown_initiated = True
                    self.shutdown_system()
        else:
            # Voltage recovered
            if self.low_voltage_start_time is not None:
                self.get_logger().info(f"Voltage recovered to {voltage:.2f}V")
                self.low_voltage_start_time = None

    def shutdown_system(self):
        """Shutdown ROS system due to critically low battery"""
        self.get_logger().fatal("=" * 80)
        self.get_logger().fatal("CRITICAL: BATTERY VOLTAGE TOO LOW!")
        self.get_logger().fatal("Stopping all ROS nodes to prevent battery damage.")
        self.get_logger().fatal("MANUALLY SHUTDOWN THE SYSTEM IMMEDIATELY!")
        self.get_logger().fatal("=" * 80)

        # Kill the entire process group (launch file and all child nodes)
        try:
            # Get the process group ID of the current process
            # The launch file is typically the parent of this process
            pgid = os.getpgid(os.getpid())
            self.get_logger().info(f"Sending SIGTERM to process group {pgid}")

            # Send SIGTERM to entire process group
            os.killpg(pgid, signal.SIGTERM)
        except Exception as e:
            self.get_logger().error(f"Failed to kill process group: {e}")
            # Fallback: just shutdown this node
            rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    battery_monitor = VescBatteryMonitor()

    try:
        rclpy.spin(battery_monitor)
    except KeyboardInterrupt:
        pass
    finally:
        battery_monitor.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
