#!/usr/bin/env python3
"""
Script to verify that odometry correctly interprets servo commands as steering angles.
This subscribes to the servo command topic and calculates what the odometry thinks
the steering angle is, based on the parameters in vesc.yaml.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
import math

# This file mirrors the logic in vesc_to_odom.cpp, calculating the actual wheel steering angle using ros topic sensors/servo_position_command
# Usage:
# 1. Run the ROS2 nodes with a launch file, such as ros2 launch f1tenth_stack no_lidar_bringup_launch.py 
# 2. Run the run_steering_checker.sh test script, and observe what the odometry guesses the true wheel steering angle to be
class SteeringAngleChecker(Node):
    def __init__(self):
        super().__init__('steering_angle_checker')

        # Declare default steering and wheelbase parameters (same as vesc_to_odom_node in vesc.yaml)
        self.declare_parameter('steering_angle_to_servo_gain', -0.696)
        self.declare_parameter('steering_angle_to_servo_offset', 0.475)
        self.declare_parameter('wheelbase', 0.33)

        # run_steering_checker.sh overwrites these command with actual config values from vesc.yaml
        self.gain = self.get_parameter('steering_angle_to_servo_gain').value
        self.offset = self.get_parameter('steering_angle_to_servo_offset').value
        self.wheelbase = self.get_parameter('wheelbase').value

        # Subscribe to servo command (same topic as vesc_to_odom_node)
        self.servo_sub = self.create_subscription(
            Float64,
            'sensors/servo_position_command',
            self.servo_callback,
            10
        )

        self.get_logger().info('=' * 60)
        self.get_logger().info('Steering Angle Checker Started')
        self.get_logger().info('=' * 60)
        self.get_logger().info(f'Parameters:')
        self.get_logger().info(f'  steering_angle_to_servo_gain: {self.gain}')
        self.get_logger().info(f'  steering_angle_to_servo_offset: {self.offset}')
        self.get_logger().info(f'  wheelbase: {self.wheelbase} m')
        self.get_logger().info('')
        self.get_logger().info('Expected values (from physical measurements):')
        self.get_logger().info('  Servo 0.10 → 0.56 rad (max left)')
        self.get_logger().info('  Servo 0.85 → -0.52 rad (max right)')
        self.get_logger().info('')
        self.get_logger().info('Waiting for servo commands...')
        self.get_logger().info('=' * 60)

    def servo_callback(self, msg):
        servo_value = msg.data

        # Calculate steering angle using same formula as vesc_to_odom.cpp:113-114
        steering_angle = (servo_value - self.offset) / self.gain

        # Calculate turning radius from steering angle
        # R = wheelbase / tan(steering_angle)
        if abs(steering_angle) > 0.01:  # Avoid division by zero
            turning_radius = self.wheelbase / math.tan(steering_angle)
            turning_diameter = 2 * turning_radius
        else:
            turning_radius = float('inf')
            turning_diameter = float('inf')

        # Log the results
        self.get_logger().info(
            f'Servo: {servo_value:.3f} → '
            f'Angle: {steering_angle:.3f} rad ({math.degrees(steering_angle):.1f}°) | '
            f'R: {turning_radius:.3f} m | '
            f'D: {turning_diameter:.3f} m | '
        )


def main(args=None):
    rclpy.init(args=args)
    node = SteeringAngleChecker()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
