#!/usr/bin/env python3
# filepath: /home/jonas/Desktop/Dev/Mirobot/src/mirobot_interfaces/scripts/mirobot_std_functions.py

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import JointState
import random
import time


class MirobotStdFunctions(Node):
    def __init__(self):
        super().__init__('mirobot_std_functions')

        # Publisher for res_std_functions
        self.res_std_functions_publisher = self.create_publisher(JointState, 'res_std_functions', 10)

        # Publisher for cmd_function (to send G-code commands)
        self.cmd_function_publisher = self.create_publisher(String, 'cmd_function', 10)

        # Subscriber for standard function commands
        self.command_subscriber = self.create_subscription(
            String,
            'cmd_std_functions',
            self.handle_command,
            10
        )

        # Joint limits (example values)
        self.joint_limits = {
            "joint1": {"lower": -1.7453, "upper": 2.7925},
            "joint2": {"lower": -0.52, "upper": 1.2},
            "joint3": {"lower": -2.09, "upper": 1.045},
            "joint4": {"lower": -3.13, "upper": 3.13},
            "joint5": {"lower": -3.48, "upper": 0.52},
            "joint6": {"lower": -6.26, "upper": 6.26},
        }

        self.get_logger().info("Mirobot Standard Functions Node is ready.")

    def handle_command(self, msg):
        """Handle incoming commands for standard functions."""
        command = msg.data.lower()
        if command == "homing":
            self.homing()
        elif command == "centering":
            self.centering()
        elif command == "randomizing":
            self.randomizing()
        else:
            self.get_logger().warn(f"Unknown command: {command}")

    def homing(self):
        """Simulate homing by sending a $H command to the G-code writer."""
        self.get_logger().info("Executing homing...")

        # Send the $H command to the G-code writer
        gcode_msg = String()
        gcode_msg.data = "$H\r\n"
        self.cmd_function_publisher.publish(gcode_msg)
        self.get_logger().info("Homing command ($H) sent to G-code writer.")

        # Publish the joint state after homing
        joint_state = JointState()
        joint_state.name = list(self.joint_limits.keys())
        joint_state.position = [0.0 for _ in self.joint_limits.values()]
        self.res_std_functions_publisher.publish(joint_state)
        self.get_logger().info("Homing completed and published to res_std_functions.")

    def centering(self):
        """Set all joints to their center positions."""
        self.get_logger().info("Executing centering...")

        joint_state = JointState()
        joint_state.name = list(self.joint_limits.keys())
        joint_state.position = [
            0 for limits in self.joint_limits.values()
        ]
        self.res_std_functions_publisher.publish(joint_state)
        self.get_logger().info("Centering completed and published to res_std_functions.")

    def randomizing(self):
        """Set all joints to random positions within their limits."""
        self.get_logger().info("Executing randomizing...")

        joint_state = JointState()
        joint_state.name = list(self.joint_limits.keys())
        joint_state.position = [
            random.uniform(limits["lower"], limits["upper"]) for limits in self.joint_limits.values()
        ]
        self.res_std_functions_publisher.publish(joint_state)
        self.get_logger().info("Randomizing completed and published to res_std_functions.")


def main(args=None):
    rclpy.init(args=args)
    node = MirobotStdFunctions()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()