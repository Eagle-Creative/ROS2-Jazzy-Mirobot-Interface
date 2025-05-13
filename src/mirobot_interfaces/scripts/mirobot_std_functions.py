#!/usr/bin/env python3
# filepath: /home/jonas/Desktop/Dev/Mirobot/src/mirobot_interfaces/scripts/mirobot_std_functions.py

import rclpy
from rclpy.node import Node
from mirobot_msgs.msg import StdFunction  # Import the StdFunction message
from std_msgs.msg import String  # Import String for GCODE commands
from sensor_msgs.msg import JointState
import random
import time


class MirobotStdFunctions(Node):
    def __init__(self):
        super().__init__('mirobot_std_functions')

        # Publisher for cmd_joint_states
        self.joint_publisher = self.create_publisher(JointState, 'cmd_joint_states', 10)

        # Publisher for cmd_function (GCODE override commands)
        self.cmd_function_publisher = self.create_publisher(String, 'cmd_function', 10)

        # Subscriber for standard function commands
        self.command_subscriber = self.create_subscription(
            StdFunction,
            'standard_function_command',
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
        command = msg.command.lower()
        if command == "homing":
            self.homing()
        elif command == "centering":
            self.centering()
        elif command == "randomizing":
            self.randomizing()
        else:
            self.get_logger().warn(f"Unknown command: {command}")

    def homing(self):
        """Publish the homing GCODE command to cmd_function and set all joints to 0 after a delay."""
        # Publish the GCODE command for homing
        gcode_msg = String()
        gcode_msg.data = "$H\r\n"  # GCODE command for homing
        self.cmd_function_publisher.publish(gcode_msg)
        self.get_logger().info("Homing GCODE command published to cmd_function.")

        # Wait for the homing sequence to complete
        time.sleep(5)  # Adjust the delay as needed

        # Set all joint states to 0
        joint_state = JointState()
        joint_state.name = list(self.joint_limits.keys())
        joint_state.position = [0.0 for _ in self.joint_limits.values()]
        self.joint_publisher.publish(joint_state)
        self.get_logger().info("Homing completed. All joint states set to 0.")

    def centering(self):
        """Set all joints to their center positions."""
        joint_state = JointState()
        joint_state.name = list(self.joint_limits.keys())
        joint_state.position = [
            0 for limits in self.joint_limits.values()
        ]
        self.joint_publisher.publish(joint_state)
        self.get_logger().info("Centering completed and published to cmd_joint_states.")

    def randomizing(self):
        """Set all joints to random positions within their limits."""
        joint_state = JointState()
        joint_state.name = list(self.joint_limits.keys())
        joint_state.position = [
            random.uniform(limits["lower"], limits["upper"]) for limits in self.joint_limits.values()
        ]
        self.joint_publisher.publish(joint_state)
        self.get_logger().info("Randomizing completed and published to cmd_joint_states.")


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