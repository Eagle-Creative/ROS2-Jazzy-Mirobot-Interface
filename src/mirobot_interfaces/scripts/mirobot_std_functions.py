#!/usr/bin/env python3
# filepath: /home/jonas/Desktop/Dev/Mirobot/src/mirobot_interfaces/scripts/mirobot_std_functions.py

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import JointState
import random
import time


class MirobotStdFunctions(Node):
    """
    A ROS 2 node that provides standard functions for the Mirobot, such as homing, centering, and randomizing.
    """

    def __init__(self):
        """
        Initialize the MirobotStdFunctions node, publishers, subscribers, and joint limits.
        """
        super().__init__('mirobot_std_functions')

        # Declare and get parameters from config file
        self.declare_parameter('topics.prefix', '')
        self.declare_parameter('topics.res_std_functions', 'res_std_functions')
        self.declare_parameter('topics.cmd_function', 'cmd_function')
        self.declare_parameter('topics.cmd_std_functions', 'cmd_std_functions')
        
        # Get topic prefix and apply it
        prefix = self.get_parameter('topics.prefix').get_parameter_value().string_value
        if prefix and not prefix.endswith('/'):
            prefix += '/'
        
        res_std_functions_topic = prefix + self.get_parameter('topics.res_std_functions').get_parameter_value().string_value
        cmd_function_topic = prefix + self.get_parameter('topics.cmd_function').get_parameter_value().string_value
        cmd_std_functions_topic = prefix + self.get_parameter('topics.cmd_std_functions').get_parameter_value().string_value

        self.get_logger().info(f'Configuration loaded')
        self.get_logger().info(f'  Topics: {cmd_std_functions_topic} -> {res_std_functions_topic}')

        # Publisher for res_std_functions
        self.res_std_functions_publisher = self.create_publisher(JointState, res_std_functions_topic, 10)

        # Publisher for cmd_function (to send G-code commands)
        self.cmd_function_publisher = self.create_publisher(String, cmd_function_topic, 10)

        # Subscriber for standard function commands
        self.command_subscriber = self.create_subscription(
            String,
            cmd_std_functions_topic,
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
        """
        Handle incoming commands for standard functions.

        Args:
            msg (std_msgs.msg.String): The command message containing the function to execute.
        """
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
        """
        Perform the homing operation by sending a $H command to the G-code writer and resetting joint states to 0.
        """
        self.get_logger().info("Executing homing...")

        # Send the $H command to the G-code writer
        gcode_msg = String()
        gcode_msg.data = "$H\r\n"
        self.cmd_function_publisher.publish(gcode_msg)
        self.get_logger().info("Homing command ($H) sent to G-code writer.")

        # Simulate delay for homing
        time.sleep(2)

        # Publish the joint state after homing
        joint_state = JointState()
        joint_state.name = list(self.joint_limits.keys())
        joint_state.position = [0.0 for _ in self.joint_limits.values()]
        self.res_std_functions_publisher.publish(joint_state)
        self.get_logger().info("Homing completed and published to res_std_functions.")

    def centering(self):
        """
        Set all joints to their 0 position (position after homing) and publish the joint states.
        """
        self.get_logger().info("Executing centering...")

        joint_state = JointState()
        joint_state.name = list(self.joint_limits.keys())
        joint_state.position = [
            (limits["lower"] + limits["upper"]) / 2 for limits in self.joint_limits.values()
        ]
        self.res_std_functions_publisher.publish(joint_state)
        self.get_logger().info("Centering completed and published to res_std_functions.")

    def randomizing(self):
        """
        Randomize all joints by setting them to random positions within their limits and publishing the joint states.
        """
        self.get_logger().info("Executing randomizing...")

        joint_state = JointState()
        joint_state.name = list(self.joint_limits.keys())
        joint_state.position = [
            random.uniform(limits["lower"], limits["upper"]) for limits in self.joint_limits.values()
        ]
        self.res_std_functions_publisher.publish(joint_state)
        self.get_logger().info("Randomizing completed and published to res_std_functions.")


def main(args=None):
    """
    Main entry point for the MirobotStdFunctions node.
    """
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