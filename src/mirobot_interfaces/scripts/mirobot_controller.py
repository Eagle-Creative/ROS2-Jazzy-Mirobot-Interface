#!/usr/bin/env python3
# filepath: /home/jonas/Desktop/Dev/Mirobot/src/mirobot_interfaces/scripts/mirobot_controller.py

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from mirobot_msgs.msg import EndeffectorState


class MirobotControllerNode(Node):
    def __init__(self):
        super().__init__('mirobot_controller_node')

        # Declare and get parameters
        self.declare_parameter('cmd_joint_states_topic', 'cmd_joint_states')
        self.cmd_joint_states_topic = self.get_parameter('cmd_joint_states_topic').get_parameter_value().string_value

        # Subscribers
        self.create_subscription(JointState, self.cmd_joint_states_topic, self.cmd_joint_states_callback, 10)
        self.create_subscription(EndeffectorState, 'cmd_endeffector_state', self.cmd_endeffector_state_callback, 10)

        # Publishers
        self.joint_states_publisher = self.create_publisher(JointState, 'joint_states', 10)
        self.endeffector_state_publisher = self.create_publisher(EndeffectorState, 'endeffector_state', 10)

        # Initialize the latest joint state with default values
        self.latest_joint_state = JointState()
        self.latest_joint_state.name = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"]
        self.latest_joint_state.position = [0.0] * len(self.latest_joint_state.name)
        self.latest_joint_state.velocity = []
        self.latest_joint_state.effort = []

        # Timer to continuously publish the latest joint state
        self.timer = self.create_timer(0.1, self.publish_latest_joint_state)  # Publish at 10 Hz

        self.get_logger().info('Mirobot Controller Node is ready.')

    def cmd_joint_states_callback(self, msg):
        """Callback for cmd_joint_states topic."""
        # Update the latest joint state
        self.latest_joint_state = msg

        # Convert joint positions (radians) to degrees
        angles = [pos * 57.296 for pos in msg.position]  # 57.296 = 180 / π

        # Format the G-code string
        gcode_string = f"M21 G0 X{angles[0]:.2f} Y{angles[1]:.2f} Z{angles[2]:.2f} A{angles[3]:.2f} B{angles[4]:.2f} C{angles[5]:.2f} F3000\r\n"

        # Log the generated G-code
        self.get_logger().info(f"Generated G-code: {gcode_string}")

        # Send the G-code to the Mirobot (serial communication logic can be added here)
        # Example: self.send_to_mirobot(gcode_string)

    def cmd_endeffector_state_callback(self, msg):
        """Callback for cmd_endeffector_state topic."""
        # Republish the endeffector state to the endeffector_state topic
        self.endeffector_state_publisher.publish(msg)
        self.get_logger().info(f"Republished end effector state to 'endeffector_state': {msg.name}, state={msg.state}")

    def publish_latest_joint_state(self):
        """Continuously publish the latest joint state."""
        if self.latest_joint_state.name:  # Ensure the latest joint state is not empty
            # Create a new JointState message
            joint_state_msg = JointState()
            joint_state_msg.name = self.latest_joint_state.name
            joint_state_msg.position = self.latest_joint_state.position
            joint_state_msg.velocity = self.latest_joint_state.velocity
            joint_state_msg.effort = self.latest_joint_state.effort
            joint_state_msg.header.stamp = self.get_clock().now().to_msg()  # Update the timestamp

            # Publish the new JointState message
            self.joint_states_publisher.publish(joint_state_msg)
            self.get_logger().info(f"Published latest joint states to 'joint_states' topic: {joint_state_msg.position}")

    # Placeholder for serial communication logic
    def send_to_mirobot(self, gcode_string):
        """Send the G-code to the Mirobot."""
        # Add serial communication logic here to send the G-code to the Mirobot
        pass


def main(args=None):
    rclpy.init(args=args)
    node = MirobotControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down Mirobot Controller Node...')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()