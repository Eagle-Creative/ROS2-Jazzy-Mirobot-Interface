#!/usr/bin/env python3
# filepath: /home/jonas/Desktop/Dev/Mirobot/src/mirobot_interfaces/scripts/mirobot_controller.py

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from mirobot_msgs.msg import EndeffectorState


class MirobotControllerNode(Node):
    def __init__(self):
        super().__init__('mirobot_controller_node')

        # Declare and get parameters from config file
        self.declare_parameter('topics.prefix', '')
        self.declare_parameter('topics.cmd_joint_states', 'cmd_joint_states')
        self.declare_parameter('topics.joint_states', 'joint_states')
        self.declare_parameter('topics.cmd_endeffector_state', 'cmd_endeffector_state')
        self.declare_parameter('topics.endeffector_state', 'endeffector_state')
        self.declare_parameter('controller.publish_rate_hz', 2.0)
        
        # Get topic prefix and apply it
        prefix = self.get_parameter('topics.prefix').get_parameter_value().string_value
        if prefix and not prefix.endswith('/'):
            prefix += '/'
        
        cmd_joint_states_topic = prefix + self.get_parameter('topics.cmd_joint_states').get_parameter_value().string_value
        joint_states_topic = prefix + self.get_parameter('topics.joint_states').get_parameter_value().string_value
        cmd_endeffector_topic = prefix + self.get_parameter('topics.cmd_endeffector_state').get_parameter_value().string_value
        endeffector_topic = prefix + self.get_parameter('topics.endeffector_state').get_parameter_value().string_value
        publish_rate_hz = self.get_parameter('controller.publish_rate_hz').get_parameter_value().double_value

        self.get_logger().info(f'Configuration loaded:')
        self.get_logger().info(f'  Publish Rate: {publish_rate_hz} Hz')
        self.get_logger().info(f'  Topics: {cmd_joint_states_topic} -> {joint_states_topic}')

        # Subscribers
        self.create_subscription(JointState, cmd_joint_states_topic, self.cmd_joint_states_callback, 10)
        self.create_subscription(EndeffectorState, cmd_endeffector_topic, self.cmd_endeffector_state_callback, 10)

        # Publishers
        # NOTE: joint_states publisher removed - only mirobot_gcode_writer publishes actual robot feedback
        # self.joint_states_publisher = self.create_publisher(JointState, joint_states_topic, 10)
        self.endeffector_state_publisher = self.create_publisher(EndeffectorState, endeffector_topic, 10)

        # Initialize the latest joint state with default values
        self.latest_joint_state = JointState()
        self.latest_joint_state.name = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"]
        self.latest_joint_state.position = [0.0] * len(self.latest_joint_state.name)
        self.latest_joint_state.velocity = []
        self.latest_joint_state.effort = []

        # Timer to continuously publish removed - joint states come from actual robot telemetry
        # timer_period = 1.0 / publish_rate_hz  # Calculate period from rate
        # self.timer = self.create_timer(timer_period, self.publish_latest_joint_state)

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

    # NOTE: publish_latest_joint_state function disabled - joint states published by mirobot_gcode_writer only
    # def publish_latest_joint_state(self):
    #     """Continuously publish the latest joint state."""
    #     if self.latest_joint_state.name:  # Ensure the latest joint state is not empty
    #         # Create a new JointState message
    #         joint_state_msg = JointState()
    #         joint_state_msg.name = self.latest_joint_state.name
    #         joint_state_msg.position = self.latest_joint_state.position
    #         joint_state_msg.velocity = self.latest_joint_state.velocity
    #         joint_state_msg.effort = self.latest_joint_state.effort
    #         joint_state_msg.header.stamp = self.get_clock().now().to_msg()  # Update the timestamp
    #
    #         # Publish the new JointState message
    #         self.joint_states_publisher.publish(joint_state_msg)
    #         # Publishing joint states (silent)


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
        if rclpy.ok():  # already shut down by the SIGINT handler under launch
            rclpy.shutdown()


if __name__ == '__main__':
    main()