#!/usr/bin/env python3
import sys
import threading
from PyQt5.QtWidgets import QApplication, QVBoxLayout, QHBoxLayout, QPushButton, QWidget, QLabel, QSlider, QGridLayout
from PyQt5.QtCore import Qt
import rclpy
from rclpy.node import Node
from mirobot_msgs.msg import EndeffectorState
from sensor_msgs.msg import JointState
from std_msgs.msg import String


class MirobotInterface(Node):
    def __init__(self):
        super().__init__('mirobot_interface')

        # Publishers
        self.cmd_endeffector_state_publisher = self.create_publisher(EndeffectorState, 'cmd_endeffector_state', 10)
        self.cmd_joint_states_publisher = self.create_publisher(JointState, 'cmd_joint_states', 10)
        self.std_functions_publisher = self.create_publisher(String, 'cmd_std_functions', 10)

        # Subscribers
        self.res_std_functions_subscriber = self.create_subscription(
            JointState,
            'res_std_functions',  # Listen for replies from the std_functions node
            self.handle_res_std_functions,
            10
        )

        # Joint limits extracted from URDF
        self.joint_limits = {
            "joint1": {"lower": -1.7453, "upper": 2.7925},
            "joint2": {"lower": -0.52, "upper": 1.2},
            "joint3": {"lower": -2.09, "upper": 1.045},
            "joint4": {"lower": -3.13, "upper": 3.13},
            "joint5": {"lower": -3.48, "upper": 0.52},
            "joint6": {"lower": -6.26, "upper": 6.26},
        }

        # Initialize the GUI
        self.app = QApplication(sys.argv)
        self.window = QWidget()
        self.window.setWindowTitle('Mirobot Interface')

        # Joint State Controls
        self.joint_sliders = []
        self.joint_labels = []

        # End Effector Controls
        self.ee_label = QLabel("End Effector Control")
        self.button_close_gripper = QPushButton('Close Gripper')
        self.button_open_gripper = QPushButton('Open Gripper')
        self.button_idle_gripper = QPushButton('Idle Gripper')

        self.button_close_gripper.clicked.connect(lambda: self.send_endeffector_state("Close", 0))
        self.button_open_gripper.clicked.connect(lambda: self.send_endeffector_state("Open", 1))
        self.button_idle_gripper.clicked.connect(lambda: self.send_endeffector_state("Idle", 2))

        # Standard Function Buttons
        self.std_functions_label = QLabel("Standard Functions")
        self.button_homing = QPushButton('Homing')
        self.button_centering = QPushButton('Centering')
        self.button_randomizing = QPushButton('Randomizing')

        self.button_homing.clicked.connect(lambda: self.execute_std_function("homing"))
        self.button_centering.clicked.connect(lambda: self.execute_std_function("centering"))
        self.button_randomizing.clicked.connect(lambda: self.execute_std_function("randomizing"))

        # Create sliders and labels for each joint
        joint_layout = QVBoxLayout()
        for i, (joint_name, limits) in enumerate(self.joint_limits.items()):
            joint_label = QLabel(f"{joint_name}: 0°")
            joint_label.setFixedWidth(100)  # Set a fixed width for the label to prevent resizing
            slider = QSlider(Qt.Horizontal)
            slider.setMinimum(int(limits['lower'] * 180.0 / 3.14159))  # Convert radians to degrees
            slider.setMaximum(int(limits['upper'] * 180.0 / 3.14159))
            slider.setValue(0)
            slider.valueChanged.connect(lambda value, index=i: self.slider_value_changed(value, index))

            self.joint_sliders.append(slider)
            self.joint_labels.append(joint_label)

            # Create a horizontal layout for the label and slider
            h_layout = QHBoxLayout()
            h_layout.addWidget(joint_label)
            h_layout.addWidget(slider)

            # Add the horizontal layout to the joint layout
            joint_layout.addLayout(h_layout)

        # Layout
        layout = QVBoxLayout()

        # Add End Effector Controls
        layout.addWidget(self.ee_label)
        layout.addWidget(self.button_close_gripper)
        layout.addWidget(self.button_open_gripper)
        layout.addWidget(self.button_idle_gripper)

        # Add Standard Function Buttons
        layout.addWidget(self.std_functions_label)
        layout.addWidget(self.button_homing)
        layout.addWidget(self.button_centering)
        layout.addWidget(self.button_randomizing)

        # Add Joint State Controls
        layout.addLayout(joint_layout)

        self.window.setLayout(layout)

        # Start a separate thread for the ROS 2 event loop
        self.ros_thread = threading.Thread(target=rclpy.spin, args=(self,), daemon=True)
        self.ros_thread.start()

    def slider_value_changed(self, value, index):
        """Handle slider value changes and publish updated joint states."""
        self.joint_labels[index].setText(f"{list(self.joint_limits.keys())[index]}: {value}°")

        # Create and publish the updated joint states
        joint_state_msg = JointState()
        joint_state_msg.name = list(self.joint_limits.keys())
        joint_state_msg.position = [slider.value() * (3.14159 / 180.0) for slider in self.joint_sliders]  # Convert degrees to radians
        joint_state_msg.header.stamp = self.get_clock().now().to_msg()

        self.cmd_joint_states_publisher.publish(joint_state_msg)
        self.get_logger().info(f"Published updated joint states to cmd_joint_states: {joint_state_msg.position}")

    def send_endeffector_state(self, name, state):
        """Send end effector state commands."""
        msg = EndeffectorState()
        msg.name = name
        msg.state = state
        self.cmd_endeffector_state_publisher.publish(msg)
        self.get_logger().info(f"Published end effector state to cmd_endeffector_state: {name}, state={state}")

    def execute_std_function(self, command):
        """Send a standard function command."""
        msg = String()
        msg.data = command
        self.std_functions_publisher.publish(msg)
        self.get_logger().info(f"Sent standard function command: {command}")

    def handle_res_std_functions(self, msg):
        """Handle the response from the std_functions node and update the GUI."""
        self.get_logger().info(f"Received joint states from res_std_functions: {msg.position}")

        # Update the GUI sliders and labels
        for i, position in enumerate(msg.position):
            if i < len(self.joint_sliders):
                slider = self.joint_sliders[i]
                slider.blockSignals(True)  # Prevent triggering slider events
                slider.setValue(int(position * 180.0 / 3.14159))  # Convert radians to degrees
                slider.blockSignals(False)

                joint_label = self.joint_labels[i]
                joint_label.setText(f"{msg.name[i]}: {int(position * 180.0 / 3.14159)}°")

        # Publish the received joint states
        joint_state_msg = JointState()
        joint_state_msg.name = msg.name
        joint_state_msg.position = msg.position
        joint_state_msg.header.stamp = self.get_clock().now().to_msg()

        self.cmd_joint_states_publisher.publish(joint_state_msg)
        self.get_logger().info(f"Published joint states to cmd_joint_states: {joint_state_msg.position}")

    def run(self):
        self.window.show()
        sys.exit(self.app.exec_())


def main(args=None):
    rclpy.init(args=args)
    interface = MirobotInterface()
    try:
        interface.run()
    except KeyboardInterrupt:
        interface.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
