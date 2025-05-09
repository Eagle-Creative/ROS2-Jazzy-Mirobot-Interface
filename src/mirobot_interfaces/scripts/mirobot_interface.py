#!/usr/bin/env python3
import sys
import random
import threading
from PyQt5.QtWidgets import QApplication, QVBoxLayout, QHBoxLayout, QPushButton, QWidget, QLabel, QSlider, QGridLayout
from PyQt5.QtCore import Qt
import rclpy
from rclpy.node import Node
from mirobot_msgs.msg import EndeffectorState
from sensor_msgs.msg import JointState


class MirobotInterface(Node):
    def __init__(self):
        super().__init__('mirobot_interface')
        self.ee_publisher = self.create_publisher(EndeffectorState, 'endeffector_state', 10)
        self.joint_publisher = self.create_publisher(JointState, 'joint_states', 10)

        # Joint limits extracted from URDF
        self.joint_limits = {
            "joint1": {"lower": -1.7453, "upper": 2.7925},
            "joint2": {"lower": -0.52, "upper": 1.2},
            "joint3": {"lower": -2.09, "upper": 1.045},
            "joint4": {"lower": -3.13, "upper": 3.13},
            "joint5": {"lower": -3.48, "upper": 0.52},
            "joint6": {"lower": -6.26, "upper": 6.26},
        }

        # Create the custom GUI
        self.app = QApplication(sys.argv)
        self.window = QWidget()
        self.window.setWindowTitle('Mirobot Interface')

        # End Effector Controls
        self.ee_label = QLabel("End Effector Control:")
        self.button_0 = QPushButton('Close Gripper')
        self.button_1 = QPushButton('Open Gripper')
        self.button_2 = QPushButton('Idle Gripper')

        self.button_0.clicked.connect(lambda: self.send_endeffector_state("ON", 0))
        self.button_1.clicked.connect(lambda: self.send_endeffector_state("Open", 1))
        self.button_2.clicked.connect(lambda: self.send_endeffector_state("Idle", 2))

        # Joint State Controls
        self.joint_label = QLabel("Joint State Control:")
        self.joint_sliders = []
        self.joint_labels = []

        # Create sliders and labels for each joint
        grid_layout = QGridLayout()
        for i, (joint_name, limits) in enumerate(self.joint_limits.items()):
            joint_label = QLabel(f"{joint_name}: 0°")
            slider = QSlider(Qt.Horizontal)
            slider.setMinimum(int(limits['lower'] * 180.0 / 3.14159))  # Convert radians to degrees
            slider.setMaximum(int(limits['upper'] * 180.0 / 3.14159))
            slider.setValue(0)
            slider.valueChanged.connect(lambda value, label=joint_label, name=joint_name: label.setText(f"{name}: {value}°"))

            self.joint_sliders.append(slider)
            self.joint_labels.append(joint_label)

            # Add label and slider to the grid layout
            grid_layout.addWidget(joint_label, i, 0)
            grid_layout.addWidget(slider, i, 1)

        # Add additional buttons
        self.center_button = QPushButton("Centering")
        self.center_button.clicked.connect(self.center_joints)

        self.randomize_button = QPushButton("Randomize")
        self.randomize_button.clicked.connect(self.randomize_joints)

        self.homing_button = QPushButton("Homing")
        self.homing_button.clicked.connect(self.homing)

        # Layout
        layout = QVBoxLayout()

        # Add End Effector Controls
        layout.addWidget(self.ee_label)
        layout.addWidget(self.button_0)
        layout.addWidget(self.button_1)
        layout.addWidget(self.button_2)

        # Add Joint State Controls
        layout.addWidget(self.joint_label)
        layout.addLayout(grid_layout)

        # Add additional buttons
        layout.addWidget(self.center_button)
        layout.addWidget(self.randomize_button)
        layout.addWidget(self.homing_button)

        self.window.setLayout(layout)

        # Publish initial joint states
        self.publish_joint_states()

        # Set up a timer to publish joint states periodically
        self.timer = self.create_timer(0.1, self.publish_joint_states)  # Publish at 10 Hz

        # Start a separate thread for the ROS 2 event loop
        self.ros_thread = threading.Thread(target=rclpy.spin, args=(self,), daemon=True)
        self.ros_thread.start()

    def send_endeffector_state(self, name, state):
        msg = EndeffectorState()
        msg.name = name
        msg.state = state
        self.ee_publisher.publish(msg)
        self.get_logger().info(f'Sent: name={name}, state={state} to endeffector_state')

    def publish_joint_states(self):
        msg = JointState()
        msg.name = list(self.joint_limits.keys())
        msg.position = [slider.value() * (3.14159 / 180.0) for slider in self.joint_sliders]  # Convert degrees to radians
        msg.header.stamp = self.get_clock().now().to_msg()
        self.joint_publisher.publish(msg)
        self.get_logger().info(f"Published joint states: {msg.position}")

    def center_joints(self):
        """Set all sliders to their center positions and send a single joint state message."""
        for slider, label, joint_name in zip(self.joint_sliders, self.joint_labels, self.joint_limits.keys()):
            slider.blockSignals(True)  # Temporarily block signals to avoid triggering publish_joint_states
            slider.setValue(0)  # Center position
            label.setText(f"{joint_name}: 0°")  # Update the label text
            slider.blockSignals(False)  # Re-enable signals

        # Publish a single joint state message after updating all sliders
        self.publish_joint_states()

    def randomize_joints(self):
        """Set all sliders to random positions within their limits and send a single joint state message."""
        for slider in self.joint_sliders:
            slider.blockSignals(True)  # Temporarily block signals to avoid triggering publish_joint_states
            slider.setValue(random.randint(slider.minimum(), slider.maximum()))
            slider.blockSignals(False)  # Re-enable signals

        # Publish a single joint state message after updating all sliders
        self.publish_joint_states()

    def homing(self):
        """Set all sliders to their minimum positions."""
        for slider in self.joint_sliders:
            slider.setValue(slider.minimum())

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
