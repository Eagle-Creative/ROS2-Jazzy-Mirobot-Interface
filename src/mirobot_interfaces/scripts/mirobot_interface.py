#!/usr/bin/env python3
import sys
from PyQt5.QtWidgets import QApplication, QVBoxLayout, QPushButton, QWidget, QLabel
import rclpy
from rclpy.node import Node
from mirobot_msgs.msg import EndeffectorState  # Import the custom message


class MirobotInterface(Node):
    def __init__(self):
        super().__init__('mirobot_interface')
        self.publisher = self.create_publisher(EndeffectorState, 'endeffector_state', 10)

        # Create the custom GUI
        self.app = QApplication(sys.argv)
        self.window = QWidget()
        self.window.setWindowTitle('Mirobot Interface')

        # Create buttons for controlling the end effector
        self.label = QLabel("End Effector Control:")
        self.button_0 = QPushButton('Close Gripper')
        self.button_1 = QPushButton('Open Gripper')
        self.button_2 = QPushButton('Idle Gripper')

        # Connect button clicks to functions
        self.button_0.clicked.connect(lambda: self.send_state("ON", 0))
        self.button_1.clicked.connect(lambda: self.send_state("Open", 1))
        self.button_2.clicked.connect(lambda: self.send_state("Idle", 2))

        # Layout
        layout = QVBoxLayout()
        layout.addWidget(self.label)
        layout.addWidget(self.button_0)
        layout.addWidget(self.button_1)
        layout.addWidget(self.button_2)
        self.window.setLayout(layout)

    def send_state(self, name, state):
        msg = EndeffectorState()
        msg.name = name
        msg.state = state
        self.publisher.publish(msg)
        #self.get_logger().info(f'Sent: name={name}, state={state} to endeffector_state')

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
