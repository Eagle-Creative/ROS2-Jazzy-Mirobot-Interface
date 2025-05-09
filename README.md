# ROS2-Jazzy Wlkata Mirobot Interface

This project provides a ROS 2 interface for controlling the Wlkata Mirobot robotic arm. It includes nodes for sending G-code commands, subscribing to joint states, and controlling the end effector.

## Features

- **G-code Command Execution**: Send G-code commands to the Mirobot via a serial connection.
- **Joint State Subscription**: Subscribe to joint states and convert them into G-code commands.
- **End Effector Control**: Control the state of the end effector using custom messages.
- **Homing Functionality**: Perform homing operations to initialize the robot.

## Project Structure

```
LICENSE
README.md
build/
    COLCON_IGNORE
    mirobot_description/
        AMENT_IGNORE
        ...
    mirobot_interfaces/
        AMENT_IGNORE
        ...
    mirobot_msgs/
        ...
install/
    _local_setup_util_ps1.py
    ...
src/
    mirobot_description/
    mirobot_interfaces/
    mirobot_msgs/
```

### Key Directories

- **`src/mirobot_description`**: Contains the URDF files, launch files, and nodes for controlling the Mirobot.
- **`src/mirobot_interfaces`**: Implements the G-code writer node and serial communication logic.
- **`src/mirobot_msgs`**: Defines custom ROS 2 messages for the project.

## Installation

1. Clone the repository:
   ```bash
   git clone https://github.com/your-repo/mirobot.git
   cd mirobot
   ```

2. Install the `serial_driver` package:
   ```bash
   sudo apt update
   sudo apt install ros-humble-serial-driver
   ```

3. Build the workspace:
   ```bash
   colcon build
   ```

4. Source the setup file:
   ```bash
   source install/setup.bash
   ```

5. Make the necessary scripts executable:
   ```bash
   chmod +x src/mirobot_description/launch/mirobot_description.launch.py
   chmod +x src/mirobot_description/launch/mirobot_isaac_control.launch.py
   ```

## Usage

### Launching the Robot Description

To launch the robot description and visualization in RViz, run:
```bash
ros2 launch mirobot_interfaces mirobot_interfaces.launch.py
```

## Dependencies

This project depends on the following ROS 2 packages:

- `rclcpp`
- `sensor_msgs`
- `std_msgs`
- `serial_driver`
- `rosidl_default_generators`

## Custom Messages

The project defines a custom message, `EndeffectorState`, in the `mirobot_msgs` package. This message is used to control the state of the end effector.

## License

This project is licensed under the Apache License 2.0. See the [LICENSE](LICENSE) file for details.

## Contributing

Contributions are welcome! Please follow the standard GitHub workflow:

1. Fork the repository.
2. Create a new branch for your feature or bug fix.
3. Submit a pull request with a detailed description of your changes.

## Acknowledgments

This project is inspired by the Wlkata Mirobot and aims to provide a robust ROS 2 interface for its control.
