# ROS2-Jazzy Wlkata Mirobot Interface

A ROS 2 - Jazzy interface for controlling the Wlkata Mirobot 6-DOF robotic arm with support for joint control, Cartesian positioning, MoveIt 2 motion planning, and flexible configuration.

[![ROS 2 Jazzy](https://img.shields.io/badge/ROS%202-Jazzy-blue)](https://docs.ros.org/en/jazzy/)
[![License](https://img.shields.io/badge/License-Apache%202.0-green.svg)](LICENSE)

## Quick Start

```bash
# Clone and build
git clone https://github.com/Eagle-Creative/ROS2-Jazzy-Mirobot-Interface.git
cd ROS2-Jazzy-Mirobot-Interface
sudo apt install ros-jazzy-serial-driver python3-pyqt5
colcon build
source install/setup.bash

# Check your serial port
ls -la /dev/ttyUSB*  # Update config if not /dev/ttyUSB1

# Launch with GUI
ros2 launch mirobot_interfaces mirobot_interface.launch.py

# Or launch headless (for integration)
ros2 launch mirobot_interfaces mirobot_headless.launch.py

# Optional: MoveIt 2 planning for the arm (needs ros-jazzy-moveit)
ros2 launch mirobot_moveit_config move_group.launch.py
```

## Features

- **Multiple Control Modes**:
  - Joint angle control (6-DOF, radian-based)
  - Cartesian position control with onboard inverse kinematics
  - Point-to-point (PTP) and linear interpolation movements
- **G-code Command Execution**: Direct serial communication with Mirobot firmware
- **End Effector Control**: Gripper control with open/close/idle states
- **Real-time Joint State Publishing**: Continuous state feedback at configurable rates
- **Homing Functionality**: Automatic robot initialization on startup
- **Graphical User Interface (GUI)**: PyQt5-based control interface with:
  - Interactive joint angle sliders
  - Real-time position monitoring
  - End effector control buttons
  - Standard function shortcuts (homing, centering, randomizing)
- **Headless Operation**: Run without GUI for integration with other ROS projects
- **Flexible Configuration**: All parameters configurable via YAML file
- **Standard Functions Library**: Pre-programmed movements and utilities
- **RViz Visualization**: Real-time 3D visualization of robot state
- **MoveIt 2 Planning** (`mirobot_moveit_config`): Pilz PTP/LIN and OMPL planners, KDL IK,
  self-collision model and a tool centre point frame calibrated against the firmware

## Compatibility

This repository is specifically designed to work with **ROS 2 Jazzy** and is not compatible with other versions of ROS 2. Please ensure that you are using the correct ROS 2 distribution to avoid compatibility issues.

**Hardware Requirements:**
- Wlkata Mirobot robotic arm
- USB serial connection (typically `/dev/ttyUSB0` or `/dev/ttyUSB1`)
- Linux system (tested on Ubuntu)

## Installation

### Prerequisites

Ensure you have ROS 2 Jazzy installed on your system. If not, follow the [official ROS 2 installation guide](https://docs.ros.org/en/jazzy/Installation.html).

### Step-by-Step Installation

1. **Clone the repository:**
   ```bash
   git clone https://github.com/Eagle-Creative/ROS2-Jazzy-Mirobot-Interface.git
   cd ROS2-Jazzy-Mirobot-Interface
   ```

2. **Install the `serial_driver` package:**
   ```bash
   sudo apt update
   sudo apt install ros-jazzy-serial-driver
   ```

3. **Install additional ROS 2 dependencies:**
   ```bash
   sudo apt install python3-colcon-common-extensions \
                    ros-jazzy-rviz2 \
                    ros-jazzy-ros2launch \
                    python3-pyqt5
   ```

4. **Configure your serial port:**
   
   Check which USB port your Mirobot is connected to:
   ```bash
   ls -la /dev/ttyUSB*
   ```
   
   Update the configuration file if needed:
   ```bash
   nano src/mirobot_interfaces/config/mirobot_config.yaml
   # Edit the serial.port value to match your device
   ```

5. **Build the workspace:**
   ```bash
   colcon build
   ```

6. **Source the setup file:**
   ```bash
   source install/setup.bash
   ```

**Note:** Add the source command to your `~/.bashrc` for automatic sourcing:
```bash
echo "source ~/path/to/ROS2-Jazzy-Mirobot-Interface/install/setup.bash" >> ~/.bashrc
```

## Usage

### Launch Options

#### 1. Full Interface with GUI and RViz (Recommended for Development)

Launch the complete system including GUI, RViz visualization, and all control nodes:
```bash
ros2 launch mirobot_interfaces mirobot_interface.launch.py
```

This will:
- Start the Mirobot G-code writer (serial communication)
- Launch the PyQt5 GUI for manual control
- Start the controller node for continuous state publishing
- Launch RViz for 3D visualization
- Initialize robot homing sequence (20 second wait)

#### 2. Headless Mode (For Integration with Other Projects)

Launch Mirobot without GUI or RViz - ideal for programmatic control:
```bash
ros2 launch mirobot_interfaces mirobot_headless.launch.py
```

Optional arguments:
```bash
# Use a custom configuration file
ros2 launch mirobot_interfaces mirobot_headless.launch.py \
    config_file:=/path/to/custom_config.yaml

# Enable RViz in headless mode
ros2 launch mirobot_interfaces mirobot_headless.launch.py \
    enable_rviz:=true
```

### Control Methods

#### Joint Angle Control

Control the robot by specifying joint angles (in radians):

```bash
# Example: Move all joints to home position
ros2 topic pub --once /cmd_joint_states sensor_msgs/msg/JointState \
  "{name: ['joint1','joint2','joint3','joint4','joint5','joint6'],
    position: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]}"
```

#### Cartesian Position Control

Control the end-effector position directly (Mirobot uses onboard IK):

```bash
# Point-to-point movement
ros2 topic pub --once /cmd_cartesian_pose mirobot_msgs/msg/CartesianPose \
  "{x: 150.0, y: 0.0, z: 150.0, 
    roll: 0.0, pitch: 0.0, yaw: 0.0,
    mode: 0, speed: 2000.0}"

# Linear interpolation movement
ros2 topic pub --once /cmd_cartesian_pose mirobot_msgs/msg/CartesianPose \
  "{x: 200.0, y: 50.0, z: 100.0,
    roll: 0.0, pitch: 45.0, yaw: 0.0,
    mode: 1, speed: 1000.0}"
```

**Cartesian Pose Parameters:**
- `x, y, z`: Position in millimeters
- `roll, pitch, yaw`: Orientation in degrees
- `mode`: 0 = Point-to-point (G0), 1 = Linear interpolation (G1)
- `speed`: Feed rate in mm/s

#### Gripper Control

Control the end effector:

```bash
# Open gripper
ros2 topic pub --once /cmd_endeffector_state mirobot_msgs/msg/EndeffectorState "{state: 1}"

# Close gripper
ros2 topic pub --once /cmd_endeffector_state mirobot_msgs/msg/EndeffectorState "{state: 0}"

# Idle (stop)
ros2 topic pub --once /cmd_endeffector_state mirobot_msgs/msg/EndeffectorState "{state: 2}"
```

#### Standard Functions

Execute pre-programmed movements:

```bash
# Home the robot
ros2 topic pub --once /cmd_std_functions std_msgs/msg/String "{data: 'homing'}"

# Center position
ros2 topic pub --once /cmd_std_functions std_msgs/msg/String "{data: 'centering'}"

# Random position (for testing)
ros2 topic pub --once /cmd_std_functions std_msgs/msg/String "{data: 'randomizing'}"
```

### Configuration

All system parameters are configured in `src/mirobot_interfaces/config/mirobot_config.yaml`:

```yaml
/**:
  ros__parameters:
    # Serial port configuration
    serial:
      port: "/dev/ttyUSB1"
      baud_rate: 115200
    
    # Controller settings
    controller:
      publish_rate_hz: 2.0
    
    # Robot behavior
    robot:
      perform_homing_on_startup: true
      homing_timeout_seconds: 60   # upper bound; completion is detected from the firmware (Idle after $H)
    
    # Topic names (all configurable)
    topics:
      joint_states: "joint_states"
      cmd_cartesian_pose: "/cmd_cartesian_pose"
      # ... etc
```

**Common Configuration Changes:**

- **Serial Port**: Change `serial.port` to match your device (`/dev/ttyUSB0`, `/dev/ttyUSB1`, etc.)
- **Disable Auto-Homing**: Set `robot.perform_homing_on_startup: false`
- **Adjust Update Rate**: Modify `controller.publish_rate_hz` (default: 2 Hz)
- **Custom Topic Namespace**: Add prefixes to all topic names for multi-robot systems

For detailed configuration options, see [`src/mirobot_interfaces/CONFIGURATION.md`](src/mirobot_interfaces/CONFIGURATION.md).

## MoveIt 2 Motion Planning

`mirobot_moveit_config` plans for the arm with MoveIt 2. It is **planning only**: the Mirobot
has no `ros2_control` interface, so `move_group` runs with trajectory execution disabled. Planned
joint targets are sent through the driver (see [Executing a plan](#executing-a-plan)).

```bash
sudo apt install ros-jazzy-moveit          # move_group, Pilz, OMPL, KDL
colcon build --packages-select mirobot_moveit_config
ros2 launch mirobot_interfaces mirobot_headless.launch.py      # driver: /mirobot_01/joint_states
ros2 launch mirobot_moveit_config move_group.launch.py         # robot_state_publisher + move_group
```

| | |
|---|---|
| Planning group | `mirobot_arm` (chain `base_link` → `mirobot_tcp`) |
| Named state | `home` (all joints 0, the pose after `$H`) |
| Planners | Pilz industrial (`PTP` default, `LIN`, `CIRC`), OMPL |
| IK | KDL |
| Joint states | `/<robot_name>/joint_states` (launch argument `robot_name`, default `mirobot_01`) |
| Tool | launch argument `tool_length` (m), default `0.0365` = WLKATA suction cup assembly |

Note: the headless launch starts its own `robot_state_publisher` with the plain description; when
running MoveIt alongside, the TFs are identical except for the extra `mirobot_tcp` frame.

### Tool centre point (`mirobot_tcp`)

`mirobot_tcp` sits on the tool axis with Z pointing along the tool (down at home):

- The firmware's own Cartesian point is **8 mm beyond `Link6`'s origin** along the tool axis. At
  home the URDF puts `Link6` at z = 238.7 mm and the firmware reports (198.67, 0, 230.72) mm.
  All six joint directions match the firmware's Cartesian output.
- That firmware point is the **J6 flange face** where the tool adapter is bolted on (measured: about
  230 mm above the mounting surface at home, matching the firmware's home Z with z 0 at the base
  underside). Note that the orange wrist cover ends about 15 mm *above* this face.
- `tool_length` is measured **from the flange face** to the tool tip. WLKATA suction cup assembly
  (adapter + vacuum block + cup): `tool_length:=0.0365` (the default). For another tool, measure
  from the flange face.
- Correction (2026-10-26): earlier versions used 0.0575 (52 mm from the end of the orange cover,
  plus 5.5 mm assumed between firmware point and cover end). That was 21 mm too long; a touch-off
  with it reports a table 21 mm below the base, as stated in older versions of this README.

### Measured on a real unit

These numbers come from one Mirobot bolted directly to a wooden plate, checked by hovering the
suction cup at known heights and over printed markers with a ruler (±1–2 mm). Check them on your
own setup before relying on them.

- **Mounting surface = `base_link` z 0:** `base_link` z = 0 (the zero the URDF and the firmware
  share) is the underside of the base, so a robot bolted directly to a table has the table at z 0
  (joint 2 axis measured 127 mm above the table, as in the URDF). Model the table at `z = 0`
  (collision object a millimetre lower). Older versions of this README said -0.021: that came from a
  touch-off with a tool length 21 mm too long (see above).
- **Joint moves are straight lines in joint space:** `M21 G90 G00 X.. Y.. Z.. A.. B.. C..` (joint 1–6
  in degrees) moves all joints simultaneously; the measured path stayed within 0.0004 rad of the
  straight joint-space line. A Pilz `PTP` plan (also a straight joint-space line) is therefore
  reproduced as a single G-code move to its end point — same path, firmware timing.
- **Reach with the suction cup pointing down** (IK incl. self-collision, straight ahead): tip 10 mm
  above the plate 0.12–0.27 m from the base axis; 50 mm above the plate 0.14–0.29 m and more.
- **IK branch:** tool-down solutions normally have joint 5 < 0 (wrist bent down). Near home
  height KDL can return a solution with joint 5 slightly > 0; seed with joint 5 negative if that
  matters.
- **Feedback rate:** firmware telemetry (`O104=1`) arrives at roughly 5 Hz, so joint states are
  not suited for closed-loop control.

### Executing a plan

There is no `FollowJointTrajectory` controller. To execute a plan, check that the robot is at the
trajectory's first point and send the joint targets through the driver's raw G-code topic, waiting
until `/mirobot_01/joint_states` reaches each target:

```bash
# one joint-space move (degrees, F = feed); a PTP plan needs only its last point
ros2 topic pub --once /mirobot_01/cmd_function std_msgs/msg/String \
  "{data: 'M21 G90 G00 X20 Y10 Z5 A0 B-15 C0 F600\r\n'}"
```

For curved plans (OMPL), send intermediate points so that each straight joint-space segment stays
close to the planned path. Keep the feed low and the workspace clear: nothing interrupts a G-code
move once the firmware has accepted it.

## Integration with Other ROS 2 Projects

The headless launch file makes it easy to integrate Mirobot into larger ROS 2 systems.

### Example Launch File Integration

```python
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node

def generate_launch_description():
    # Include Mirobot headless launch
    mirobot_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare('mirobot_interfaces'),
            '/launch/mirobot_headless.launch.py'
        ]),
        launch_arguments={
            'config_file': '/path/to/your/custom_config.yaml'
        }.items()
    )
    
    # Your custom node
    your_node = Node(
        package='your_package',
        executable='your_node',
        name='your_node',
        output='screen'
    )
    
    return LaunchDescription([
        mirobot_launch,
        your_node,
    ])
```

### Example Python Node Integration

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from mirobot_msgs.msg import CartesianPose, EndeffectorState

class MirobotController(Node):
    def __init__(self):
        super().__init__('mirobot_controller')
        
        # Publishers for commanding the robot
        self.joint_pub = self.create_publisher(
            JointState, 'cmd_joint_states', 10)
        self.cartesian_pub = self.create_publisher(
            CartesianPose, '/cmd_cartesian_pose', 10)
        self.gripper_pub = self.create_publisher(
            EndeffectorState, 'cmd_endeffector_state', 10)
        
        # Subscriber for robot state
        self.create_subscription(
            JointState, 'joint_states', self.joint_callback, 10)
    
    def joint_callback(self, msg):
        # Process current joint states
        self.get_logger().info(f'Current joints: {msg.position}')
    
    def move_to_position(self, x, y, z):
        msg = CartesianPose()
        msg.x, msg.y, msg.z = x, y, z
        msg.roll = msg.pitch = msg.yaw = 0.0
        msg.mode = 0  # PTP
        msg.speed = 2000.0
        self.cartesian_pub.publish(msg)

def main():
    rclpy.init()
    node = MirobotController()
    rclpy.spin(node)
    rclpy.shutdown()
```

## Dependencies

This project depends on the following ROS 2 packages:

### Core Dependencies
- `rclcpp` - ROS 2 C++ client library
- `rclpy` - ROS 2 Python client library
- `sensor_msgs` - Standard sensor message types
- `std_msgs` - Standard message types
- `serial_driver` - Serial port communication (ros-jazzy-serial-driver)
- `rosidl_default_generators` - Message generation tools

### GUI Dependencies
- `python3-pyqt5` - Qt5 GUI framework for Python
- `ros-jazzy-rviz2` - 3D visualization tool

### Custom Packages
- `mirobot_msgs` - Custom message definitions (EndeffectorState, CartesianPose, StdFunction)
- `mirobot_description` - URDF and visualization configuration

## Custom Messages

### EndeffectorState
Controls the gripper state:
```
uint8 state  # 0=close(500), 1=open(1000), 2=idle(0)
```

### CartesianPose
Cartesian position and orientation control:
```
float64 x, y, z              # Position in millimeters
float64 roll, pitch, yaw     # Orientation in degrees
uint8 mode                   # 0=PTP (G0), 1=Linear (G1)
float64 speed                # Feed rate in mm/s
```

### StdFunction
Standard function commands (internal use):
```
string command  # "homing", "centering", "randomizing"
```

## Architecture

### Module Descriptions

#### `mirobot_gcode_writer` (C++)
The core serial communication node that translates ROS messages into Mirobot G-code commands:
- Subscribes to joint states, Cartesian poses, and end effector commands
- Generates appropriate G-code (M21 for joints, M20 for Cartesian, M3 for gripper)
- Manages serial port communication and robot responses
- Implements duplicate command prevention for efficiency
- Configurable homing behavior and timeout

**Subscribed Topics:**
- `joint_states` - Joint angle commands (radians)
- `/cmd_cartesian_pose` - Cartesian position commands
- `endeffector_state` - Gripper control
- `cmd_function` - Direct G-code override

**Published Topics:**
- `mirobot_response` - Robot feedback messages

#### `mirobot_controller` (Python)
Bridges user commands and continuous state publishing:
- Receives commands from GUI or external nodes
- Maintains and publishes latest joint state at configurable rate (default: 2 Hz)
- Republishes end effector states
- Converts angles and formats data for G-code writer

**Subscribed Topics:**
- `cmd_joint_states` - Joint commands from GUI/external
- `cmd_endeffector_state` - Gripper commands from GUI/external

**Published Topics:**
- `joint_states` - Continuous joint state updates (2 Hz)
- `endeffector_state` - Gripper state updates

#### `mirobot_std_functions` (Python)
Provides pre-programmed standard movements and utilities:
- **Homing**: Sends `$H` command for auto-calibration
- **Centering**: Moves all joints to neutral position (0 radians)
- **Randomizing**: Generates random valid joint configurations for testing

**Subscribed Topics:**
- `cmd_std_functions` - Function command strings

**Published Topics:**
- `res_std_functions` - Result joint states
- `cmd_function` - G-code commands to writer

#### `mirobot_gui` (Python/PyQt5)
Interactive graphical interface for manual robot control:
- 6 joint angle sliders with real-time position display
- Gripper control buttons (Open/Close/Idle)
- Standard function buttons (Home/Center/Randomize)
- Receives feedback from controller for UI updates

**Published Topics:**
- `cmd_joint_states` - User slider input
- `cmd_endeffector_state` - User button input
- `cmd_std_functions` - Standard function requests

**Subscribed Topics:**
- `res_std_functions` - Updates sliders after standard functions

### System Architecture Diagram

Below is a graphical representation of how the modules interact:

```
┌─────────────────┐     ┌──────────────────┐     ┌────────────────────┐
│                 │────▶│                  │────▶│                    │
│  mirobot_gui    │     │ mirobot_         │     │  mirobot_gcode_    │
│  (PyQt5)        │     │ controller       │     │  writer (C++)      │
│                 │◀────│                  │     │                    │
└─────────────────┘     └──────────────────┘     └──────────┬─────────┘
        │                       │                            │
        │                       │                            │
        ▼                       ▼                            ▼
┌─────────────────┐     ┌──────────────────┐     ┌────────────────────┐
│ cmd_joint_      │     │  joint_states    │     │  Serial Port       │
│ states          │     │  (2 Hz)          │     │  /dev/ttyUSB1      │
└─────────────────┘     └──────────────────┘     └────────────────────┘
                                                           │
┌─────────────────┐                                       │
│ mirobot_std_    │                                       ▼
│ functions       │                              ┌─────────────────────┐
└────────┬────────┘                              │   Wlkata Mirobot    │
         │                                       │   Hardware          │
         ▼                                       └─────────────────────┘
┌─────────────────┐
│ cmd_function    │
└─────────────────┘
```

**Data Flow:**
1. **User Input** → GUI publishes to `cmd_joint_states`
2. **Controller** → Republishes to `joint_states` at 2 Hz
3. **G-code Writer** → Converts to G-code and sends via serial
4. **Robot** → Executes motion and returns status
5. **Response** → Published to `mirobot_response` topic

### Topic Reference

| Topic Name | Message Type | Direction | Description |
|------------|-------------|-----------|-------------|
| `joint_states` | `sensor_msgs/JointState` | Pub | Current/commanded joint positions |
| `cmd_joint_states` | `sensor_msgs/JointState` | Sub | Joint angle commands |
| `/cmd_cartesian_pose` | `mirobot_msgs/CartesianPose` | Sub | Cartesian position commands |
| `endeffector_state` | `mirobot_msgs/EndeffectorState` | Pub/Sub | Gripper state |
| `cmd_endeffector_state` | `mirobot_msgs/EndeffectorState` | Sub | Gripper commands |
| `cmd_function` | `std_msgs/String` | Sub | Direct G-code override |
| `cmd_std_functions` | `std_msgs/String` | Sub | Standard function requests |
| `res_std_functions` | `sensor_msgs/JointState` | Pub | Standard function results |
| `mirobot_response` | `std_msgs/String` | Pub | Robot status messages |

## Troubleshooting

### Serial Port Issues

**Problem:** Robot not connecting or "Failed to open serial port" error

**Solutions:**
1. Check which port your Mirobot is using:
   ```bash
   ls -la /dev/ttyUSB*
   ```

2. Update the config file:
   ```bash
   nano src/mirobot_interfaces/config/mirobot_config.yaml
   # Change serial.port to /dev/ttyUSB0 or /dev/ttyUSB1
   ```

3. Ensure you have permissions:
   ```bash
   sudo usermod -a -G dialout $USER
   # Log out and log back in for changes to take effect
   ```

### GUI Not Appearing

**Problem:** Launch succeeds but no GUI window appears

**Solution:** This is typically caused by GTK module conflicts in VS Code snap. The launch file includes environment variable clearing that should fix this. If issues persist:
```bash
# Run outside VS Code terminal
unset GTK_MODULES && unset GTK3_MODULES && unset GTK_PATH
ros2 launch mirobot_interfaces mirobot_interface.launch.py
```

### Robot Not Responding to Commands

**Problem:** Commands published but robot doesn't move

**Possible Causes:**
1. **Serial port mismatch** - Check configuration
2. **Homing not completed** - Wait 20 seconds after launch
3. **Robot in error state** - Power cycle the Mirobot
4. **Wrong topic names** - Verify topic names match configuration

**Debug Commands:**
```bash
# Check if nodes are running
ros2 node list

# Monitor joint states
ros2 topic echo /joint_states

# Check for errors
ros2 topic echo /mirobot_response

# Manually send homing command
ros2 topic pub --once /cmd_std_functions std_msgs/msg/String "{data: 'homing'}"
```

### Build Errors

**Problem:** `colcon build` fails

**Common Solutions:**
```bash
# Clean and rebuild
rm -rf build/ install/ log/
colcon build

# Check for missing dependencies
rosdep install --from-paths src --ignore-src -r -y

# Ensure serial_driver is installed
sudo apt install ros-jazzy-serial-driver
```

### Rate/Performance Issues

**Problem:** Robot movement is jerky or delayed

**Solution:** Adjust the controller publish rate in config:
```yaml
controller:
  publish_rate_hz: 2.0  # Lower values = less frequent updates
```
**Note:** Values above 5 Hz may flood the serial communication.

## Roadmap / Future Enhancements

### Completed ✅
- ✅ Joint angle control
- ✅ Cartesian coordinate control with onboard IK
- ✅ End effector (gripper) control
- ✅ Graphical user interface
- ✅ RViz visualization
- ✅ Standard function library
- ✅ Headless operation mode
- ✅ YAML-based configuration
- ✅ Flexible topic naming
- ✅ MoveIt 2 motion planning (`mirobot_moveit_config`, plan only)

### Planned 🚧

1. **MoveIt 2 Execution**
   - `FollowJointTrajectory` action server streaming G-code (or a `ros2_control` hardware interface)
   - Interrupting a running move (firmware feed-hold behaviour still to be verified)

2. **Advanced Control Features**
   - Trajectory playback from files
   - Recording and replay functionality
   - Speed and acceleration profiles

3. **Enhanced Visualization**
   - Real-time trajectory preview in RViz
   - Workspace boundaries display
   - Collision geometry visualization

4. **Additional Interfaces**
   - REST API for web-based control
   - ROS 2 Actions for long-running commands
   - Service interfaces for synchronous operations

5. **Documentation Improvements**
   - Video tutorials
   - Example applications

Contributions for any of these features are welcome!

## License

This project is licensed under the Apache License 2.0. See the [LICENSE](LICENSE) file for details.

## Contributing

Contributions are welcome! Whether you're fixing bugs, adding features, or improving documentation, your help is appreciated.

### How to Contribute

1. **Fork the repository**
2. **Create a feature branch:**
   ```bash
   git checkout -b feature/your-feature-name
   ```
3. **Make your changes** and test thoroughly
4. **Commit with clear messages:**
   ```bash
   git commit -m "Add: description of your changes"
   ```
5. **Push to your fork:**
   ```bash
   git push origin feature/your-feature-name
   ```
6. **Submit a Pull Request** with a detailed description

### Contribution Guidelines
- Follow ROS 2 coding standards
- Add comments and documentation for new features
- Test your changes with actual hardware if possible
- Update relevant documentation (README, CONFIGURATION.md, etc.)
- Keep commits focused and atomic

## Support

- **Issues:** Report bugs or request features via [GitHub Issues](https://github.com/Eagle-Creative/ROS2-Jazzy-Mirobot-Interface/issues)
- **Discussions:** Ask questions in [GitHub Discussions](https://github.com/Eagle-Creative/ROS2-Jazzy-Mirobot-Interface/discussions)
- **Documentation:** See [`CONFIGURATION.md`](src/mirobot_interfaces/CONFIGURATION.md) for detailed configuration guide

## Acknowledgments

This project is inspired by the Wlkata Mirobot robotic arm and aims to provide a comprehensive, production-ready ROS 2 interface.

**Based on work from:**
- [kimsooyoung/mirobot_ros2](https://github.com/kimsooyoung/mirobot_ros2) - Original ROS 2 Humble implementation

**Key Improvements:**
- ✨ Upgraded to ROS 2 Jazzy with `serial_driver` package
- ✨ Added Cartesian coordinate control with onboard IK
- ✨ Implemented flexible YAML-based configuration system
- ✨ Created headless launch file for project integration
- ✨ Enhanced GUI with real-time feedback
- ✨ Optimized controller rate for reliable performance
- ✨ Added comprehensive documentation

**Special Thanks:**
- ROS 2 community for the excellent tools and libraries
- Wlkata for creating an accessible robotic platform
- Contributors and users providing feedback and improvements

---

**Maintained by:** [Eagle-Creative](https://github.com/Eagle-Creative)  
**Repository:** [ROS2-Jazzy-Mirobot-Interface](https://github.com/Eagle-Creative/ROS2-Jazzy-Mirobot-Interface)  
**ROS 2 Distribution:** Jazzy Jalisco  
**Last Updated:** January 2026
