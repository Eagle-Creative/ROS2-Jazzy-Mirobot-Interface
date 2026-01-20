# Mirobot ROS 2 Interface - Configuration Guide

## Launch Files

### 1. Full Interface with GUI (mirobot_interface.launch.py)
Launches the complete Mirobot system including GUI, RViz, and all nodes:
```bash
ros2 launch mirobot_interfaces mirobot_interface.launch.py
```

### 2. Headless Mode (mirobot_headless.launch.py)
Launches Mirobot without GUI or RViz - ideal for integration with other ROS projects:
```bash
ros2 launch mirobot_interfaces mirobot_headless.launch.py
```

Optional arguments:
- `config_file`: Path to custom configuration file (default: uses mirobot_config.yaml)
- `enable_rviz`: Set to 'true' to enable RViz in headless mode (default: false)

Example with custom config:
```bash
ros2 launch mirobot_interfaces mirobot_headless.launch.py config_file:=/path/to/my_config.yaml
```

## Configuration File

All Mirobot parameters are configured in `config/mirobot_config.yaml`:

### Serial Communication
```yaml
serial:
  port: "/dev/ttyUSB1"        # USB port for Mirobot
  baud_rate: 115200           # Serial baud rate
```

### Controller Settings
```yaml
controller:
  publish_rate_hz: 2.0        # Rate for continuous joint state publishing
```

### Topic Names
All ROS topic names are configurable:
```yaml
topics:
  joint_states: "joint_states"
  cmd_joint_states: "cmd_joint_states"
  cmd_endeffector_state: "cmd_endeffector_state"
  endeffector_state: "endeffector_state"
  cmd_cartesian_pose: "/cmd_cartesian_pose"
  cmd_function: "cmd_function"
  cmd_std_functions: "cmd_std_functions"
  res_std_functions: "res_std_functions"
  mirobot_response: "mirobot_response"
```

### Robot Behavior
```yaml
robot:
  perform_homing_on_startup: true    # Auto-home robot on launch
  homing_timeout_seconds: 20         # How long to wait for homing
```

## Integration with Other ROS Projects

To integrate Mirobot into your ROS 2 project:

1. **Include the headless launch file** in your launch file:
```python
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    mirobot_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare('mirobot_interfaces'), 
            '/launch/mirobot_headless.launch.py'
        ]),
        launch_arguments={
            'config_file': '/path/to/your/custom_config.yaml'
        }.items()
    )
    
    return LaunchDescription([
        mirobot_launch,
        # Your other nodes...
    ])
```

2. **Subscribe to Mirobot topics** in your nodes:
```python
# Joint states
self.create_subscription(JointState, 'joint_states', callback, 10)

# Mirobot responses
self.create_subscription(String, 'mirobot_response', callback, 10)
```

3. **Publish commands to Mirobot**:
```python
from sensor_msgs.msg import JointState
from mirobot_msgs.msg import CartesianPose, EndeffectorState

# Joint angle control
joint_pub = self.create_publisher(JointState, 'cmd_joint_states', 10)

# Cartesian control
cartesian_pub = self.create_publisher(CartesianPose, '/cmd_cartesian_pose', 10)

# Gripper control
gripper_pub = self.create_publisher(EndeffectorState, 'cmd_endeffector_state', 10)
```

## Control Modes

### Joint Angle Control
Publish to `cmd_joint_states` with 6 joint positions (radians):
```python
msg = JointState()
msg.name = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"]
msg.position = [0.0, 0.5, -0.5, 0.0, 0.0, 0.0]  # radians
joint_pub.publish(msg)
```

### Cartesian Position Control
Publish to `/cmd_cartesian_pose`:
```python
from mirobot_msgs.msg import CartesianPose

msg = CartesianPose()
msg.x = 150.0        # mm
msg.y = 0.0          # mm  
msg.z = 150.0        # mm
msg.roll = 0.0       # degrees
msg.pitch = 0.0      # degrees
msg.yaw = 0.0        # degrees
msg.mode = 0         # 0=PTP, 1=Linear
msg.speed = 2000.0   # mm/s
cartesian_pub.publish(msg)
```

### Gripper Control
Publish to `cmd_endeffector_state`:
```python
from mirobot_msgs.msg import EndeffectorState

msg = EndeffectorState()
msg.state = 1  # 0=close(500), 1=open(1000), 2=idle(0)
gripper_pub.publish(msg)
```

## Troubleshooting

### Serial Port Issues
If the robot doesn't connect, check which USB port it's on:
```bash
ls -la /dev/ttyUSB*
```
Update the `serial.port` value in `mirobot_config.yaml` accordingly.

### Custom Topic Namespaces
To use a namespace for all Mirobot topics, create a custom config file and prefix all topic names:
```yaml
topics:
  joint_states: "/mirobot/joint_states"
  cmd_joint_states: "/mirobot/cmd_joint_states"
  # ... etc
```
