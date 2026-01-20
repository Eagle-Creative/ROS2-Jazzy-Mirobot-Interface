# Using Standard ROS Messages with Mirobot

The Mirobot interface now uses **standard ROS 2 messages** for Cartesian control, making it easy to integrate with other ROS projects without custom message dependencies.

## Cartesian Pose Control

### Using `geometry_msgs/PoseStamped`

Instead of the custom `CartesianPose` message, you can now use the standard `geometry_msgs/PoseStamped`:

```bash
# Command line example - Move to a target pose
ros2 topic pub --once /target_pose geometry_msgs/msg/PoseStamped "{
  header: {stamp: {sec: 0, nanosec: 0}, frame_id: 'base_link'},
  pose: {
    position: {x: 0.150, y: 0.0, z: 0.150},
    orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}
  }
}"
```

**Note:** Position is in **meters** (will be converted to mm internally), orientation uses **quaternions** (will be converted to roll-pitch-yaw in degrees).

### Setting Movement Mode and Speed

Control the movement type and speed using standard messages:

```bash
# Set to Linear mode (true = G1 linear, false = G0 point-to-point)
ros2 topic pub --once /cartesian_linear_mode std_msgs/msg/Bool "{data: true}"

# Set speed (in mm/s)
ros2 topic pub --once /cartesian_speed std_msgs/msg/Float64 "{data: 1500.0}"
```

## Python Integration Example

```python
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool, Float64
from tf_transformations import quaternion_from_euler
import math

class MirobotController(Node):
    def __init__(self):
        super().__init__('mirobot_controller')
        
        # Publishers using standard ROS messages
        self.pose_pub = self.create_publisher(PoseStamped, '/target_pose', 10)
        self.mode_pub = self.create_publisher(Bool, '/cartesian_linear_mode', 10)
        self.speed_pub = self.create_publisher(Float64, '/cartesian_speed', 10)
    
    def move_to_pose(self, x, y, z, roll, pitch, yaw, speed=2000.0, linear=False):
        """
        Move robot to Cartesian pose
        
        Args:
            x, y, z: Position in meters
            roll, pitch, yaw: Orientation in degrees
            speed: Speed in mm/s
            linear: True for linear motion (G1), False for PTP (G0)
        """
        # Set speed
        speed_msg = Float64()
        speed_msg.data = speed
        self.speed_pub.publish(speed_msg)
        
        # Set motion mode
        mode_msg = Bool()
        mode_msg.data = linear
        self.mode_pub.publish(mode_msg)
        
        # Convert euler angles to quaternion
        roll_rad = math.radians(roll)
        pitch_rad = math.radians(pitch)
        yaw_rad = math.radians(yaw)
        q = quaternion_from_euler(roll_rad, pitch_rad, yaw_rad)
        
        # Create and publish pose
        pose_msg = PoseStamped()
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.header.frame_id = 'base_link'
        pose_msg.pose.position.x = x
        pose_msg.pose.position.y = y
        pose_msg.pose.position.z = z
        pose_msg.pose.orientation.x = q[0]
        pose_msg.pose.orientation.y = q[1]
        pose_msg.pose.orientation.z = q[2]
        pose_msg.pose.orientation.w = q[3]
        
        self.pose_pub.publish(pose_msg)
        self.get_logger().info(f'Moving to: x={x}, y={y}, z={z}')

def main():
    rclpy.init()
    controller = MirobotController()
    
    # Example: Move to position (150mm, 0mm, 150mm) with no rotation
    controller.move_to_pose(0.150, 0.0, 0.150, 0.0, 0.0, 0.0, speed=2000.0, linear=False)
    
    rclpy.spin_once(controller, timeout_sec=0.1)
    controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## C++ Integration Example

```cpp
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float64.hpp>
#include <tf2/LinearMath/Quaternion.h>

class MirobotController : public rclcpp::Node {
public:
    MirobotController() : Node("mirobot_controller") {
        pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/target_pose", 10);
        mode_pub_ = this->create_publisher<std_msgs::msg::Bool>("/cartesian_linear_mode", 10);
        speed_pub_ = this->create_publisher<std_msgs::msg::Float64>("/cartesian_speed", 10);
    }
    
    void moveToPose(double x, double y, double z, 
                    double roll, double pitch, double yaw,
                    double speed = 2000.0, bool linear = false) {
        // Set speed
        auto speed_msg = std_msgs::msg::Float64();
        speed_msg.data = speed;
        speed_pub_->publish(speed_msg);
        
        // Set mode
        auto mode_msg = std_msgs::msg::Bool();
        mode_msg.data = linear;
        mode_pub_->publish(mode_msg);
        
        // Create quaternion from euler angles (in radians)
        tf2::Quaternion q;
        q.setRPY(roll * M_PI / 180.0, pitch * M_PI / 180.0, yaw * M_PI / 180.0);
        
        // Create pose message
        auto pose_msg = geometry_msgs::msg::PoseStamped();
        pose_msg.header.stamp = this->now();
        pose_msg.header.frame_id = "base_link";
        pose_msg.pose.position.x = x;
        pose_msg.pose.position.y = y;
        pose_msg.pose.position.z = z;
        pose_msg.pose.orientation.x = q.x();
        pose_msg.pose.orientation.y = q.y();
        pose_msg.pose.orientation.z = q.z();
        pose_msg.pose.orientation.w = q.w();
        
        pose_pub_->publish(pose_msg);
        RCLCPP_INFO(this->get_logger(), "Moving to: x=%.3f, y=%.3f, z=%.3f", x, y, z);
    }

private:
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr mode_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr speed_pub_;
};
```

## Joint Control (Still Standard)

Joint control already uses the standard `sensor_msgs/JointState`:

```bash
ros2 topic pub --once /joint_states sensor_msgs/msg/JointState "{
  name: ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6'],
  position: [0.0, 0.5, -0.5, 0.0, 0.0, 0.0]
}"
```

**Note:** Joint positions are in **radians** (will be converted to degrees internally).

## Configuration

All topic names are configurable in `mirobot_config.yaml`:

```yaml
topics:
  cmd_cartesian_pose: "/target_pose"  # geometry_msgs/PoseStamped
  cartesian_linear_mode: "/cartesian_linear_mode"  # std_msgs/Bool
  cartesian_speed: "/cartesian_speed"  # std_msgs/Float64
  joint_states: "joint_states"  # sensor_msgs/JointState
```

## Benefits

✅ **No custom message dependencies** - Use standard ROS 2 messages
✅ **Easy integration** - Works with MoveIt, Nav2, and other ROS tools
✅ **Standard tools** - Use RQT, Foxglove, PlotJuggler without plugins
✅ **Cross-project compatibility** - Share code between different robot projects
✅ **Better documentation** - Standard messages are well-documented
