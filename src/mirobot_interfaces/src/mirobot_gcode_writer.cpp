#include <memory>
#include <cmath>
#include "serial_driver/serial_driver.hpp"
#include "serial_driver/serial_port.hpp"
#include "io_context/io_context.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/bool.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "mirobot_msgs/msg/endeffector_state.hpp"

class MirobotWriteNode : public rclcpp::Node {
public:
    MirobotWriteNode() : Node("mirobot_gcode_writer"),
                         io_context_(),
                         serial_config_(115200, drivers::serial_driver::FlowControl::NONE,
                                       drivers::serial_driver::Parity::NONE,
                                       drivers::serial_driver::StopBits::ONE) {
        // Declare parameters with defaults matching config file structure
        this->declare_parameter("topics.prefix", "");
        this->declare_parameter("topics.joint_states", "joint_states");
        this->declare_parameter("topics.endeffector_state", "endeffector_state");
        this->declare_parameter("topics.cmd_cartesian_pose", "target_pose");
        this->declare_parameter("topics.cartesian_linear_mode", "cartesian_linear_mode");
        this->declare_parameter("topics.cartesian_speed", "cartesian_speed");
        this->declare_parameter("topics.cmd_function", "cmd_function");
        this->declare_parameter("topics.mirobot_response", "mirobot_response");
        this->declare_parameter("serial.port", "/dev/ttyUSB1");
        this->declare_parameter("serial.baud_rate", 115200);
        this->declare_parameter("robot.perform_homing_on_startup", true);
        this->declare_parameter("robot.homing_timeout_seconds", 20);
        this->declare_parameter("robot.default_cartesian_speed", 2000.0);
        
        // Get topic prefix and apply it to all topics
        std::string prefix = this->get_parameter("topics.prefix").as_string();
        if (!prefix.empty() && prefix.back() != '/') {
            prefix += "/";  // Ensure prefix ends with /
        }
        
        joint_states_topic_name = prefix + this->get_parameter("topics.joint_states").as_string();
        std::string endeffector_topic = prefix + this->get_parameter("topics.endeffector_state").as_string();
        std::string cartesian_topic = prefix + this->get_parameter("topics.cmd_cartesian_pose").as_string();
        std::string linear_mode_topic = prefix + this->get_parameter("topics.cartesian_linear_mode").as_string();
        std::string speed_topic = prefix + this->get_parameter("topics.cartesian_speed").as_string();
        std::string cmd_function_topic = prefix + this->get_parameter("topics.cmd_function").as_string();
        std::string response_topic = prefix + this->get_parameter("topics.mirobot_response").as_string();
        std::string serial_port = this->get_parameter("serial.port").as_string();
        int baud_rate = this->get_parameter("serial.baud_rate").as_int();
        bool perform_homing = this->get_parameter("robot.perform_homing_on_startup").as_bool();
        homing_timeout_seconds_ = this->get_parameter("robot.homing_timeout_seconds").as_int();
        cartesian_speed_ = this->get_parameter("robot.default_cartesian_speed").as_double();
        cartesian_linear_mode_ = false; // Default to PTP mode
        
        RCLCPP_INFO(this->get_logger(), "Configuration loaded:");
        RCLCPP_INFO(this->get_logger(), "  Serial Port: %s", serial_port.c_str());
        RCLCPP_INFO(this->get_logger(), "  Baud Rate: %d", baud_rate);
        RCLCPP_INFO(this->get_logger(), "  Joint States Topic: %s", joint_states_topic_name.c_str());
        RCLCPP_INFO(this->get_logger(), "  Perform Homing on Startup: %s", perform_homing ? "true" : "false");

        // Initialize the serial port with the configured port from config file
        serial_ = std::make_unique<drivers::serial_driver::SerialPort>(io_context_, serial_port, serial_config_);

        js_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
            joint_states_topic_name, 10, std::bind(&MirobotWriteNode::joint_state_callback, this, std::placeholders::_1));

        ee_state_sub_ = this->create_subscription<mirobot_msgs::msg::EndeffectorState>(
            endeffector_topic, 10, std::bind(&MirobotWriteNode::endeffector_state_callback, this, std::placeholders::_1));

        cartesian_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            cartesian_topic, 10, std::bind(&MirobotWriteNode::cartesian_pose_callback, this, std::placeholders::_1));

        linear_mode_sub_ = this->create_subscription<std_msgs::msg::Bool>(
            linear_mode_topic, 10, std::bind(&MirobotWriteNode::linear_mode_callback, this, std::placeholders::_1));

        speed_sub_ = this->create_subscription<std_msgs::msg::Float64>(
            speed_topic, 10, std::bind(&MirobotWriteNode::speed_callback, this, std::placeholders::_1));

        cmd_function_sub_ = this->create_subscription<std_msgs::msg::String>(
            cmd_function_topic, 10, std::bind(&MirobotWriteNode::handle_cmd_function, this, std::placeholders::_1));

        // Initialize the mirobot_response publisher
        mirobot_response_publisher_ = this->create_publisher<std_msgs::msg::String>(response_topic, 10);

        RCLCPP_INFO(this->get_logger(), "Subscribed to topics: %s, %s, %s", 
                    endeffector_topic.c_str(), cartesian_topic.c_str(), cmd_function_topic.c_str());

        // Check if the serial port is open
        try {
            if (!serial_->is_open()) {
                RCLCPP_ERROR(this->get_logger(), "Failed to open serial port %s", serial_port.c_str());
            } else {
                RCLCPP_INFO(this->get_logger(), "Serial port %s opened successfully.", serial_port.c_str());
            }
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Exception during serial port initialization: %s", e.what());
        }

        // Perform homing during startup if enabled
        if (perform_homing) {
            homing();
        } else {
            RCLCPP_INFO(this->get_logger(), "Homing on startup is disabled.");
        }
    }

    void homing() {
        std::vector<uint8_t> HomingGcode = {'$', 'H', '\r', '\n'};
        ensure_serial_port_open();

        // Check if the serial port is open
        if (!serial_->is_open()) {
            RCLCPP_ERROR(this->get_logger(), "Serial port is not open. Cannot send homing command.");
            return;
        }

        // Send the G-code command to the serial port
        try {
            serial_->send(HomingGcode);
            RCLCPP_INFO(this->get_logger(), "Homing command sent successfully.");
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Error sending homing command: %s", e.what());
            return;
        }

        // Receive the response from the serial port
        try {
            std::vector<uint8_t> buffer(1024);
            size_t bytes_read = serial_->receive(buffer);

            if (bytes_read > 0) {
                std::string result(buffer.begin(), buffer.begin() + bytes_read);
                RCLCPP_INFO(this->get_logger(), "Received: %s", result.c_str());
            } else {
                RCLCPP_WARN(this->get_logger(), "No data received within the timeout period.");
            }
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Error receiving data: %s", e.what());
        }

        // Wait for configured time to complete the homing process
        RCLCPP_INFO(this->get_logger(), "Waiting for %d seconds to complete homing...", homing_timeout_seconds_);
        rclcpp::sleep_for(std::chrono::seconds(homing_timeout_seconds_));
        RCLCPP_INFO(this->get_logger(), "Homing completed. The robot is ready for usage.");
    }

    void handle_cmd_function(const std_msgs::msg::String::SharedPtr msg) {
        std::string gcode = msg->data;
        ensure_serial_port_open(); // Ensure the serial port is open before proceeding

        if (!serial_->is_open()) {
            RCLCPP_ERROR(this->get_logger(), "Serial port is not open. Cannot send GCODE override command.");
            return;
        }

        // Check if the command is a homing command
        if (gcode == "$H\r\n") {
            RCLCPP_INFO(this->get_logger(), "Homing command detected. Delegating to the homing() function.");
            homing(); // Delegate to the homing() function
            return;
        }

        // Handle other GCODE commands
        try {
            std::vector<uint8_t> Gcode(gcode.begin(), gcode.end());
            serial_->send(Gcode);
            RCLCPP_INFO(this->get_logger(), "GCODE override command received and sent: %s", gcode.c_str());
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Error sending GCODE override command: %s", e.what());
        }

        // Receive the response from the Mirobot
        try {
            std::vector<uint8_t> buffer(1024);
            size_t bytes_read = serial_->receive(buffer);

            if (bytes_read > 0) {
                std::string response(buffer.begin(), buffer.begin() + bytes_read);
                RCLCPP_INFO(this->get_logger(), "Received response from Mirobot: %s", response.c_str());

                // Publish the response to the mirobot_response topic
                auto response_msg = std_msgs::msg::String();
                response_msg.data = response;
                mirobot_response_publisher_->publish(response_msg);
            } else {
                RCLCPP_WARN(this->get_logger(), "No data received within the timeout period.");
            }
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Error receiving response from Mirobot: %s", e.what());
        }
    }

private:
    void ensure_serial_port_open() {
        if (!serial_->is_open()) {
            try {
                serial_->open();
                RCLCPP_INFO(this->get_logger(), "Serial port reopened successfully.");
            } catch (const std::exception& e) {
                RCLCPP_ERROR(this->get_logger(), "Failed to reopen serial port: %s", e.what());
            }
        }
    }

    void joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg) {
        ensure_serial_port_open(); // Ensure the serial port is open before proceeding

        if (!serial_->is_open()) {
            RCLCPP_ERROR(this->get_logger(), "Serial port is not open. Cannot send data.");
            return;
        }

        char angle0[10], angle1[10], angle2[10], angle3[10], angle4[10], angle5[10];

        sprintf(angle0, "%.2f", msg->position[0] * 57.296);
        sprintf(angle1, "%.2f", msg->position[1] * 57.296);
        sprintf(angle2, "%.2f", msg->position[2] * 57.296);
        sprintf(angle3, "%.2f", msg->position[3] * 57.296);
        sprintf(angle4, "%.2f", msg->position[4] * 57.296);
        sprintf(angle5, "%.2f", msg->position[5] * 57.296);

        std::string GcodeString = (std::string)"M21 G0 X" + angle0 + " Y" + angle1 + " Z" + angle2 + " A" + angle3 + " B" + angle4 + " C" + angle5 + " F3000" + "\r\n";
        
        // Check if the new G-code is the same as the last sent one
        if (GcodeString == last_sent_gcode_) {
            return;
        }

        RCLCPP_INFO(this->get_logger(), "Generated G-code: %s", GcodeString.c_str());

        std::vector<uint8_t> Gcode(GcodeString.begin(), GcodeString.end());

        try {
            
            serial_->send(Gcode);
            RCLCPP_INFO(this->get_logger(), "Sent G-code: %s", GcodeString.c_str());
            last_sent_gcode_ = GcodeString; // Update the last sent G-code
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Error sending data: %s", e.what());
            return;
        }

        try {            
            std::vector<uint8_t> buffer(2056);
            size_t bytes_read = serial_->receive(buffer);

            if (bytes_read > 0) {
                std::string result(buffer.begin(), buffer.begin() + bytes_read);
                RCLCPP_INFO(this->get_logger(), "Received: %s", result.c_str());
            } else {
                RCLCPP_WARN(this->get_logger(), "No data received within the timeout period.");
            }
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Error receiving data: %s", e.what());
        }
    }

    void endeffector_state_callback(const mirobot_msgs::msg::EndeffectorState::SharedPtr msg) {
        RCLCPP_INFO(this->get_logger(), "Received end effector state: %d", msg->state);

        ensure_serial_port_open(); // Ensure the serial port is open before proceeding

        if (!serial_->is_open()) {
            RCLCPP_ERROR(this->get_logger(), "Serial port is not open. Cannot send end effector state.");
            return;
        }

        // Map the state to corresponding G-code commands
        std::string GcodeString;
        if (msg->state == 0) {
            GcodeString = "M3S500\r\n"; // Example G-code for state 0 (e.g., turn off end effector)
        } else if (msg->state == 1) {
            GcodeString = "M3S1000\r\n"; // Example G-code for state 1 (e.g., activate end effector)
        } else if (msg->state == 2) {
            GcodeString = "M3S0\r\n"; // Example G-code for state 2 (e.g., blow air or another action)
        } else {
            RCLCPP_WARN(this->get_logger(), "Unknown end effector state: %d", msg->state);
            return;
        }

        // Send the G-code command to the serial port
        try {
            std::vector<uint8_t> Gcode(GcodeString.begin(), GcodeString.end());
            serial_->send(Gcode);
            RCLCPP_INFO(this->get_logger(), "Sent G-code for end effector state: %s", GcodeString.c_str());
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Error sending end effector G-code: %s", e.what());
        }
    }

    void linear_mode_callback(const std_msgs::msg::Bool::SharedPtr msg) {
        cartesian_linear_mode_ = msg->data;
        RCLCPP_INFO(this->get_logger(), "Cartesian mode set to: %s", 
                    cartesian_linear_mode_ ? "Linear (G1)" : "PTP (G0)");
    }

    void speed_callback(const std_msgs::msg::Float64::SharedPtr msg) {
        cartesian_speed_ = msg->data;
        RCLCPP_INFO(this->get_logger(), "Cartesian speed set to: %.0f mm/s", cartesian_speed_);
    }

    void cartesian_pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        ensure_serial_port_open(); // Ensure the serial port is open before proceeding

        if (!serial_->is_open()) {
            RCLCPP_ERROR(this->get_logger(), "Serial port is not open. Cannot send Cartesian pose command.");
            return;
        }

        // Extract position (in meters, convert to mm)
        double x_mm = msg->pose.position.x * 1000.0;
        double y_mm = msg->pose.position.y * 1000.0;
        double z_mm = msg->pose.position.z * 1000.0;

        // Convert quaternion to Roll-Pitch-Yaw (in degrees)
        double roll, pitch, yaw;
        quaternion_to_rpy(msg->pose.orientation, roll, pitch, yaw);

        RCLCPP_INFO(this->get_logger(), "Received Cartesian pose: X=%.2f Y=%.2f Z=%.2f Roll=%.2f Pitch=%.2f Yaw=%.2f",
                    x_mm, y_mm, z_mm, roll, pitch, yaw);

        // Format the position and orientation values
        char x_str[10], y_str[10], z_str[10], roll_str[10], pitch_str[10], yaw_str[10], speed_str[10];
        sprintf(x_str, "%.2f", x_mm);
        sprintf(y_str, "%.2f", y_mm);
        sprintf(z_str, "%.2f", z_mm);
        sprintf(roll_str, "%.2f", roll);
        sprintf(pitch_str, "%.2f", pitch);
        sprintf(yaw_str, "%.2f", yaw);
        sprintf(speed_str, "%.0f", cartesian_speed_);

        // Determine movement type based on linear mode flag
        std::string move_type = cartesian_linear_mode_ ? "G1" : "G0";
        std::string mode_name = cartesian_linear_mode_ ? "Linear" : "PTP";

        // Build the G-code command: M20 G90 G0/G1 X Y Z A B C F<speed>
        std::string GcodeString = (std::string)"M20 G90 " + move_type + " X" + x_str + " Y" + y_str + " Z" + z_str + 
                                  " A" + roll_str + " B" + pitch_str + " C" + yaw_str + " F" + speed_str + "\r\n";

        // Check if the new G-code is the same as the last sent one
        if (GcodeString == last_sent_cartesian_gcode_) {
            return;
        }

        RCLCPP_INFO(this->get_logger(), "Generated Cartesian G-code (%s): %s", mode_name.c_str(), GcodeString.c_str());

        std::vector<uint8_t> Gcode(GcodeString.begin(), GcodeString.end());

        try {
            serial_->send(Gcode);
            RCLCPP_INFO(this->get_logger(), "Sent Cartesian G-code: %s", GcodeString.c_str());
            last_sent_cartesian_gcode_ = GcodeString; // Update the last sent Cartesian G-code
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Error sending Cartesian G-code: %s", e.what());
            return;
        }

        try {
            std::vector<uint8_t> buffer(2056);
            size_t bytes_read = serial_->receive(buffer);

            if (bytes_read > 0) {
                std::string result(buffer.begin(), buffer.begin() + bytes_read);
                RCLCPP_INFO(this->get_logger(), "Received: %s", result.c_str());
            } else {
                RCLCPP_WARN(this->get_logger(), "No data received within the timeout period.");
            }
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Error receiving data: %s", e.what());
        }
    }

    // Helper function to convert quaternion to Roll-Pitch-Yaw (in degrees)
    void quaternion_to_rpy(const geometry_msgs::msg::Quaternion& q, double& roll, double& pitch, double& yaw) {
        // Roll (x-axis rotation)
        double sinr_cosp = 2.0 * (q.w * q.x + q.y * q.z);
        double cosr_cosp = 1.0 - 2.0 * (q.x * q.x + q.y * q.y);
        roll = std::atan2(sinr_cosp, cosr_cosp) * 180.0 / M_PI;

        // Pitch (y-axis rotation)
        double sinp = 2.0 * (q.w * q.y - q.z * q.x);
        if (std::abs(sinp) >= 1)
            pitch = std::copysign(M_PI / 2, sinp) * 180.0 / M_PI; // use 90 degrees if out of range
        else
            pitch = std::asin(sinp) * 180.0 / M_PI;

        // Yaw (z-axis rotation)
        double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
        double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
        yaw = std::atan2(siny_cosp, cosy_cosp) * 180.0 / M_PI;
    }

    std::string last_sent_gcode_; // Store the last sent joint G-code
    std::string last_sent_cartesian_gcode_; // Store the last sent Cartesian G-code
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr js_sub_;
    rclcpp::Subscription<mirobot_msgs::msg::EndeffectorState>::SharedPtr ee_state_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr cartesian_pose_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr linear_mode_sub_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr speed_sub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr cmd_function_sub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr mirobot_response_publisher_;
    std::string joint_states_topic_name;
    int homing_timeout_seconds_;
    
    // Cartesian control parameters
    double cartesian_speed_;
    bool cartesian_linear_mode_;
    
    // Serial port members
    drivers::common::IoContext io_context_;
    drivers::serial_driver::SerialPortConfig serial_config_;
    std::unique_ptr<drivers::serial_driver::SerialPort> serial_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);

    auto node = std::make_shared<MirobotWriteNode>();

    RCLCPP_INFO(node->get_logger(), "Mirobot G-code Writer Node is ready.");
    
    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}
