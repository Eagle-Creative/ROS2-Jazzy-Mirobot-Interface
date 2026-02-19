#include <memory>
#include <cmath>
#include <chrono>
#include <deque>
#include <cctype>
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
#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "diagnostic_msgs/msg/key_value.hpp"

class MirobotWriteNode : public rclcpp::Node {
public:
    MirobotWriteNode() : Node("mirobot_gcode_writer"),
                         io_context_(),
                         serial_config_(115200, drivers::serial_driver::FlowControl::NONE,
                                       drivers::serial_driver::Parity::NONE,
                                       drivers::serial_driver::StopBits::ONE) {
        // Declare parameters with defaults matching config file structure
        this->declare_parameter("robot.name", "mirobot_01");
        this->declare_parameter("topics.prefix", "");
        this->declare_parameter("topics.cmd_joint_states", "cmd_joint_states");
        this->declare_parameter("topics.cmd_endeffector_state", "cmd_endeffector_state");
        this->declare_parameter("topics.cmd_cartesian_pose", "target_pose");
        this->declare_parameter("topics.cartesian_mode", "cartesian_mode");
        this->declare_parameter("topics.cartesian_linear_mode", "cartesian_linear_mode");
        this->declare_parameter("topics.cartesian_speed", "cartesian_speed");
        this->declare_parameter("topics.cmd_function", "cmd_function");
        this->declare_parameter("topics.mirobot_response", "mirobot_response");
        this->declare_parameter("topics.mirobot_status", "status");
        this->declare_parameter("serial.port", "/dev/ttyUSB1");
        this->declare_parameter("serial.baud_rate", 115200);
        this->declare_parameter("robot.perform_homing_on_startup", true);
        this->declare_parameter("robot.homing_timeout_seconds", 20);
        this->declare_parameter("robot.default_cartesian_speed", 2000.0);
        this->declare_parameter("cartesian.default_mode", "linear");
        
        // Get robot name
        robot_name_ = this->get_parameter("robot.name").as_string();
        
        // Get topic prefix and apply it to all topics
        std::string prefix = this->get_parameter("topics.prefix").as_string();
        if (!prefix.empty() && prefix.back() != '/') {
            prefix += "/";  // Ensure prefix ends with /
        }
        
        joint_states_topic_name = prefix + this->get_parameter("topics.cmd_joint_states").as_string();
        std::string endeffector_topic = prefix + this->get_parameter("topics.cmd_endeffector_state").as_string();
        std::string cartesian_topic = prefix + this->get_parameter("topics.cmd_cartesian_pose").as_string();
        std::string mode_topic = prefix + this->get_parameter("topics.cartesian_mode").as_string();
        std::string linear_mode_topic = prefix + this->get_parameter("topics.cartesian_linear_mode").as_string();
        std::string speed_topic = prefix + this->get_parameter("topics.cartesian_speed").as_string();
        std::string cmd_function_topic = prefix + this->get_parameter("topics.cmd_function").as_string();
        std::string response_topic = prefix + this->get_parameter("topics.mirobot_response").as_string();
        std::string status_topic = prefix + this->get_parameter("topics.mirobot_status").as_string();
        std::string serial_port = this->get_parameter("serial.port").as_string();
        int baud_rate = this->get_parameter("serial.baud_rate").as_int();
        bool perform_homing = this->get_parameter("robot.perform_homing_on_startup").as_bool();
        homing_timeout_seconds_ = this->get_parameter("robot.homing_timeout_seconds").as_int();
        cartesian_speed_ = this->get_parameter("robot.default_cartesian_speed").as_double();
        
        // Get default mode and set cartesian_linear_mode_
        std::string default_mode = this->get_parameter("cartesian.default_mode").as_string();
        cartesian_linear_mode_ = (default_mode == "linear");
        
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

        mode_sub_ = this->create_subscription<std_msgs::msg::String>(
            mode_topic, 10, std::bind(&MirobotWriteNode::mode_callback, this, std::placeholders::_1));

        linear_mode_sub_ = this->create_subscription<std_msgs::msg::Bool>(
            linear_mode_topic, 10, std::bind(&MirobotWriteNode::linear_mode_callback, this, std::placeholders::_1));

        speed_sub_ = this->create_subscription<std_msgs::msg::Float64>(
            speed_topic, 10, std::bind(&MirobotWriteNode::speed_callback, this, std::placeholders::_1));

        cmd_function_sub_ = this->create_subscription<std_msgs::msg::String>(
            cmd_function_topic, 10, std::bind(&MirobotWriteNode::handle_cmd_function, this, std::placeholders::_1));

        // Initialize the mirobot_response publisher
        mirobot_response_publisher_ = this->create_publisher<std_msgs::msg::String>(response_topic, 10);

        // Initialize the diagnostic status publisher
        mirobot_status_publisher_ = this->create_publisher<diagnostic_msgs::msg::DiagnosticStatus>(status_topic, 10);

        // Create timer to publish status at 2 Hz
        status_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(500),
            std::bind(&MirobotWriteNode::publish_status, this));

        // Poll serial feedback at 10 Hz to consume continuous state updates (e.g., O104=1)
        feedback_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&MirobotWriteNode::read_serial_feedback_nonblocking, this));

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

        enable_continuous_status_feedback();
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
                
                // Mark robot as busy during homing
                is_busy_ = true;
                last_movement_time_ = std::chrono::steady_clock::now();
                RCLCPP_INFO(this->get_logger(), "Robot is now BUSY (homing in progress)");
            } else {
                RCLCPP_WARN(this->get_logger(), "No data received within the timeout period.");
            }
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Error receiving data: %s", e.what());
        }

        // Wait for configured time to complete the homing process
        RCLCPP_INFO(this->get_logger(), "Waiting for %d seconds to complete homing...", homing_timeout_seconds_);
        rclcpp::sleep_for(std::chrono::seconds(homing_timeout_seconds_));
        is_homed_ = true;  // Mark as homed
        is_busy_ = false;  // Homing complete, robot is now idle
        last_movement_time_ = std::chrono::steady_clock::time_point::min();  // Clear movement time
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
    struct QueuedCommand {
        std::string gcode;
        std::string source;
        double estimated_duration_seconds;
    };

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

    bool send_gcode_and_read_response(const std::string& gcode, const std::string& source) {
        try {
            std::vector<uint8_t> Gcode(gcode.begin(), gcode.end());
            serial_->send(Gcode);
            RCLCPP_INFO(this->get_logger(), "Sent %s G-code: %s", source.c_str(), gcode.c_str());
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Error sending %s G-code: %s", source.c_str(), e.what());
            return false;
        }

        try {
            std::vector<uint8_t> buffer(2056);
            size_t bytes_read = serial_->receive(buffer);

            if (bytes_read > 0) {
                std::string result(buffer.begin(), buffer.begin() + bytes_read);
                RCLCPP_INFO(this->get_logger(), "Received: %s", result.c_str());
                parse_mirobot_response(result);

                auto response_msg = std_msgs::msg::String();
                response_msg.data = result;
                mirobot_response_publisher_->publish(response_msg);
            } else {
                RCLCPP_WARN(this->get_logger(), "No data received within the timeout period.");
            }
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Error receiving %s response: %s", source.c_str(), e.what());
        }

        return true;
    }

    void enable_continuous_status_feedback() {
        ensure_serial_port_open();
        if (!serial_->is_open()) {
            RCLCPP_ERROR(this->get_logger(), "Serial port is not open. Cannot enable continuous status feedback.");
            return;
        }

        try {
            const std::string feedback_enable_gcode = "O104=1\r\n";
            std::vector<uint8_t> gcode(feedback_enable_gcode.begin(), feedback_enable_gcode.end());
            serial_->send(gcode);
            RCLCPP_INFO(this->get_logger(), "Enabled continuous robot state feedback with O104=1");
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Failed to enable continuous status feedback: %s", e.what());
        }
    }

    void read_serial_feedback_nonblocking() {
        if (command_dispatch_in_progress_) {
            return;
        }

        ensure_serial_port_open();
        if (!serial_->is_open()) {
            return;
        }

        try {
            std::vector<uint8_t> buffer(2056);
            size_t bytes_read = serial_->receive(buffer);
            if (bytes_read == 0) {
                return;
            }

            std::string result(buffer.begin(), buffer.begin() + bytes_read);
            parse_mirobot_response(result);

            auto response_msg = std_msgs::msg::String();
            response_msg.data = result;
            mirobot_response_publisher_->publish(response_msg);
        } catch (const std::exception&) {
            // Expected when no serial data is available within timeout.
        }
    }

    void enqueue_command(const std::string& gcode, const std::string& source, double estimated_duration_seconds = 0.0) {
        command_queue_.push_back({gcode, source, estimated_duration_seconds});
        RCLCPP_INFO(this->get_logger(), "Queued %s command (queue size: %zu)", source.c_str(), command_queue_.size());
        process_command_queue();
    }

    void process_command_queue() {
        if (command_dispatch_in_progress_ || is_busy_ || command_queue_.empty()) {
            return;
        }

        ensure_serial_port_open();
        if (!serial_->is_open()) {
            RCLCPP_ERROR(this->get_logger(), "Serial port is not open. Cannot process queued commands.");
            return;
        }

        command_dispatch_in_progress_ = true;
        QueuedCommand command = command_queue_.front();
        command_queue_.pop_front();

        if (command.estimated_duration_seconds > 0.0) {
            estimated_movement_duration_ = command.estimated_duration_seconds;
        }

        bool success = send_gcode_and_read_response(command.gcode, command.source);

        if (!success) {
            has_error_ = true;
            error_message_ = "Failed to send queued command";
            is_busy_ = false;
        }

        command_dispatch_in_progress_ = false;
    }

    void clear_command_queue(const std::string& reason) {
        if (command_queue_.empty()) {
            return;
        }

        size_t dropped = command_queue_.size();
        command_queue_.clear();
        RCLCPP_WARN(this->get_logger(), "Cleared %zu queued command(s): %s", dropped, reason.c_str());
    }

    void joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg) {
        char angle0[10], angle1[10], angle2[10], angle3[10], angle4[10], angle5[10];

        sprintf(angle0, "%.2f", msg->position[0] * 57.296);
        sprintf(angle1, "%.2f", msg->position[1] * 57.296);
        sprintf(angle2, "%.2f", msg->position[2] * 57.296);
        sprintf(angle3, "%.2f", msg->position[3] * 57.296);
        sprintf(angle4, "%.2f", msg->position[4] * 57.296);
        sprintf(angle5, "%.2f", msg->position[5] * 57.296);

        std::string GcodeString = (std::string)"M21 G0 X" + angle0 + " Y" + angle1 + " Z" + angle2 + " A" + angle3 + " B" + angle4 + " C" + angle5 + " F3000" + "\r\n";
        
        RCLCPP_INFO(this->get_logger(), "Generated G-code: %s", GcodeString.c_str());

        last_sent_gcode_ = GcodeString; // Update last queued joint G-code
        enqueue_command(GcodeString, "joint", 5.0);
    }

    void endeffector_state_callback(const mirobot_msgs::msg::EndeffectorState::SharedPtr msg) {
        RCLCPP_INFO(this->get_logger(), "Received end effector state: %d", msg->state);

        // Update gripper state tracking
        if (msg->state == 0) {
            gripper_open_ = false;  // Gripper closed
        } else if (msg->state == 1) {
            gripper_open_ = true;   // Gripper open
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

        enqueue_command(GcodeString, "endeffector", 0.0);
    }

    void mode_callback(const std_msgs::msg::String::SharedPtr msg) {
        std::string mode = msg->data;
        if (mode == "linear") {
            cartesian_linear_mode_ = true;
            RCLCPP_INFO(this->get_logger(), "Cartesian mode set to: Linear (G01)");
        } else if (mode == "ptp") {
            cartesian_linear_mode_ = false;
            RCLCPP_INFO(this->get_logger(), "Cartesian mode set to: PTP (G00)");
        } else {
            RCLCPP_WARN(this->get_logger(), "Unknown mode '%s'. Use 'linear' or 'ptp'.", mode.c_str());
        }
    }

    void linear_mode_callback(const std_msgs::msg::Bool::SharedPtr msg) {
        cartesian_linear_mode_ = msg->data;
        RCLCPP_INFO(this->get_logger(), "Cartesian mode set to: %s (DEPRECATED: Use cartesian_mode topic instead)", 
                    cartesian_linear_mode_ ? "Linear (G01)" : "PTP (G00)");
    }

    void speed_callback(const std_msgs::msg::Float64::SharedPtr msg) {
        cartesian_speed_ = msg->data;
        RCLCPP_INFO(this->get_logger(), "Cartesian speed set to: %.0f mm/s", cartesian_speed_);
    }

    void cartesian_pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
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
        std::string move_type = cartesian_linear_mode_ ? "G01" : "G00";  // Match Wlkata Studio format (two digits)
        std::string mode_name = cartesian_linear_mode_ ? "Linear" : "PTP";

        // Build the G-code command: M20 G90 G00/G01 X Y Z A B C F<speed>
        std::string GcodeString = (std::string)"M20 G90 " + move_type + " X" + x_str + " Y" + y_str + " Z" + z_str + 
                                  " A" + roll_str + " B" + pitch_str + " C" + yaw_str + " F" + speed_str + "\r\n";

        RCLCPP_INFO(this->get_logger(), "Generated Cartesian G-code (%s): %s", mode_name.c_str(), GcodeString.c_str());

        // Calculate estimated movement time based on distance and speed
        double distance = std::sqrt(
            std::pow(x_mm - last_x_, 2) + 
            std::pow(y_mm - last_y_, 2) + 
            std::pow(z_mm - last_z_, 2)
        );
        
        // Estimate time: distance / speed, add 20% safety margin + 1 second for acceleration
        if (cartesian_speed_ > 0) {
            estimated_movement_duration_ = (distance / cartesian_speed_) * 1.2 + 1.0;
            // Clamp between 2 and 15 seconds
            estimated_movement_duration_ = std::max(2.0, std::min(15.0, estimated_movement_duration_));
            RCLCPP_INFO(this->get_logger(), "Movement: %.1f mm at %.0f mm/s → estimated %.1f seconds", 
                        distance, cartesian_speed_, estimated_movement_duration_);
        } else {
            estimated_movement_duration_ = 5.0;  // Default if speed not set
        }
        
        // Update last position
        last_x_ = x_mm;
        last_y_ = y_mm;
        last_z_ = z_mm;

        last_sent_cartesian_gcode_ = GcodeString; // Update last queued Cartesian G-code
        enqueue_command(GcodeString, "cartesian", estimated_movement_duration_);
    }

    void parse_mirobot_response(const std::string& response) {
        // Parse Mirobot responses to update robot state
        // Common responses: "Info,Idle", "Info,Busy", "Info,Run", "Info,E100,..."
        
        RCLCPP_INFO(this->get_logger(), "🔍 PARSING RESPONSE: '%s' (length: %zu)", response.c_str(), response.length());

        // Motion_MODE appears in continuous telemetry, but in observed logs it is not
        // a reliable movement-state indicator. Treat it as informational only.
        const std::string motion_mode_key = "Motion_MODE:";
        size_t motion_mode_pos = response.rfind(motion_mode_key);
        if (motion_mode_pos != std::string::npos) {
            size_t value_start = motion_mode_pos + motion_mode_key.size();
            while (value_start < response.size() && !std::isdigit(static_cast<unsigned char>(response[value_start]))) {
                ++value_start;
            }

            size_t value_end = value_start;
            while (value_end < response.size() && std::isdigit(static_cast<unsigned char>(response[value_end]))) {
                ++value_end;
            }

            if (value_end > value_start) {
                int motion_mode = std::stoi(response.substr(value_start, value_end - value_start));
                if (motion_mode != last_motion_mode_) {
                    last_motion_mode_ = motion_mode;
                    RCLCPP_INFO(this->get_logger(), "ℹ️ Telemetry Motion_MODE=%d (informational)", motion_mode);
                }
            }
        }
        
        if (response.find("Idle") != std::string::npos) {
            RCLCPP_INFO(this->get_logger(), "✅ Status: IDLE detected in response - setting is_busy_ = false");
            is_busy_ = false;
            has_error_ = false;
            last_movement_time_ = std::chrono::steady_clock::time_point::min();  // Clear movement time
        } else if (response.find("Busy") != std::string::npos || response.find("Run") != std::string::npos) {
            RCLCPP_INFO(this->get_logger(), "🔄 Status: BUSY/RUN detected in response - setting is_busy_ = true");
            bool was_busy = is_busy_;
            is_busy_ = true;
            has_error_ = false;
            if (!was_busy || last_movement_time_ == std::chrono::steady_clock::time_point::min()) {
                last_movement_time_ = std::chrono::steady_clock::now();  // Record movement start time
            }
        } else if (response.find(",E") != std::string::npos) {
            // Error detected (e.g., "Info,E100,Expected command letter"
            RCLCPP_WARN(this->get_logger(), "⚠️ Status: ERROR detected in response");
            has_error_ = true;
            is_busy_ = false;
            last_movement_time_ = std::chrono::steady_clock::time_point::min();
            clear_command_queue("Firmware error response");
        } else if (response.find("ok") != std::string::npos) {
            // Robot acknowledges command with "ok" - start movement
            RCLCPP_INFO(this->get_logger(), "✅ Status: OK detected - robot starting movement, setting is_busy_ = true");
            bool was_busy = is_busy_;
            is_busy_ = true;
            has_error_ = false;
            if (!was_busy || last_movement_time_ == std::chrono::steady_clock::time_point::min()) {
                last_movement_time_ = std::chrono::steady_clock::now();  // Record movement start time
            }
        }

        if (!is_busy_) {
            process_command_queue();
        }
    }

    void publish_status() {
        // Status is updated from command responses via parse_mirobot_response()
        // We don't query here to avoid interfering with robot movements
        
        RCLCPP_INFO(this->get_logger(), "=== STATUS CHECK === is_busy_: %s, is_homed_: %s, has_error_: %s", 
                     is_busy_ ? "true" : "false", is_homed_ ? "true" : "false", has_error_ ? "true" : "false");
        
        // If robot was moving, check if enough time has passed to assume it's idle
        if (is_busy_ && last_movement_time_ != std::chrono::steady_clock::time_point::min()) {
            auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(
                std::chrono::steady_clock::now() - last_movement_time_).count();
            
            // Use estimated movement duration (calculated from distance/speed) or default to 10 seconds
            double timeout = (estimated_movement_duration_ > 0) ? estimated_movement_duration_ : 10.0;
            
            RCLCPP_INFO(this->get_logger(), "Movement elapsed time: %ld seconds (timeout at %.1fs)", elapsed, timeout);
            
            // Assume movement finished after estimated duration
            if (elapsed > timeout) {
                RCLCPP_INFO(this->get_logger(), "⏰ TIMEOUT: Movement timeout - setting robot to idle");
                is_busy_ = false;
                estimated_movement_duration_ = 0.0;  // Reset for next movement
            }
        } else if (!is_busy_) {
            RCLCPP_INFO(this->get_logger(), "Robot is NOT busy");
        }
        
        auto status_msg = diagnostic_msgs::msg::DiagnosticStatus();
        status_msg.name = robot_name_;
        status_msg.hardware_id = robot_name_;
        
        // Determine state and level
        std::string state;
        if (has_error_) {
            status_msg.level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
            state = "error";
            status_msg.message = "Robot error";
            RCLCPP_INFO(this->get_logger(), "📊 Publishing status: ERROR - %s", error_message_.c_str());
        } else if (!is_homed_) {
            status_msg.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
            state = "not_homed";
            status_msg.message = "Robot not homed";
            RCLCPP_INFO(this->get_logger(), "📊 Publishing status: WARN - not_homed");
        } else if (is_busy_) {
            status_msg.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
            state = "busy";
            status_msg.message = "Robot busy";
            RCLCPP_INFO(this->get_logger(), "📊 Publishing status: OK - busy");
        } else {
            status_msg.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
            state = "idle";
            status_msg.message = "Robot idle and ready";
            RCLCPP_INFO(this->get_logger(), "📊 Publishing status: OK - idle");
        }
        
        // Add key-value pairs for additional information
        diagnostic_msgs::msg::KeyValue kv_state;
        kv_state.key = "state";
        kv_state.value = state;
        status_msg.values.push_back(kv_state);
        
        diagnostic_msgs::msg::KeyValue kv_homed;
        kv_homed.key = "is_homed";
        kv_homed.value = is_homed_ ? "true" : "false";
        status_msg.values.push_back(kv_homed);
        
        diagnostic_msgs::msg::KeyValue kv_gripper;
        kv_gripper.key = "gripper_open";
        kv_gripper.value = gripper_open_ ? "true" : "false";
        status_msg.values.push_back(kv_gripper);
        
        diagnostic_msgs::msg::KeyValue kv_linear_mode;
        kv_linear_mode.key = "linear_mode";
        kv_linear_mode.value = cartesian_linear_mode_ ? "true" : "false";
        status_msg.values.push_back(kv_linear_mode);
        
        diagnostic_msgs::msg::KeyValue kv_speed;
        kv_speed.key = "cartesian_speed_mm_per_sec";
        kv_speed.value = std::to_string(cartesian_speed_);
        status_msg.values.push_back(kv_speed);
        
        mirobot_status_publisher_->publish(status_msg);

        if (!is_busy_) {
            process_command_queue();
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
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr mode_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr linear_mode_sub_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr speed_sub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr cmd_function_sub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr mirobot_response_publisher_;
    rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticStatus>::SharedPtr mirobot_status_publisher_;
    rclcpp::TimerBase::SharedPtr status_timer_;
    rclcpp::TimerBase::SharedPtr feedback_timer_;
    std::string joint_states_topic_name;
    int homing_timeout_seconds_;
    
    // Cartesian control parameters
    double cartesian_speed_;
    
    // Position tracking for movement time estimation
    double last_x_ = 0.0;
    double last_y_ = 0.0;
    double last_z_ = 0.0;
    double estimated_movement_duration_ = 0.0;  // Estimated time for current movement in seconds
    bool cartesian_linear_mode_;
    
    // Robot state tracking
    std::string robot_name_;
    bool is_homed_ = false;
    bool is_busy_ = false;
    bool has_error_ = false;
    bool gripper_open_ = false;
    std::string error_message_ = "";
    int last_motion_mode_ = -1;
    std::chrono::steady_clock::time_point last_movement_time_ = std::chrono::steady_clock::time_point::min();
    std::deque<QueuedCommand> command_queue_;
    bool command_dispatch_in_progress_ = false;
    
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
