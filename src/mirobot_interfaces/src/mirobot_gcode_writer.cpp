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
        // Upper bound for homing; completion itself is detected from the firmware
        // state (Idle after $H), not assumed after a fixed wait.
        this->declare_parameter("robot.homing_timeout_seconds", 60);
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

        // Initialize the joint_states publisher for actual robot position feedback
        // Use the same topic as configured for input joint_states but with prefix
        std::string joint_states_feedback_topic = prefix + "joint_states";
        joint_states_publisher_ = this->create_publisher<sensor_msgs::msg::JointState>(joint_states_feedback_topic, 10);

        // Create timer to publish status at 2 Hz
        status_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(500),
            std::bind(&MirobotWriteNode::publish_status, this));

        // Poll serial feedback at 50 Hz for smooth joint state updates in RViz
        feedback_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(20),
            std::bind(&MirobotWriteNode::read_serial_feedback_nonblocking, this));

        RCLCPP_INFO(this->get_logger(), "Subscribed to topics: %s, %s, %s", 
                    endeffector_topic.c_str(), cartesian_topic.c_str(), cmd_function_topic.c_str());

        // Open now (it was only constructed above) - the old check always logged
        // "Failed to open" because nothing had opened the port yet.
        ensure_serial_port_open();
        try {
            if (!serial_->is_open()) {
                RCLCPP_ERROR(this->get_logger(), "Failed to open serial port %s", serial_port.c_str());
            } else {
                RCLCPP_INFO(this->get_logger(), "Serial port %s opened successfully.", serial_port.c_str());
            }
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Exception during serial port initialization: %s", e.what());
        }

        // Telemetry first: homing completion and the homed state are read from it.
        enable_continuous_status_feedback();

        // Perform homing during startup if enabled
        if (perform_homing) {
            homing();
        } else {
            RCLCPP_INFO(this->get_logger(), "Homing on startup is disabled (homed state follows the firmware telemetry).");
        }
    }

    void homing() {
        // Non-blocking: send $H and let the telemetry (read_serial_feedback_nonblocking
        // -> parse_mirobot_response) report completion.  The firmware is in "Alarm"
        // until homed and reports "Idle" once homing finished; publish_status()
        // flags a homing that does not finish within robot.homing_timeout_seconds.
        std::vector<uint8_t> HomingGcode = {'$', 'H', '\r', '\n'};
        ensure_serial_port_open();

        if (!serial_->is_open()) {
            RCLCPP_ERROR(this->get_logger(), "Serial port is not open. Cannot send homing command.");
            return;
        }

        try {
            serial_->send(HomingGcode);
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Error sending homing command: %s", e.what());
            return;
        }

        homing_in_progress_ = true;
        homing_started_ = std::chrono::steady_clock::now();
        is_homed_ = false;            // joint feedback is invalid until homing completes
        is_busy_ = true;
        has_error_ = false;
        error_message_.clear();
        last_movement_time_ = std::chrono::steady_clock::time_point::min();  // homing has its own timeout
        RCLCPP_INFO(this->get_logger(), "Homing started ($H) - waiting for the firmware to report Idle (timeout %d s)",
                    homing_timeout_seconds_);
    }

    // Firmware state of a telemetry frame: "<Idle,Angle(...)...>" -> "Idle".
    static std::string firmware_state_of(const std::string& response) {
        size_t lt = response.find('<');
        if (lt == std::string::npos) {
            return "";
        }
        size_t comma = response.find(',', lt + 1);
        size_t gt = response.find('>', lt + 1);
        size_t end = std::min(comma == std::string::npos ? response.size() : comma,
                              gt == std::string::npos ? response.size() : gt);
        return response.substr(lt + 1, end - lt - 1);
    }

    void update_homed_state(const std::string& state) {
        if (state.empty()) {
            return;
        }
        if (state != firmware_state_) {
            RCLCPP_INFO(this->get_logger(), "Firmware state: %s -> %s",
                        firmware_state_.empty() ? "(unknown)" : firmware_state_.c_str(), state.c_str());
            firmware_state_ = state;
        }
        if (state == "Alarm") {
            // Power-on / limit alarm: position unknown until homed.
            if (is_homed_) {
                RCLCPP_WARN(this->get_logger(), "Firmware reports Alarm - robot is no longer homed");
            }
            is_homed_ = false;
            return;
        }
        if (state == "Home") {
            return;  // homing cycle running
        }
        if (homing_in_progress_) {
            // Ignore frames that may predate the $H (sent before the firmware switched state).
            auto since_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                std::chrono::steady_clock::now() - homing_started_).count();
            if (since_ms < 1500 || state != "Idle") {
                return;
            }
            homing_in_progress_ = false;
            is_busy_ = false;
            RCLCPP_INFO(this->get_logger(), "Homing completed after %.1f s - robot ready", since_ms / 1000.0);
        }
        if (!is_homed_) {
            RCLCPP_INFO(this->get_logger(), "Robot homed (firmware state %s) - publishing joint states", state.c_str());
        }
        is_homed_ = true;
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
            RCLCPP_INFO(this->get_logger(), "Manual G-code sent: %s", gcode.c_str());
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Error sending GCODE override command: %s", e.what());
        }
        // No synchronous read here: read_serial_feedback_nonblocking() owns the serial input
        // and publishes complete messages on mirobot_response. Reading here took a random
        // chunk out of the telemetry stream; the reader then stitched the start of one status
        // frame to the end of another and published wrong joint angles (2026-10-26).
    }

private:
    struct QueuedCommand {
        uint64_t id;
        std::string gcode;
        std::string source;
        double estimated_duration_seconds;
    };

    enum class MotionPhase {
        NONE,
        AWAITING_START,
        IN_MOTION
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

    bool send_gcode(const std::string& gcode, const std::string& source) {
        try {
            std::vector<uint8_t> Gcode(gcode.begin(), gcode.end());
            serial_->send(Gcode);
            // Sent G-code (silent)
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Error sending %s G-code: %s", source.c_str(), e.what());
            return false;
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
            // Read all available data from serial buffer (loop to drain)
            int reads_done = 0;
            const int max_reads_per_call = 10; // Limit to avoid blocking too long
            
            while (reads_done < max_reads_per_call) {
                std::vector<uint8_t> buffer(2056);
                size_t bytes_read = serial_->receive(buffer);
                if (bytes_read == 0) {
                    break; // No more data available
                }

                // Accumulate data in buffer
                serial_buffer_.append(buffer.begin(), buffer.begin() + bytes_read);
                reads_done++;
            }
            
            // Process complete messages (ending with '>' or '\n')
            size_t message_end;
            while ((message_end = serial_buffer_.find('>')) != std::string::npos) {
                // Extract complete message including '>'
                std::string complete_message = serial_buffer_.substr(0, message_end + 1);
                serial_buffer_.erase(0, message_end + 1);
                
                // Parse and publish complete message (angles are taken from the last
                // "<...>" frame only, see parse_mirobot_response)
                parse_mirobot_response(complete_message);
                
                auto response_msg = std_msgs::msg::String();
                response_msg.data = complete_message;
                mirobot_response_publisher_->publish(response_msg);
            }
            
            // Prevent buffer from growing indefinitely if no '>' found
            if (serial_buffer_.size() > 4096) {
                RCLCPP_WARN(this->get_logger(), "Serial buffer overflow, clearing: %s", serial_buffer_.c_str());
                serial_buffer_.clear();
            }
        } catch (const std::exception&) {
            // Expected when no serial data is available within timeout.
        }
    }

    void enqueue_command(const std::string& gcode, const std::string& source, double estimated_duration_seconds = 0.0) {
        uint64_t command_id = next_command_id_++;
        command_queue_.push_back({command_id, gcode, source, estimated_duration_seconds});
        // Queued command (silent)
        process_command_queue();
    }

    void process_command_queue() {
        if (command_dispatch_in_progress_ || command_in_flight_ || is_busy_ || homing_in_progress_ || command_queue_.empty()) {
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
        active_command_id_ = command.id;
        active_command_source_ = command.source;
        command_sent_time_ = std::chrono::steady_clock::now();

        if (command.estimated_duration_seconds > 0.0) {
            estimated_movement_duration_ = command.estimated_duration_seconds;
        }

        bool success = send_gcode(command.gcode, command.source);

        if (!success) {
            has_error_ = true;
            error_message_ = "Failed to send queued command";
            is_busy_ = false;
            command_in_flight_ = false;
            motion_phase_ = MotionPhase::NONE;
            active_command_id_ = 0;
            active_command_source_.clear();
        } else {
            command_in_flight_ = true;
            // Dispatched command (silent)

            // Motion commands must complete before next dispatch.
            if (command.source == "cartesian" || command.source == "joint") {
                is_busy_ = true;
                has_error_ = false;
                motion_phase_ = MotionPhase::AWAITING_START;
                last_movement_time_ = std::chrono::steady_clock::now();
            } else if (command.source == "endeffector") {
                // End-effector commands: set busy for 1 second to allow gripper action
                is_busy_ = true;
                has_error_ = false;
                motion_phase_ = MotionPhase::IN_MOTION;
                last_movement_time_ = std::chrono::steady_clock::now();
                estimated_movement_duration_ = 1.0;  // 1 second for gripper action
                RCLCPP_INFO(this->get_logger(), "Endeffector command dispatched, setting busy for 1 second");
            } else {
                // Other function commands are treated as immediate.
                RCLCPP_INFO(this->get_logger(), "Completed command #%lu (%s): immediate", active_command_id_, active_command_source_.c_str());
                command_in_flight_ = false;
                motion_phase_ = MotionPhase::NONE;
                active_command_id_ = 0;
                active_command_source_.clear();
            }
        }

        command_dispatch_in_progress_ = false;

        if (!is_busy_) {
            process_command_queue();
        }
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

        // Received Cartesian pose (silent)

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

        // Generated G-code (silent)

        // Calculate estimated movement time based on distance and speed
        double distance = std::sqrt(
            std::pow(x_mm - last_x_, 2) + 
            std::pow(y_mm - last_y_, 2) + 
            std::pow(z_mm - last_z_, 2)
        );
        
        // Estimate time: distance / speed, add 20% safety margin + 1 second for acceleration
        if (cartesian_speed_ > 0) {
            estimated_movement_duration_ = (distance / cartesian_speed_) * 1.2 + 1.0;
            // Clamp between 2 and 60 seconds
            estimated_movement_duration_ = std::max(2.0, std::min(60.0, estimated_movement_duration_));
            // Movement estimation (silent)
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
        // Telemetry frames (O104=1): "<Idle|Alarm|Home|Run,Angle(ABCDXYZ):...>"
        update_homed_state(firmware_state_of(response));
        
        // Parse response (silent - only errors shown)

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
                    // Motion mode changed (silent)
                }
            }
        }
        
        // Parse joint angles from telemetry response
        // Format: "Angle(ABCDXYZ):J4,J5,J6,D,J1,J2,J3,Cartesian..."
        // Telemetry order: [J4, J5, J6, D(rail/gripper), J1, J2, J3] = 7 values
        // Angles only from the last complete "<...>" frame: text before it ("ok", error
        // lines, a partial frame) must not feed the angle list.
        const size_t frame_start = response.rfind('<');
        const std::string frame = frame_start != std::string::npos ? response.substr(frame_start) : std::string();
        size_t angle_start = frame.find("Angle(ABCDXYZ):");
        if (angle_start != std::string::npos) {
            angle_start += 15;  // Skip "Angle(ABCDXYZ):"
            
            // The angle list must end at ",Cartesian" and hold exactly 7 complete numbers
            // (J4,J5,J6,D,J1,J2,J3). Anything else is a damaged frame: drop it rather than
            // publish shifted values (2026-10-26: stitched frames showed J1..J3 = 0 and
            // J6 = 52 deg on an arm standing still).
            size_t angle_end = frame.find(",Cartesian", angle_start);
            std::vector<double> parsed_angles;
            bool frame_ok = angle_end != std::string::npos;
            if (frame_ok) {
                std::string angles_str = frame.substr(angle_start, angle_end - angle_start);
                size_t pos = 0;
                while (frame_ok && pos <= angles_str.length()) {
                    size_t comma_pos = angles_str.find(',', pos);
                    size_t end_pos = (comma_pos != std::string::npos) ? comma_pos : angles_str.length();
                    std::string angle_str = angles_str.substr(pos, end_pos - pos);
                    try {
                        size_t used = 0;
                        double angle_deg = std::stod(angle_str, &used);
                        if (used != angle_str.size()) {
                            frame_ok = false;  // e.g. "-37.712.714": two frames glued together
                        } else {
                            parsed_angles.push_back(angle_deg * M_PI / 180.0);
                        }
                    } catch (const std::exception&) {
                        frame_ok = false;
                    }
                    if (comma_pos == std::string::npos) {
                        break;
                    }
                    pos = comma_pos + 1;
                }
                frame_ok = frame_ok && parsed_angles.size() == 7;
            }
            if (!frame_ok) {
                // Not published as joint states; the status handling below still runs.
                ++dropped_frames_;
                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 10000,
                    "Dropped damaged telemetry frame (%llu dropped so far): %s",
                    static_cast<unsigned long long>(dropped_frames_), frame.substr(0, 120).c_str());
            }

            // Telemetry order: [J4, J5, J6, D, J1, J2, J3]
            // We need to reorder to: [J1, J2, J3, J4, J5, J6]
            if (frame_ok && parsed_angles.size() == 7) {
                static bool first_parse = true;
                if (first_parse) {
                    RCLCPP_INFO(this->get_logger(), "✅ Joint angle parsing active - receiving telemetry");
                    first_parse = false;
                }
                
                std::vector<double> reordered_joints(6);
                if (parsed_angles.size() >= 7) {
                    // Full telemetry with gripper/rail angle (most common)
                    reordered_joints[0] = parsed_angles[4];  // J1
                    reordered_joints[1] = parsed_angles[5];  // J2
                    reordered_joints[2] = parsed_angles[6];  // J3
                    reordered_joints[3] = parsed_angles[0];  // J4
                    reordered_joints[4] = parsed_angles[1];  // J5
                    reordered_joints[5] = parsed_angles[2];  // J6
                } else {
                    // Only 6 values (D axis missing - rare case)
                    reordered_joints[0] = parsed_angles[3];  // J1
                    reordered_joints[1] = parsed_angles[4];  // J2
                    reordered_joints[2] = parsed_angles[5];  // J3
                    reordered_joints[3] = parsed_angles[0];  // J4
                    reordered_joints[4] = parsed_angles[1];  // J5
                    reordered_joints[5] = parsed_angles[2];  // J6
                }
                
                current_joint_positions_ = reordered_joints;
                joint_positions_initialized_ = true;

                // Before homing (Alarm) the firmware reports 0 for every joint
                // regardless of the real pose: publishing that would show a wrong
                // arm.  Publish only once the position is known.
                if (!is_homed_) {
                    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 10000,
                        "Not publishing joint states: robot not homed (firmware state %s)",
                        firmware_state_.empty() ? "unknown" : firmware_state_.c_str());
                    return;
                }
                
                auto joint_msg = sensor_msgs::msg::JointState();
                joint_msg.header.stamp = this->now();
                joint_msg.header.frame_id = "base_link";
                joint_msg.name = {"joint1", "joint2", "joint3", "joint4", "joint5", "joint6"};
                joint_msg.position = reordered_joints;
                
                joint_states_publisher_->publish(joint_msg);
            }
        }
        
        if (response.find("Idle") != std::string::npos) {
            if (command_in_flight_ && motion_phase_ == MotionPhase::AWAITING_START) {
                auto since_send_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                    std::chrono::steady_clock::now() - command_sent_time_).count();
                if (since_send_ms < 300) {
                    RCLCPP_INFO(this->get_logger(), "ℹ️ IDLE while awaiting motion start (%ldms) - ignoring possible stale frame", since_send_ms);
                } else {
                    RCLCPP_INFO(this->get_logger(), "✅ IDLE after send (%ldms) - completing command #%lu", since_send_ms, active_command_id_);
                    is_busy_ = false;
                    command_in_flight_ = false;
                    motion_phase_ = MotionPhase::NONE;
                    has_error_ = false;
                    last_movement_time_ = std::chrono::steady_clock::time_point::min();
                    if (active_command_id_ != 0) {
                        RCLCPP_INFO(this->get_logger(), "Completed command #%lu (%s): idle-after-send", active_command_id_, active_command_source_.c_str());
                    }
                    active_command_id_ = 0;
                    active_command_source_.clear();
                }
            } else {
                // IDLE status detected - clear busy flags
                is_busy_ = false;
                command_in_flight_ = false;
                motion_phase_ = MotionPhase::NONE;
                has_error_ = false;
                last_movement_time_ = std::chrono::steady_clock::time_point::min();  // Clear movement time
                if (active_command_id_ != 0) {
                    RCLCPP_INFO(this->get_logger(), "Completed command #%lu (%s): idle", active_command_id_, active_command_source_.c_str());
                }
                active_command_id_ = 0;
                active_command_source_.clear();
            }
        } else if (response.find("Busy") != std::string::npos || response.find("Run") != std::string::npos) {
            RCLCPP_INFO(this->get_logger(), "🔄 Status: BUSY/RUN detected in response - setting is_busy_ = true");
            bool was_busy = is_busy_;
            is_busy_ = true;
            command_in_flight_ = true;
            motion_phase_ = MotionPhase::IN_MOTION;
            has_error_ = false;
            if (!was_busy || last_movement_time_ == std::chrono::steady_clock::time_point::min()) {
                last_movement_time_ = std::chrono::steady_clock::now();  // Record movement start time
            }
        } else if (response.find(",E") != std::string::npos) {
            // Error detected (e.g., "Info,E100,Expected command letter"
            RCLCPP_WARN(this->get_logger(), "⚠️ Status: ERROR detected in response");
            has_error_ = true;
            is_busy_ = false;
            command_in_flight_ = false;
            motion_phase_ = MotionPhase::NONE;
            last_movement_time_ = std::chrono::steady_clock::time_point::min();
            clear_command_queue("Firmware error response");
            active_command_id_ = 0;
            active_command_source_.clear();
        } else if (response.find("ok") != std::string::npos) {
            // Robot acknowledges command with "ok" - start movement
            RCLCPP_INFO(this->get_logger(), "✅ Status: OK detected - robot starting movement, setting is_busy_ = true");
            bool was_busy = is_busy_;
            is_busy_ = true;
            command_in_flight_ = true;
            motion_phase_ = MotionPhase::IN_MOTION;
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
        
        // If robot was moving, check if enough time has passed to assume it's idle
        if (is_busy_ && last_movement_time_ != std::chrono::steady_clock::time_point::min()) {
            auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(
                std::chrono::steady_clock::now() - last_movement_time_).count();
            
            // Use estimated movement duration (calculated from distance/speed) or default to 30 seconds
            double timeout = (estimated_movement_duration_ > 0) ? estimated_movement_duration_ : 30.0;
            
            // Assume movement finished after estimated duration
            if (elapsed > timeout) {
                RCLCPP_INFO(this->get_logger(), "⏰ TIMEOUT: Movement timeout - setting robot to idle");
                is_busy_ = false;
                command_in_flight_ = false;
                motion_phase_ = MotionPhase::NONE;
                estimated_movement_duration_ = 0.0;  // Reset for next movement
                if (active_command_id_ != 0) {
                    RCLCPP_INFO(this->get_logger(), "Completed command #%lu (%s): timeout", active_command_id_, active_command_source_.c_str());
                }
                active_command_id_ = 0;
                active_command_source_.clear();
            }
        }
        
        auto status_msg = diagnostic_msgs::msg::DiagnosticStatus();
        status_msg.name = robot_name_;
        status_msg.hardware_id = robot_name_;
        
        // Homing that never reaches Idle: report instead of assuming success.
        if (homing_in_progress_) {
            auto since_s = std::chrono::duration_cast<std::chrono::seconds>(
                std::chrono::steady_clock::now() - homing_started_).count();
            if (since_s > homing_timeout_seconds_) {
                homing_in_progress_ = false;
                is_busy_ = false;
                has_error_ = true;
                error_message_ = "Homing did not complete within " + std::to_string(homing_timeout_seconds_) +
                                 " s (firmware state " + (firmware_state_.empty() ? std::string("unknown") : firmware_state_) + ")";
                RCLCPP_ERROR(this->get_logger(), "%s", error_message_.c_str());
            }
        }

        // Determine state and level
        std::string state;
        if (homing_in_progress_) {
            status_msg.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
            state = "homing";
            status_msg.message = "Robot homing";
        } else if (has_error_) {
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
        } else {
            status_msg.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
            state = "idle";
            status_msg.message = "Robot idle and ready";
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
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_states_publisher_;
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
    bool homing_in_progress_ = false;
    std::chrono::steady_clock::time_point homing_started_;
    std::string firmware_state_;   // last telemetry state: Idle, Alarm, Home, Run, ...
    bool is_busy_ = false;
    bool has_error_ = false;
    bool gripper_open_ = false;
    std::string error_message_ = "";
    int last_motion_mode_ = -1;
    uint64_t dropped_frames_ = 0;  // damaged/partial telemetry frames not published
    
    // Current joint positions (in radians) from robot feedback
    std::vector<double> current_joint_positions_ = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    bool joint_positions_initialized_ = false;
    std::chrono::steady_clock::time_point last_movement_time_ = std::chrono::steady_clock::time_point::min();
    std::deque<QueuedCommand> command_queue_;
    bool command_dispatch_in_progress_ = false;
    bool command_in_flight_ = false;
    MotionPhase motion_phase_ = MotionPhase::NONE;
    uint64_t next_command_id_ = 1;
    uint64_t active_command_id_ = 0;
    std::string active_command_source_;
    std::chrono::steady_clock::time_point command_sent_time_ = std::chrono::steady_clock::time_point::min();
    
    // Serial port members
    drivers::common::IoContext io_context_;
    drivers::serial_driver::SerialPortConfig serial_config_;
    std::unique_ptr<drivers::serial_driver::SerialPort> serial_;
    std::string serial_buffer_;  // Buffer for accumulating partial serial messages
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);

    auto node = std::make_shared<MirobotWriteNode>();

    RCLCPP_INFO(node->get_logger(), "Mirobot G-code Writer Node is ready.");
    
    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}
