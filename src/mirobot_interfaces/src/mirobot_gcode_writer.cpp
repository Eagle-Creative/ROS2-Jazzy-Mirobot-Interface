#include "serial_driver/serial_driver.hpp"
#include "serial_driver/serial_port.hpp"
#include "io_context/io_context.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "mirobot_msgs/msg/endeffector_state.hpp"

// Create an IoContext object
drivers::common::IoContext io_context;

// Configure the SerialPort
drivers::serial_driver::SerialPortConfig config(115200, drivers::serial_driver::FlowControl::NONE,
                                                drivers::serial_driver::Parity::NONE,
                                                drivers::serial_driver::StopBits::ONE);

// Initialize the SerialPort object
drivers::serial_driver::SerialPort _serial(io_context, "/dev/ttyUSB0", config);

class MirobotWriteNode : public rclcpp::Node {
public:
    MirobotWriteNode() : Node("mirobot_gcode_writer") {
        this->declare_parameter("joint_states_topic_name", "joint_states");
        joint_states_topic_name = this->get_parameter("joint_states_topic_name").as_string();
        RCLCPP_INFO(this->get_logger(), "Joint States Topic Name : %s", joint_states_topic_name.c_str());

        js_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
            joint_states_topic_name, 10, std::bind(&MirobotWriteNode::joint_state_callback, this, std::placeholders::_1));

        ee_state_sub_ = this->create_subscription<mirobot_msgs::msg::EndeffectorState>(
            "/endeffector_state", 10, std::bind(&MirobotWriteNode::endeffector_state_callback, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "Subscribed to /endeffector_state");

        // Check if the serial port is open
        try {
            if (!_serial.is_open()) {
                RCLCPP_ERROR(this->get_logger(), "Failed to open serial port /dev/ttyUSB0");
            } else {
                RCLCPP_INFO(this->get_logger(), "Serial port /dev/ttyUSB0 opened successfully.");
            }
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Exception during serial port initialization: %s", e.what());
        }

        // Perform homing during startup
        homing();
    }

    void homing() {
        std::vector<uint8_t> HomingGcode = {'$', 'H', '\r', '\n'};
        ensure_serial_port_open();

        // Check if the serial port is open
        if (!_serial.is_open()) {
            RCLCPP_ERROR(this->get_logger(), "Serial port is not open. Cannot send homing command.");
            return;
        }

        // Send the G-code command to the serial port
        try {
            _serial.send(HomingGcode);
            RCLCPP_INFO(this->get_logger(), "Homing command sent successfully.");
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Error sending homing command: %s", e.what());
            return;
        }

        // Receive the response from the serial port
        try {
            std::vector<uint8_t> buffer(1024);
            size_t bytes_read = _serial.receive(buffer);

            if (bytes_read > 0) {
                std::string result(buffer.begin(), buffer.begin() + bytes_read);
                RCLCPP_INFO(this->get_logger(), "Received: %s", result.c_str());
            } else {
                RCLCPP_WARN(this->get_logger(), "No data received within the timeout period.");
            }
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Error receiving data: %s", e.what());
        }

        // Wait for 20 seconds to complete the homing process
        RCLCPP_INFO(this->get_logger(), "Waiting for 20 seconds to complete homing...");
        rclcpp::sleep_for(std::chrono::seconds(20));
        RCLCPP_INFO(this->get_logger(), "Homing completed. The robot is ready for usage.");
    }

private:
    void ensure_serial_port_open() {
        if (!_serial.is_open()) {
            try {
                _serial.open();
                RCLCPP_INFO(this->get_logger(), "Serial port reopened successfully.");
            } catch (const std::exception& e) {
                RCLCPP_ERROR(this->get_logger(), "Failed to reopen serial port: %s", e.what());
            }
        }
    }

    void joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg) {
        ensure_serial_port_open(); // Ensure the serial port is open before proceeding

        if (!_serial.is_open()) {
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

        std::string GcodeString = (std::string)"M21 G0 X" + angle0 + " Y" + angle1 + " Z" + angle2 + " A" + angle3 + "B" + angle4 + "C" + angle5 + " F3000" + "\r\n";
        
        // Check if the new G-code is the same as the last sent one
        if (GcodeString == last_sent_gcode_) {
            return;
        }

        RCLCPP_INFO(this->get_logger(), "Generated G-code: %s", GcodeString.c_str());

        std::vector<uint8_t> Gcode(GcodeString.begin(), GcodeString.end());

        try {
            
            _serial.send(Gcode);
            RCLCPP_INFO(this->get_logger(), "Sent G-code: %s", GcodeString.c_str());
            last_sent_gcode_ = GcodeString; // Update the last sent G-code
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Error sending data: %s", e.what());
            return;
        }

        try {            
            std::vector<uint8_t> buffer(2056);
            size_t bytes_read = _serial.receive(buffer);

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

        if (!_serial.is_open()) {
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
            _serial.send(Gcode);
            RCLCPP_INFO(this->get_logger(), "Sent G-code for end effector state: %s", GcodeString.c_str());
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Error sending end effector G-code: %s", e.what());
        }
    }

    std::string last_sent_gcode_; // Store the last sent G-code
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr js_sub_;
    rclcpp::Subscription<mirobot_msgs::msg::EndeffectorState>::SharedPtr ee_state_sub_;
    std::string joint_states_topic_name;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);

    auto node = std::make_shared<MirobotWriteNode>();

    RCLCPP_INFO(node->get_logger(), "Mirobot G-code Writer Node is ready.");
    
    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}
