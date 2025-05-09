#include "serial_driver/serial_driver.hpp"
#include "serial_driver/serial_port.hpp"
#include "io_context/io_context.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

// Create an IoContext object
drivers::common::IoContext io_context;

drivers::serial_driver::SerialPortConfig config(115200,drivers::serial_driver::FlowControl::NONE,
                                                drivers::serial_driver::Parity::NONE,
                                                drivers::serial_driver::StopBits::ONE);


// Initialize the SerialPort object
drivers::serial_driver::SerialPort _serial(io_context, "/dev/ttyUSB0", config);

class MirobotWriteNode : public rclcpp::Node {
public:
    MirobotWriteNode() : Node("mirobot_write_node") {
        this->declare_parameter("joint_states_topic_name", "/issac/joint_states");
// this->declare_parameter("joint_states_topic_name", "/issac/joint_command");
        
        joint_states_topic_name = this->get_parameter("joint_states_topic_name").as_string();
        RCLCPP_INFO(this->get_logger(), "Joint States Topic Name : %s", joint_states_topic_name.c_str());

        js_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
            joint_states_topic_name, 10, std::bind(&MirobotWriteNode::joint_state_callback, this, std::placeholders::_1));
    }

    void homing() {
        std::vector<uint8_t> HomingGcode = {'$', 'H', '\r', '\n'};

        // Send the G-code command to the serial port
        _serial.send(HomingGcode);

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

        RCLCPP_INFO(this->get_logger(), "Wait for seconds, Mirobot is Homing now...");
    }

private:
    void joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg) {
        RCLCPP_INFO(this->get_logger(), "Received JointState message:");
        for (size_t i = 0; i < msg->name.size(); ++i) {
            RCLCPP_INFO(this->get_logger(), "Joint: %s, Position: %f", msg->name[i].c_str(), msg->position[i]);
        }
    }

    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr js_sub_;
    std::string joint_states_topic_name;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);

    auto mirobot_gcode_write_node = std::make_shared<MirobotWriteNode>();

    RCLCPP_INFO(mirobot_gcode_write_node->get_logger(), "Port has been open successfully");

    mirobot_gcode_write_node->homing();
    rclcpp::sleep_for(std::chrono::seconds(13));
    RCLCPP_WARN(mirobot_gcode_write_node->get_logger(), "Homing Done!!!");

    rclcpp::spin(mirobot_gcode_write_node);
    rclcpp::shutdown();

    return 0;
}
