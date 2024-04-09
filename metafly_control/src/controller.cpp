#include <iostream>
#include <SerialPort.h>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/u_int64.hpp"
#include "metafly_interfaces/msg/controls.hpp"
#include <vector>
#include <chrono> // Include chrono header for chrono literals

// Use 'LibSerial::' instead of 'using namespace LibSerial;'
using LibSerial::SerialPort;
using LibSerial::BaudRate;
using LibSerial::CharacterSize;
using LibSerial::Parity;
using LibSerial::StopBits;
using LibSerial::FlowControl;
using namespace std::chrono_literals; // Add this line for chrono literals

class controller : public rclcpp::Node
{
public:
    controller() : Node("controller")
    {
        // Parameter description
        auto timer_frequency_des = rcl_interfaces::msg::ParameterDescriptor{};
        auto baud_rate_des = rcl_interfaces::msg::ParameterDescriptor{};

        timer_frequency_des.description = "Timer callback frequency [Hz]";
        baud_rate_des.description = "Baud rate for communication [bits/s]";

        // Declare default parameters values
        declare_parameter("timer_frequency", -1.0, timer_frequency_des);     // Hz 
        declare_parameter("baud_rate_int", -1, baud_rate_des);         // Bits per second

        // Get params - Read params from yaml file that is passed in the launch file
        timer_frequency_ = get_parameter("timer_frequency").get_parameter_value().get<double>();
        baud_rate_int_ = get_parameter("baud_rate_int").get_parameter_value().get<int>();
        baud_rate_ = integerToBaudRate(baud_rate_int_);

        // Check all params
        check_yaml_params();

        try 
        {
            serial_port_.Open("/dev/ttyUSB0"); // Adjust the port name as necessary
            serial_port_.SetBaudRate(baud_rate_); // Adjust the baud rate
            serial_port_.SetCharacterSize(CharacterSize::CHAR_SIZE_8);
            serial_port_.SetParity(Parity::PARITY_NONE);
            serial_port_.SetStopBits(StopBits::STOP_BITS_1);
            serial_port_.SetFlowControl(FlowControl::FLOW_CONTROL_NONE);

            RCLCPP_INFO(this->get_logger(), "Serial port opened successfully.");
        }
        catch (const LibSerial::OpenFailed &e) 
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to open serial port: %s", e.what());
            throw;
        } 
        catch (const std::exception &e) 
        {
            RCLCPP_ERROR(this->get_logger(), "An error occurred during serial port setup: %s", e.what());
            throw;
        }

        // Subscribers
        cmd_controls_subscriber_ = create_subscription<metafly_interfaces::msg::Controls>(
        "/cmd_controls", 10, std::bind(
            &controller::cmd_controls_callback, this,
            std::placeholders::_1));

        // Create timer to periodically send control commands
        timer_ = this->create_wall_timer(std::chrono::milliseconds(static_cast<int>(1000.0 / timer_frequency_)), std::bind(&controller::sendArduinoCommands, this));
    }

private:
    double timer_frequency_ = -1.0; // Hz
    int baud_rate_int_ = -1; // bits per second
    BaudRate baud_rate_ = LibSerial::BaudRate::BAUD_INVALID; // bits per second

    SerialPort serial_port_;
    rclcpp::TimerBase::SharedPtr timer_;
    char speed_ = 0;     // Speed value (0-100)
    char steering_ = 0;  // Steering value (-100 to 100)

    rclcpp::Subscription<metafly_interfaces::msg::Controls>::SharedPtr cmd_controls_subscriber_;

    void sendArduinoCommands()
    {
        // WRITE

        // Create Float32MultiArray message
        std::vector<char> msg;
        msg.push_back(speed_);    // Add speed value (0-100)
        msg.push_back(steering_); // Add steering value (-100 to 100)

        // Send data over serial
        std::vector<char> bytes_written(msg.size() * sizeof(char));
        std::memcpy(bytes_written.data(), msg.data(), bytes_written.size());
        std::string data_string(bytes_written.begin(), bytes_written.end());
        serial_port_.Write(data_string);

        // Debug output
        // RCLCPP_INFO(this->get_logger(), "Sent control commands: Speed=%.2f, Steering=%.2f", speed_, steering_);

        // READ

        // Wait for response
        std::vector<char> response;
        try 
        {
            // int timeout_ms = 1000/timer_frequency_; // timeout value in milliseconds
            // int timeout_ms = 1000; // timeout value in milliseconds
            int timeout_ms = 1; // timeout value in milliseconds

            char reading = 0;      // variable to store the read result

            serial_port_.ReadByte( reading, timeout_ms );
            response.push_back(reading);
            serial_port_.ReadByte( reading, timeout_ms );
            response.push_back(reading);
            serial_port_.ReadByte( reading, timeout_ms );
            response.push_back(reading);
            serial_port_.ReadByte( reading, timeout_ms );
            response.push_back(reading);

            // RCLCPP_INFO(this->get_logger(), "UNGA BUNGA: %d", reading);
        } 
        catch (const std::exception& e) 
        {
            RCLCPP_DEBUG(this->get_logger(), "Exception occurred while reading from serial port: %s", e.what());
        }

        if (response.size() == 4 && response.at(0) == '!') 
        {
            RCLCPP_INFO(this->get_logger(), "Received confirmation: %c, %d, %d", response.at(0), response.at(1), response.at(2));
        } 
        else 
        {
            RCLCPP_DEBUG(this->get_logger(), "Invalid response from Arduino");
        }
    }

    /// \brief cmd_controls topic callback
    void cmd_controls_callback(const metafly_interfaces::msg::Controls & msg)
    {
        speed_ = msg.speed;
        steering_ = msg.steering;
    }

    void check_yaml_params()
    {
        if (timer_frequency_ == -1.0 ||
            baud_rate_int_ == -1
            )
        {
            RCLCPP_ERROR(this->get_logger(), "Param timer frequency: %f", timer_frequency_);
            RCLCPP_ERROR(this->get_logger(), "Param Baud rate: %d", baud_rate_int_);
            
            throw std::runtime_error("Missing necessary parameters in diff_params.yaml!");
        }
        if (timer_frequency_ <= 0.0 ||
            baud_rate_ == LibSerial::BaudRate::BAUD_INVALID 
          )
        {
            RCLCPP_ERROR(this->get_logger(), "Param timer frequency: %f", timer_frequency_);
            RCLCPP_ERROR(this->get_logger(), "Param Baud rate: %d", baud_rate_int_);
            
            throw std::runtime_error("Incorrect params in diff_params.yaml!");
        }
    }

    // Function to convert an integer to a BaudRate enum value
    BaudRate integerToBaudRate(int baud_int)
    {
        // Check the integer value against each possible baud rate using a switch-case
        switch (baud_int)
        {
            case 50:
                return BaudRate::BAUD_50;
            case 75:
                return BaudRate::BAUD_75;
            case 110:
                return BaudRate::BAUD_110;
            case 134:
                return BaudRate::BAUD_134;
            case 150:
                return BaudRate::BAUD_150;
            case 200:
                return BaudRate::BAUD_200;
            case 300:
                return BaudRate::BAUD_300;
            case 600:
                return BaudRate::BAUD_600;
            case 1200:
                return BaudRate::BAUD_1200;
            case 1800:
                return BaudRate::BAUD_1800;
            case 2400:
                return BaudRate::BAUD_2400;
            case 4800:
                return BaudRate::BAUD_4800;
            case 9600:
                return BaudRate::BAUD_9600;
            case 19200:
                return BaudRate::BAUD_19200;
            case 38400:
                return BaudRate::BAUD_38400;
            case 57600:
                return BaudRate::BAUD_57600;
            case 115200:
                return BaudRate::BAUD_115200;
            case 230400:
                return BaudRate::BAUD_230400;
            // Add other cases for additional baud rates as needed
            default:
                return BaudRate::BAUD_INVALID; // Return invalid if no match found
        }
    }
};



int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<controller>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
