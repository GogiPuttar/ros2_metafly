#include "rclcpp/rclcpp.hpp"
#include <unistd.h>
#include <fcntl.h>
#include <linux/joystick.h>
#include "metafly_interfaces/msg/controls.hpp"

class teleop_ps3 : public rclcpp::Node {
public:
    teleop_ps3() : Node("teleop_ps3") 
    {
        // Initialize char variables
        speed_ = 0;
        steering_ = 0;

        min_speed_ = 0;
        max_speed_ = static_cast<char>(std::numeric_limits<char>::max());
        min_steering_ = static_cast<char>(std::numeric_limits<char>::min());
        max_steering_ = static_cast<char>(std::numeric_limits<char>::max());

        safe_ = true;

        // Publishers
        cmd_controls_publisher_ = create_publisher<metafly_interfaces::msg::Controls>(
        "/cmd_controls", 10);
        timer_ = this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&teleop_ps3::publishJoystickValues, this));
    }

private:

    char speed_;
    char steering_;
    char min_speed_;
    char max_speed_;
    char min_steering_;
    char max_steering_;
    bool safe_;

    rclcpp::Publisher<metafly_interfaces::msg::Controls>::SharedPtr cmd_controls_publisher_;
    rclcpp::TimerBase::SharedPtr timer_;

    void publishJoystickValues() 
    {
        RCLCPP_INFO(this->get_logger(), "Publishing joystick values");

        // Open the joystick device
        int joy_fd = open("/dev/input/js0", O_RDONLY);
        if (joy_fd < 0) 
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to open joystick device: %s", strerror(errno));
            return;
        }
        RCLCPP_INFO(this->get_logger(), "Joystick device opened successfully");

        // Initialize joystick event
        struct js_event js;

        // Initialize joystick message
        std::vector<double> js_vals;

        for (int i=0; i<4; i++)
        {
            js_vals.push_back(0);
        }

        // Initialize metafly controls message
        metafly_interfaces::msg::Controls msg;

        // Joystick closing needs to be handled correctly, hence the infinite time loop
        while (true) 
        {
            // Read joystick event
            int bytes_read = read(joy_fd, &js, sizeof(js));

            // Handle improper reading
            if (bytes_read < 0) 
            {
                RCLCPP_ERROR(this->get_logger(), "Error reading joystick event: %s", strerror(errno));
                break;
            } 
            else if (bytes_read == 0) 
            {
                RCLCPP_WARN(this->get_logger(), "No data read from joystick device");
                break;
            } 
            else if (bytes_read != sizeof(js)) 
            {
                RCLCPP_ERROR(this->get_logger(), "Incomplete joystick event read");
                continue;
            }

            // If joysticks of the PS3 controller have been moved
            if (js.type & JS_EVENT_AXIS) 
            {
                // Handle joystick axes
                if (js.number <= 4) 
                {
                    if (js.number == 0 || js.number == 1)
                    {
                        js_vals[js.number] = static_cast<float>(js.value) / 32767.0;
                    }
                    else if (js.number == 3 || js.number == 4)
                    {
                        js_vals[js.number-1] = static_cast<float>(js.value) / 32767.0;
                    }
                } 
                else 
                {
                    RCLCPP_ERROR(this->get_logger(), "Axis number out of range: %d", js.number);
                    safe_ = false;
                }
            } 

            if(safe_)
            {
                // Convert joystick values to control inputs
                msg.speed = -js_vals[1] >= 0 ? static_cast<char>(-js_vals[1] * static_cast<double>(max_speed_ - min_speed_)) : 0;
                msg.steering = static_cast<char>((js_vals[2]) * static_cast<double>(max_steering_ - min_steering_) / 2.0);

                cmd_controls_publisher_->publish(msg);

                RCLCPP_DEBUG(this->get_logger(), "Joystick message published: time: %d, val: %d, type: %d, number: %d", js.time, js.value, js.type, js.number);
            }
            else
            {
                msg.speed = 0;
                msg.steering = 0;

                cmd_controls_publisher_->publish(msg);

                RCLCPP_DEBUG(this->get_logger(), "PREVENTED MOTION");
            }
            safe_ = true;
        }

        close(joy_fd);
        RCLCPP_INFO(this->get_logger(), "Joystick device closed");
    }

};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<teleop_ps3>());
    rclcpp::shutdown();
    return 0;
}
