#include "rclcpp/rclcpp.hpp"
#include <termios.h>
#include <fcntl.h>
#include <sys/select.h>
#include <ncurses.h>
#include <unistd.h>
#include <iostream>
#include "metafly_interfaces/msg/controls.hpp"
#include <limits.h>

using namespace std::chrono_literals;

class teleop_keyboard : public rclcpp::Node {
public:
    teleop_keyboard() : Node("teleop_keyboard") {
        // Initialize char variables
        speed_ = 0;
        steering_ = 0;

        speed_step_ = 10;
        steering_step_ = 20;

        min_speed_ = 0;
        max_speed_ = static_cast<char>(std::numeric_limits<char>::max());
        min_steering_ = static_cast<char>(std::numeric_limits<char>::min());
        max_steering_ = static_cast<char>(std::numeric_limits<char>::max());

        // Disable line buffering and character echoing
        struct termios t;
        tcgetattr(STDIN_FILENO, &t);
        t.c_lflag &= ~(ICANON | ECHO);
        tcsetattr(STDIN_FILENO, TCSANOW, &t);

        // Create a timer to check keyboard input periodically
        timer_ = create_wall_timer(1ms, std::bind(&teleop_keyboard::checkKeyboardInput, this));

        // Publishers
        cmd_controls_publisher_ = create_publisher<metafly_interfaces::msg::Controls>(
        "/cmd_controls", 10);
    }

private:

    char speed_;
    char steering_;
    char speed_step_;
    char steering_step_;
    char min_speed_;
    char max_speed_;
    char min_steering_;
    char max_steering_;
    rclcpp::TimerBase::SharedPtr timer_;

    rclcpp::Publisher<metafly_interfaces::msg::Controls>::SharedPtr cmd_controls_publisher_;

    void checkKeyboardInput() {

        // Check if a key has been pressed
        if (kbhit()) 
        {
            // Read the pressed key
            int ch = getchar();

            // Update variables based on the key pressed
            switch (ch) {
                case 'W':
                case 'w':
                    speed_ += (speed_ + speed_step_) <= max_speed_ ? speed_step_ : 0;
                    break;
                case 'S':
                case 's':
                    speed_ -= (speed_ - speed_step_) >= min_speed_ ? speed_step_ : 0;
                    break;
                case 'A':
                case 'a':
                    steering_ -= (steering_ - steering_step_) >= min_steering_ ? steering_step_ : 0;
                    break;
                case 'D':
                case 'd':
                    steering_ += (steering_ + steering_step_) <= max_steering_ ? steering_step_ : 0;
                    break;
                default:
                    break;
            }

            // Print current values of char variables
            printf("Speed: %d, Steering: %d\n", speed_, steering_);
        }

        metafly_interfaces::msg::Controls msg;

        msg.speed = speed_;
        msg.steering = steering_;

        cmd_controls_publisher_->publish(msg);
    }

    bool kbhit() 
    {
        struct timeval tv = {0, 0};
        fd_set fds;
        FD_ZERO(&fds);
        FD_SET(STDIN_FILENO, &fds);
        return select(STDIN_FILENO + 1, &fds, NULL, NULL, &tv) == 1;
    }

};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<teleop_keyboard>());
    rclcpp::shutdown();
    return 0;
}
