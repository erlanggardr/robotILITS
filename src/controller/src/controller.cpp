#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <algorithm>  

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "interfaces/msg/command.hpp"

using namespace std::chrono_literals;
using namespace std::placeholders;

#define X_CMD_MAX 250
#define X_CMD_MIN -250
#define Y_CMD_MAX 250
#define Y_CMD_MIN -250
#define YAW_CMD_MAX 180
#define YAW_CMD_MIN -180
#define DEPTH_CMD_MAX 10
#define DEPTH_CMD_MIN 0

class Publisher : public rclcpp::Node
{
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr sub;
    rclcpp::Publisher<interfaces::msg::Command>::SharedPtr pub;

    int yaw_value = 0;  
    int depth_value = 0;  

    void callback_topic(const sensor_msgs::msg::Joy &msg)
    {
        auto cmd = interfaces::msg::Command();

        int x_cmd = std::clamp(static_cast<int>(msg.axes[0] * -X_CMD_MAX), X_CMD_MIN, X_CMD_MAX);
        int y_cmd = std::clamp(static_cast<int>(msg.axes[1] * Y_CMD_MAX), Y_CMD_MIN, Y_CMD_MAX);

        if (msg.axes[3] != 0.0f) {
            yaw_value -= static_cast<int>(msg.axes[3] * 5.0f);
            if (yaw_value > YAW_CMD_MAX) yaw_value -= 360;
            if (yaw_value < YAW_CMD_MIN) yaw_value += 360;
        }

        if (msg.axes[4] != 0.0f) {
            depth_value = std::clamp(depth_value - static_cast<int>(msg.axes[4] * 1.0f), DEPTH_CMD_MIN, DEPTH_CMD_MAX);
        }

        cmd.x = x_cmd;
        cmd.y = y_cmd;
        cmd.yaw = yaw_value;
        cmd.depth = depth_value;

        pub->publish(cmd);
    }

public:
    Publisher() : Node("node")
    {
        sub = this->create_subscription<sensor_msgs::msg::Joy>(
            "joy", 10, std::bind(&Publisher::callback_topic, this, _1));

        pub = this->create_publisher<interfaces::msg::Command>("cmd_vel", 10);
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);    
    rclcpp::spin(std::make_shared<Publisher>());
    rclcpp::shutdown();
    return 0;
}