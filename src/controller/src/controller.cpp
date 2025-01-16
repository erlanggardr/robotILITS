#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "interfaces/msg/commands.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"

using namespace std::chrono_literals;

/*
      /cmd_vel
      x_cmd = [-250] to [250]
      y_cmd = [-250] to [250]
      yaw = [-180] to [180] // set point (stay after controller go back to 0)
      depth = [0] to [10] // set point (stay after controller go back to 0)
*/

/*
      axes index  action
         0        Left_Right_StickLeft
         1        Up_Down_StickLeft
         3 (buttons)        Square/X
         3        Left_Right_StickRight
         4        Up_Down_StickRight
         2        LT
         5        RT
*/

class ControllerNode : public rclcpp::Node {
 private:
   rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr sub_;
   rclcpp::Publisher<interfaces::msg::Commands>::SharedPtr pub_;
   int solenoidState = 0;
   const int X_RANGE = 250;
   const int Y_RANGE = 250;
   const int YAW_RANGE = 180;
   const int DEPTH_RANGE = 10;
   int yaw = 0;
   int depth = 0;

   // int value(int input, int min, int neutral, int max) {
   //    int adjustedValue = input - neutral; // Shift neutral to 0

   //    if (adjustedValue + neutral < min) {
   //       return min;
   //    } else if (adjustedValue + neutral > max) {
   //       return max;
   //    } else {
   //       return adjustedValue + neutral;
   //    }
   // }

   void controller_callback(const sensor_msgs::msg::Joy &msg) {
      // int x_cmd = value(static_cast<int>(msg.axes[0] * -5 + 5), 1, 5, 9);
      // int y_cmd = value(static_cast<int>(msg.axes[1] * 5 + 5), 1, 5, 9);
      // int yaw = value(static_cast<int>(msg.axes[3] * -5 + 5), 1, 5, 9);
      // int depth = value(static_cast<int>(msg.axes[4] * 5 + 5), 1, 5, 9);

      int x_cmd = msg.axes[0] * X_RANGE * -1;
      int y_cmd = msg.axes[1] * Y_RANGE;

      yaw += msg.axes[3] * YAW_RANGE * -1 * 0.05;
      if (yaw < -YAW_RANGE) {
         yaw += 360;
      } else if (yaw > YAW_RANGE) {
         yaw -= 360;
      }

      double depthInput = msg.axes[4] * 0.05;
      // go down
      if (depthInput < 0) {
         depth += 1;
         if (depth > DEPTH_RANGE) {
            depth = DEPTH_RANGE;
         }
      }
      // go up
      if (depthInput > 0) {
         depth -= 1;
         if (depth < 0) {
            depth = 0;
         }
      }

      int solenoidInput = msg.buttons[3];
      if (solenoidState == 0 && solenoidInput == 1) {
         solenoidState = 1;
      } else if (solenoidState == 1 && solenoidInput == 1) {
         solenoidState = 0;
      }

      auto cmd = interfaces::msg::Commands();
      cmd.x_cmd = x_cmd; // move left & right using left stick
      cmd.y_cmd = y_cmd; // move up & down using left stick
      cmd.depth = depth;
      cmd.yaw = yaw;
      cmd.solenoid = solenoidState;

      pub_->publish(cmd);
   }

 public:
   ControllerNode() : Node("controller_node") {
      pub_ = this->create_publisher<interfaces::msg::Commands>("cmd_vel", 10);
      sub_ = this->create_subscription<sensor_msgs::msg::Joy>("joy", 10, std::bind(&ControllerNode::controller_callback, this, std::placeholders::_1));
      RCLCPP_INFO(this->get_logger(), "Starting control_node");
   }
};

int main(int argc, char *argv[]) {
   rclcpp::init(argc, argv);
   rclcpp::spin(std::make_shared<ControllerNode>());
   rclcpp::shutdown();
   return 0;
}