#include <asio.hpp>
#include <chrono>
#include <functional>
#include <iostream>
#include <memory>
#include <string>

#include "interfaces/msg/commands.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

class StmComm : public rclcpp::Node {
   rclcpp::Subscription<interfaces::msg::Commands>::SharedPtr subscriber_;
   asio::io_context io_;
   asio::serial_port port_;

   void port_config() {
      const std::string PORT = "/dev/ttyACM0";
      try {
         port_.open(PORT);
         port_.set_option(asio::serial_port_base::flow_control(asio::serial_port_base::flow_control::software));
         port_.set_option(asio::serial_port_base::parity(asio::serial_port_base::parity::none));
         port_.set_option(asio::serial_port_base::stop_bits(asio::serial_port_base::stop_bits::one));
         port_.set_option(asio::serial_port_base::character_size(8));
         port_.set_option(asio::serial_port_base::baud_rate(115200));
         port_.set_option(asio::serial_port_base::flow_control(asio::serial_port_base::flow_control::software));
         port_.set_option(asio::serial_port_base::parity(asio::serial_port_base::parity::none));
         port_.set_option(asio::serial_port_base::stop_bits(asio::serial_port_base::stop_bits::one));
         port_.set_option(asio::serial_port_base::character_size(8));
      } catch (std::exception &e) {
         RCLCPP_ERROR(this->get_logger(), "Failed to open serial port: %s", e.what());
      }
   }

   void communication_callback(const interfaces::msg::Commands &msg) {
      std::ostringstream oss;
      oss << msg.x_cmd << msg.y_cmd << msg.yaw << msg.depth << msg.solenoid;

      try {
         // asio::string buf = oss;
         asio::write(port_, asio::buffer(oss.str()));
         RCLCPP_INFO(this->get_logger(), "Sent data to STM32: %s", oss.str().c_str());
      } catch (std::exception &e) {
         RCLCPP_ERROR(this->get_logger(), "Failed to send data to STM: %s", e.what());
      }
   }

 public:
   StmComm() : Node("stm_comm"), port_(io_) {
      port_config();
      subscriber_ = this->create_subscription<interfaces::msg::Commands>("cmd_vel", 10, std::bind(&StmComm::communication_callback, this, std::placeholders::_1));
   }
};

int main(int argc, char **argv) {
   rclcpp::init(argc, argv);
   rclcpp::spin(std::make_shared<StmComm>());
   rclcpp::shutdown();
   return 0;
}
