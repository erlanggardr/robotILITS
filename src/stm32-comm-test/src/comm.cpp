#include <asio.hpp>
#include <chrono>
#include <functional>
#include <iostream>
#include <memory>
#include <string>

#include "interfaces/msg/commands.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

class StmTest : public rclcpp::Node {
   rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
   rclcpp::TimerBase::SharedPtr timer_;
   asio::io_context io_;
   asio::serial_port port_;
   std::mutex port_mutex_;

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

   void comm_callback() {
      std::lock_guard<std::mutex> lock(port_mutex_);

      int dt1 = 2000;
      int dt2 = 2000;
      int dt3 = 2000;
      int dt4 = 2000;

      std::ostringstream sent_string;
      sent_string << dt1 << ',' << dt2 << ',' << dt3 << ',' << dt4;

      try {
         asio::write(port_, asio::buffer(sent_string.str()));
         RCLCPP_INFO(this->get_logger(), "Sent data to STM32: %s", sent_string.str().c_str());
      } catch (std::exception &e) {
         RCLCPP_ERROR(this->get_logger(), "Failed to send data to STM: %s", e.what());
      }
   }

 public:
   StmTest() : Node("stm_test"), port_(io_) {
      port_config();
      publisher_ = this->create_publisher<std_msgs::msg::String>("/sent_test", 10);
      timer_ = this->create_wall_timer(std::chrono::milliseconds(1000), std::bind(&StmTest::comm_callback, this));
   }
};

int main(int argc, char **argv) {
   rclcpp::init(argc, argv);
   rclcpp::spin(std::make_shared<StmTest>());
   rclcpp::shutdown();
   return 0;
}