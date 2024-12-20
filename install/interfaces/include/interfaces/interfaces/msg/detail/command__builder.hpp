// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from interfaces:msg/Command.idl
// generated code does not contain a copyright notice

#ifndef INTERFACES__MSG__DETAIL__COMMAND__BUILDER_HPP_
#define INTERFACES__MSG__DETAIL__COMMAND__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "interfaces/msg/detail/command__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace interfaces
{

namespace msg
{

namespace builder
{

class Init_Command_depth
{
public:
  explicit Init_Command_depth(::interfaces::msg::Command & msg)
  : msg_(msg)
  {}
  ::interfaces::msg::Command depth(::interfaces::msg::Command::_depth_type arg)
  {
    msg_.depth = std::move(arg);
    return std::move(msg_);
  }

private:
  ::interfaces::msg::Command msg_;
};

class Init_Command_yaw
{
public:
  explicit Init_Command_yaw(::interfaces::msg::Command & msg)
  : msg_(msg)
  {}
  Init_Command_depth yaw(::interfaces::msg::Command::_yaw_type arg)
  {
    msg_.yaw = std::move(arg);
    return Init_Command_depth(msg_);
  }

private:
  ::interfaces::msg::Command msg_;
};

class Init_Command_y
{
public:
  explicit Init_Command_y(::interfaces::msg::Command & msg)
  : msg_(msg)
  {}
  Init_Command_yaw y(::interfaces::msg::Command::_y_type arg)
  {
    msg_.y = std::move(arg);
    return Init_Command_yaw(msg_);
  }

private:
  ::interfaces::msg::Command msg_;
};

class Init_Command_x
{
public:
  Init_Command_x()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Command_y x(::interfaces::msg::Command::_x_type arg)
  {
    msg_.x = std::move(arg);
    return Init_Command_y(msg_);
  }

private:
  ::interfaces::msg::Command msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::interfaces::msg::Command>()
{
  return interfaces::msg::builder::Init_Command_x();
}

}  // namespace interfaces

#endif  // INTERFACES__MSG__DETAIL__COMMAND__BUILDER_HPP_
