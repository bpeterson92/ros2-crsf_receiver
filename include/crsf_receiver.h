#ifndef CRSF_RECEIVER_HPP
#define CRSF_RECEIVER_HPP

#include <chrono>
#include <functional>
#include <vector>

#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/int32_multi_array.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/qos.hpp"
#include <CppLinuxSerial/SerialPort.hpp>

#include "crsf_parser.h"


using namespace std::chrono_literals;
using namespace mn;


class CrsfReceiverNode : public rclcpp::Node
{
public:
  explicit CrsfReceiverNode();
  ~CrsfReceiverNode();

  void timer_callback();

private:
  CppLinuxSerial::SerialPort serial;
  CrsfParser parser;

  std::string device;
  vector<uint8_t> received_buffer;
  rclcpp::TimerBase::SharedPtr timer_;

  rclcpp::Publisher<std_msgs::msg::Int32MultiArray>::SharedPtr channels_publisher; // crsf_receiver::msg::CRSFChannels16
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr link_publisher;
};


#endif 