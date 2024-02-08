#ifndef CRSF_RECEIVER_HPP
#define CRSF_RECEIVER_HPP

#include <chrono>
#include <functional>

#include "std_msgs/msg/string.hpp"
#include "rclcpp/rclcpp.hpp"

// #include "crsf_receiver/msg/CRSFChannels16.hpp"  


using namespace std::chrono_literals;


class CrsfReceiverNode : public rclcpp::Node
{
public:
  explicit CrsfReceiverNode();
  ~CrsfReceiverNode();

  void timer_callback();

private:
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr channels_publisher; // crsf_receiver::msg::CRSFChannels16
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr link_publisher;
};



#endif 