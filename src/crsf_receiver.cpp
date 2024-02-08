#include "crsf_receiver.h"


CrsfReceiverNode::CrsfReceiverNode(): Node("crsf_reader_node")
{
    this->declare_parameter("device", "/dev/ttyUSB0");
    this->declare_parameter("baudrate", 425000);
    this->declare_parameter("link_stats", false);
    this->declare_parameter("receiver_rate", false);

    channels_publisher = this->create_publisher<std_msgs::msg::String>("rc/channels", 10);
    link_publisher = this->create_publisher<std_msgs::msg::String>("rc/link", 10);

    std::string device = this->get_parameter("device").as_string();
    int baudrate = this->get_parameter("baudrate").as_int();
    int rate = this->get_parameter("receiver_rate").as_int();
    int period = 1000 / rate;

    RCLCPP_INFO(this->get_logger(), "Receiver rate is %dhz (period %dms)", rate, period);
    RCLCPP_INFO(this->get_logger(), "Target serial device is: %s", device.c_str());
    RCLCPP_INFO(this->get_logger(), "Selected baudrate: %d", baudrate);

    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(period), 
        std::bind(&CrsfReceiverNode::timer_callback, this)
    );
}

void CrsfReceiverNode::timer_callback()
{
    // Do nothing now.
}

CrsfReceiverNode::~CrsfReceiverNode() {
    // Do nothing now.
}
