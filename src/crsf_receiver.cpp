#include "crsf_receiver.h"


CrsfReceiverNode::CrsfReceiverNode(): Node("crsf_reader_node")
{
    this->declare_parameter("device", "/dev/ttyUSB0");
    this->declare_parameter("baudrate", CRSF_BAUDRATE);
    this->declare_parameter("link_stats", false);
    this->declare_parameter("receiver_rate", false);

    channels_publisher = this->create_publisher<std_msgs::msg::Int32MultiArray>("rc/channels", 10);
    link_publisher = this->create_publisher<std_msgs::msg::String>("rc/link", 10);

    device = this->get_parameter("device").as_string();
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

    received_buffer.reserve(CRSF_FRAME_SIZE_MAX * 10);

    serial.SetDevice(device);
    serial.SetBaudRate(baudrate);
    serial.SetTimeout(period / 2);
}

void CrsfReceiverNode::timer_callback()
{
    if(serial.GetState() == CppLinuxSerial::State::CLOSED) {
        try {
            serial.Open();
        } catch(const CppLinuxSerial::Exception& e) {
            RCLCPP_WARN(this->get_logger(), "Can not open serial port: %s", device);
            return;
        }
    }

    if(serial.Available())
    {
        serial.ReadBinary(received_buffer);
        for (uint8_t b: received_buffer) {
            parser.parse_incoming_byte(b);
        }
        received_buffer.clear();
    }
}

CrsfReceiverNode::~CrsfReceiverNode() {
    if(serial.GetState() == CppLinuxSerial::State::OPEN) {
        serial.Close();
    }
}
