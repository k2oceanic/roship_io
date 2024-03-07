#include "connection/serial_connection.hpp"

CONNECTION_NS_HEAD

using namespace std::chrono_literals;

SerialConnection::Params::Params()
{
  // Defaults defined in asio_serial.h
}

void SerialConnection::Params::declare(rclcpp::Node::SharedPtr node)
{
  node->declare_parameter("serial.port", serial.port);
  node->declare_parameter("serial.baud_rate", serial.baud_rate);
  node->declare_parameter("serial.buffer_size", serial.buffer_size);
  node->declare_parameter("serial.character_size", serial.character_size);
  node->declare_parameter("serial.stop_bits", serial.stop_bits);
  node->declare_parameter("serial.read_timeout_ms", serial.read_timeout_ms);
  node->declare_parameter("serial.end_of_frame_byte", serial.end_of_frame_byte);
  node->declare_parameter("serial.end_of_frame_ascii", serial.end_of_frame_ascii);
  node->declare_parameter("serial.use_end_of_frame_byte", serial.use_end_of_frame_byte);
  node->declare_parameter("serial.use_end_of_frame_ascii", serial.use_end_of_frame_ascii);
}

void SerialConnection::Params::update(rclcpp::Node::SharedPtr node)
{
  node->get_parameter("serial.port", serial.port);
  node->get_parameter("serial.baud_rate", serial.baud_rate);
  node->get_parameter("serial.buffer_size", serial.buffer_size);
  node->get_parameter("serial.character_size", serial.character_size);
  node->get_parameter("serial.stop_bits", serial.stop_bits);
  node->get_parameter("serial.read_timeout_ms", serial.read_timeout_ms);
  node->get_parameter("serial.end_of_frame_byte", serial.end_of_frame_byte);
  node->get_parameter("serial.end_of_frame_ascii", serial.end_of_frame_ascii);
  node->get_parameter("serial.use_end_of_frame_byte", serial.use_end_of_frame_byte);
  node->get_parameter("serial.use_end_of_frame_ascii", serial.use_end_of_frame_ascii);
}

SerialConnection::SerialConnection(rclcpp::Node::SharedPtr node):
  IoConnection<transport::LspSerial>(node)
{
  params_.declare(node_ptr_);
  params_.update(node_ptr_);

  RCLCPP_INFO(node_ptr_->get_logger(),
              "SerialConnection::SerialConnection - Serial port: %s, Baud rate: %d, Buffer size: %d, Character size: %d, Stop bits: %d",
              params_.serial.port.c_str(),
              params_.serial.baud_rate,
              params_.serial.buffer_size,
              params_.serial.character_size,
              params_.serial.stop_bits);

  serial_ptr_.reset(
        new transport::LspSerial(params_.serial)
        );
  serial_ptr_->addCallback(std::bind(&SerialConnection::serialCallback,
                                   this , std::placeholders::_1));

  raw_pub_ = node_ptr_->create_publisher<io_interfaces::msg::RawPacket>("~/from_device", 10);

  raw_sub_ = node_ptr_->create_subscription<io_interfaces::msg::RawPacket>(
          "~/to_device", 1, std::bind(&SerialConnection::sendToDevice, this, std::placeholders::_1));

  timer_ = node_ptr_->create_wall_timer(
        1ms, std::bind(&SerialConnection::spin_once, this));

  RCLCPP_INFO(node_ptr_->get_logger(),
              "connecting to port %s with buffer size %i", params_.serial.port.c_str(),params_.serial.buffer_size);
  RCLCPP_INFO(node_ptr_->get_logger(),
              "sending messages to device from topic: %s", raw_sub_->get_topic_name());
}

void SerialConnection::serialCallback(const std::vector<byte> &datagram)
{
  auto rx_time = node_ptr_->now();
  io_interfaces::msg::RawPacket::SharedPtr msg(new io_interfaces::msg::RawPacket);
  msg->header.stamp = rx_time;
  msg->data = datagram;
  raw_pub_->publish(*msg);
  RCLCPP_DEBUG(node_ptr_->get_logger(),
               "SerialConnection::serialCallback - Received data of size: %zu", datagram.size());
}

void SerialConnection::sendToDevice(const io_interfaces::msg::RawPacket msg)
{
  serial_ptr_->send(msg.data);
  RCLCPP_DEBUG(node_ptr_->get_logger(),
               "SerialConnection::sendToDevice - Sent data of size: %zu", msg.data.size());
}

void SerialConnection::spin_once()
{
  /*serial_ptr_->spinOnce();
  RCLCPP_DEBUG(node_ptr_->get_logger(),
               "SerialConnection::spin_once - Called spinOnce");*/
}

CONNECTION_NS_FOOT
