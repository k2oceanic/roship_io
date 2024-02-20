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
}

void SerialConnection::Params::update(rclcpp::Node::SharedPtr node)
{
  node->get_parameter("serial.port", serial.port);
  node->get_parameter("serial.baud_rate", serial.baud_rate);
  node->get_parameter("serial.buffer_size", serial.buffer_size);
  node->get_parameter("serial.character_size", serial.character_size);
}

SerialConnection::SerialConnection(rclcpp::Node::SharedPtr node):
  IoConnection<transport::AsioSerial>(node)
{
  params_.declare(node_ptr_);
  params_.update(node_ptr_);

  serial_ptr_.reset(
        new transport::AsioSerial(params_.serial)
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
              "sending message to device from topic: %s", raw_sub_->get_topic_name());

}

void SerialConnection::serialCallback(const std::vector<byte> &datagram)
{
  auto rx_time = node_ptr_->now();
  io_interfaces::msg::RawPacket::SharedPtr msg(new io_interfaces::msg::RawPacket);
  msg->header.stamp = rx_time;
  msg->data = datagram;
  raw_pub_->publish(*msg);
}

void SerialConnection::sendToDevice(const io_interfaces::msg::RawPacket msg)
{
  serial_ptr_->send(msg.data);
}

void SerialConnection::spin_once()
{
  serial_ptr_->spinOnce();
}

CONNECTION_NS_FOOT
