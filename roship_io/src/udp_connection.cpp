#include "connection/udp_connection.hpp"

CONNECTION_NS_HEAD

using namespace std::chrono_literals;

UdpConnection::Params::Params()
{
  rx.port = 1234;
  rx.buffer_size = 1024;
  tx.host = "127.0.0.1";
  tx.port = 4321;
}

void UdpConnection::Params::declare(rclcpp::Node::SharedPtr node)
{
  node->declare_parameter("rx.port", rx.port);
  node->declare_parameter("rx.buffer_size", rx.buffer_size);
  node->declare_parameter("tx.host", tx.host);
  node->declare_parameter("tx.port", tx.port);
}

void UdpConnection::Params::update(rclcpp::Node::SharedPtr node)
{
  node->get_parameter("rx.port", rx.port);
  node->get_parameter("rx.buffer_size", rx.buffer_size);
  node->get_parameter("tx.host", tx.host);
  node->get_parameter("tx.port", tx.port);
}

UdpConnection::UdpConnection(rclcpp::Node::SharedPtr node):
  IoConnection<transport::UdpSocket>(node)
{
  params_.declare(node_ptr_);
  params_.update(node_ptr_);

  sock_ptr_.reset(
        new transport::UdpSocket(
          params_.rx.port,
          params_.rx.buffer_size)
        );
  sock_ptr_->AddCallback(std::bind(&UdpConnection::udpCallback,
                                   this , std::placeholders::_1));

  raw_pub_ = node_ptr_->create_publisher<io_interfaces::msg::RawPacket>("~/from_device", 10);

  raw_sub_ = node_ptr_->create_subscription<io_interfaces::msg::RawPacket>(
          "~/to_device", 1, std::bind(&UdpConnection::sendToDevice, this, std::placeholders::_1));

  timer_ = node_ptr_->create_wall_timer(
        1ms, std::bind(&UdpConnection::spin_once, this));

  RCLCPP_INFO(node_ptr_->get_logger(),
              "Listeing on port %i with buffer size %i", params_.rx.port,params_.rx.buffer_size);
  RCLCPP_INFO(node_ptr_->get_logger(),
              "sending message to device from topic: %s", raw_sub_->get_topic_name());

}

void UdpConnection::udpCallback(const std::vector<byte> &datagram)
{
  auto rx_time = node_ptr_->now();
  io_interfaces::msg::RawPacket::SharedPtr msg(new io_interfaces::msg::RawPacket);
  msg->header.stamp = rx_time;
  msg->data = datagram;
  raw_pub_->publish(*msg);
}

void UdpConnection::sendToDevice(const io_interfaces::msg::RawPacket msg)
{
  sock_ptr_->SendTo(params_.tx.host, params_.tx.port,msg.data);
}

void UdpConnection::spin_once()
{
  sock_ptr_->Receive();
}

CONNECTION_NS_FOOT
