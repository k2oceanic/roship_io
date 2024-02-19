#pragma once  // Favor using this over the #ifndef, #define method


// First include your local package stuff
#include "connection_defs.hpp"  //  This is where we include all our namespace stuff for the package
#include <rclcpp/rclcpp.hpp>
#include "io_connection.hpp"
#include "transport/udp_socket.hpp"

#include <io_interfaces/msg/raw_packet.hpp>


CONNECTION_NS_HEAD

class UdpConnection : public IoConnection<transport::UdpSocket>
{
public:
  struct Params
  {
    Params();
    void declare(rclcpp::Node::SharedPtr node);
    void update(rclcpp::Node::SharedPtr node);
    struct{
      int port;
      std::string host;
    } tx;
    struct{
      int port;
      int buffer_size;
    } rx;

  };

  UdpConnection(rclcpp::Node::SharedPtr node);

  void udpCallback(const std::vector<byte>& datagram);

  void sendToDevice(const io_interfaces::msg::RawPacket msg);

  void spin_once();


protected:
  rclcpp::TimerBase::SharedPtr          timer_;
  std::shared_ptr<transport::UdpSocket> sock_ptr_;
  rclcpp::Publisher<io_interfaces::msg::RawPacket>::SharedPtr raw_pub_;
  rclcpp::Subscription<io_interfaces::msg::RawPacket>::SharedPtr raw_sub_;
  Params params_;

};

CONNECTION_NS_FOOT
