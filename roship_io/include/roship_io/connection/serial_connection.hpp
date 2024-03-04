#pragma once  

#include "connection_defs.hpp" 
#include <rclcpp/rclcpp.hpp>
#include "io_connection.hpp"
#include "transport/lsp_serial.hpp"

#include <io_interfaces/msg/raw_packet.hpp>

CONNECTION_NS_HEAD

/**
 * @class SerialConnection
 * @brief A class for serial communication in a ROS2 context.
 *
 * This class handles serial communication within a ROS2 node, using the LspSerial class for the actual
 * serial communication. It subscribes to messages to be sent to the device and publishes messages received
 * from the device.
 */
class SerialConnection : public IoConnection<transport::LspSerial>
{
public:
  /**
   * @struct Params
   * @brief Configuration parameters for the SerialConnection.
   */
  struct Params
  {
    Params();
    void declare(rclcpp::Node::SharedPtr node);
    void update(rclcpp::Node::SharedPtr node);
    transport::LspSerial::Params serial;
  };

  /**
   * @brief Constructor for SerialConnection.
   * @param node Shared pointer to the ROS2 node.
   */
  SerialConnection(rclcpp::Node::SharedPtr node);

  void serialCallback(const std::vector<byte>& datagram);
  void sendToDevice(const io_interfaces::msg::RawPacket msg);
  void spin_once();


protected:
  rclcpp::TimerBase::SharedPtr          timer_;
  std::shared_ptr<transport::LspSerial> serial_ptr_;
  rclcpp::Publisher<io_interfaces::msg::RawPacket>::SharedPtr raw_pub_;
  rclcpp::Subscription<io_interfaces::msg::RawPacket>::SharedPtr raw_sub_;
  Params params_;

};

CONNECTION_NS_FOOT
