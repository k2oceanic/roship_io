#pragma once  // Favor using this over the #ifndef, #define method

#include "connection_defs.hpp"
#include <rclcpp/rclcpp.hpp>
#include "io_connection.hpp"
#include "transport/udp_socket.hpp"
#include <io_interfaces/msg/raw_packet.hpp>

CONNECTION_NS_HEAD

/**
 * @brief ROS2 connection node wrapper for UDP transport.
 *
 * `UdpConnection` owns a `transport::UdpSocket`, bridges it to two ROS2 topics,
 * and drives the socket's polling loop via a 1 ms timer:
 *
 */
class UdpConnection : public IoConnection<transport::UdpSocket>
{
public:
  /**
   * @brief Configuration parameters for UdpConnection.
   */
  struct Params
  {
    /**
     * @brief Initialise parameters with safe localhost defaults.
     */
    Params();

    /**
     * @brief Declare all UDP parameters on the given ROS2 node.
     * @param node Shared pointer to the ROS2 node.
     */
    void declare(rclcpp::Node::SharedPtr node);

    /**
     * @brief Read current parameter values from the ROS2 node.
     *
     * Throws `std::runtime_error` if `dst_hosts` and `dst_ports` differ in length.
     *
     * @param node Shared pointer to the ROS2 node.
     */
    void update(rclcpp::Node::SharedPtr node);

    transport::UdpSocket::Params sock; ///< UDP socket configuration (port, destinations, buffer).
  };

  /**
   * @brief Construct a UdpConnection, declare/read parameters, and start the timer.
   *
   * @param node Shared pointer to the ROS2 node that owns topics and parameters.
   */
  UdpConnection(rclcpp::Node::SharedPtr node);

  /**
   * @brief Callback invoked by UdpSocket for each received datagram.
   *
   * Wraps the raw bytes in a `RawPacket` stamped with the current ROS2 time
   * and publishes it on `~/from_device`.
   *
   * @param datagram Raw received bytes.
   */
  void udpCallback(const std::vector<byte>& datagram);

  /**
   * @brief ROS2 subscription callback — forward a `RawPacket` to the device.
   *
   * @param msg Message received on `~/to_device`; its `data` field is transmitted.
   */
  void sendToDevice(const io_interfaces::msg::RawPacket msg);

  /**
   * @brief Poll the UDP socket for new datagrams (called by the internal timer).
   */
  void spin_once();

protected:
  rclcpp::TimerBase::SharedPtr          timer_;     ///< 1 ms polling timer calling `spin_once()`.
  std::shared_ptr<transport::UdpSocket> sock_ptr_;  ///< Underlying UDP socket.
  rclcpp::Publisher<io_interfaces::msg::RawPacket>::SharedPtr raw_pub_;  ///< Publishes received datagrams.
  rclcpp::Subscription<io_interfaces::msg::RawPacket>::SharedPtr raw_sub_; ///< Subscribes to outgoing packets.
  Params params_; ///< Runtime parameters.
};

CONNECTION_NS_FOOT
