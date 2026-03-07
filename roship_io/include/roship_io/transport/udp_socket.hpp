#pragma once

#include "transport_interface.hpp"

#include <string>
#include <vector>
#include <functional>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <cstring>
#include <stdexcept>
#include <cstring>  // For strerror
#include <sstream>  // For std::ostringstream
#include <fcntl.h>  // For fcntl

TRANSPORT_NS_HEAD

/**
 * @brief Non-blocking UDP transport implementing TransportInterface.
 *
 */
class UdpSocket : public TransportInterface {
 public:
  /**
   * @brief Configuration parameters for UdpSocket.
   */
  struct Params {
    std::vector<std::string> dst_hosts; ///< Destination IP addresses for outgoing datagrams.
    std::vector<long int>    dst_ports; ///< Destination ports, one per entry in `dst_hosts`.
    int buffer_size;                    ///< Size of the receive buffer in bytes.
    int port;                           ///< Local UDP port to bind and listen on.
  };

  /**
   * @brief Construct and bind a non-blocking UDP socket.
   *
   * @param params Socket configuration (bind port, destinations, buffer size).
   */
  UdpSocket(Params params);

  /**
   * @brief Destructor. Closes the socket and releases the receive buffer.
   */
  ~UdpSocket();

  /**
   * @brief Send a datagram to all configured destination hosts/ports.
   *
   * @param message Raw bytes to transmit.
   */
  void send(const std::vector<byte>& message) override;

  /**
   * @brief Send a datagram to an explicit IP/port.
   *
   * @param ip   Destination IP address string (e.g. `"192.168.1.10"`).
   * @param port Destination UDP port.
   * @param message Raw bytes to transmit.
   */
  void sendTo(const std::string& ip, int port, const std::vector<byte>& message);

  /**
   * @brief Read one pending datagram and invoke all registered callbacks.
   */
  void spinOnce() override;

  /**
   * @brief Register a callback invoked for each received datagram.
   *
   * @param callback Function called with the raw datagram bytes.
   */
  void addCallback(const MessageCallback& callback) override;

 private:
  Params params_;                        ///< Socket configuration.
  int sockfd_;                           ///< File descriptor for the bound UDP socket.
  size_t buffer_size_;                   ///< Receive buffer size in bytes.
  struct sockaddr_in addr_;              ///< Local address struct used for binding.
  std::vector<MessageCallback> callbacks_; ///< Registered receive callbacks.
  byte* buffer_;                         ///< Heap-allocated receive buffer.
};

TRANSPORT_NS_FOOT
