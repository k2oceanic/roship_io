#pragma once

#include "transport_defs.hpp"
#include "primitives.hpp"

#include <functional>

TRANSPORT_NS_HEAD

/**
 * @brief Callback type invoked when a message is received from the transport layer.
 *
 * The callback receives the raw byte payload of the incoming message.
 */
using MessageCallback = std::function<void(const std::vector<byte>&)>;

/**
 * @brief base class defining the interface for all transport implementations.
 */
class TransportInterface {
 public:
  /**
   * @brief Send a raw byte payload over the transport.
   *
   * @param message Byte vector to transmit.
   */
  virtual void send(const std::vector<byte>& message) = 0;

  /**
   * @brief Process pending I/O for transports that require polling.
   *
   * For transports that rely on a background thread (e.g. `LspSerial`) this is
   * not used .  For polling transports (e.g. `UdpSocket`).
   */
  virtual void spinOnce() = 0;

  /**
   * @brief Register a callback to be invoked when a message is received.
   *
   * @param callback Function to call with the raw received bytes.
   */
  virtual void addCallback(const MessageCallback& callback) = 0;
};

TRANSPORT_NS_FOOT
