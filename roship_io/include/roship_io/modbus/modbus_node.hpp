#pragma once  // Favor using this over the #ifndef, #define method

#include "package_defs.hpp"  //  This is where we include all our namespace stuff for the package
#include "modbus.hpp"

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

MODBUS_NS_HEAD  // macro for consistantly defining our namespace for the package

/**
 * @brief Base ROS2 node for polling a Modbus TCP device.                 |
 */
class ModbusNode : public rclcpp::Node
{
public:
  /**
   * @brief All configurable parameters for the ModbusNode.
   */
  struct Params {
    /**
     * @brief Timer interval parameters.
     */
    struct {
      int poll_ms    = 500;  ///< Polling interval in milliseconds.
      int connect_ms = 1000; ///< Re-connection attempt interval in milliseconds.
      int write_ms   = 10;   ///< Write operation interval in milliseconds.
    } timers;

    /**
     * @brief TCP connection parameters.
     */
    struct {
      std::string type            = "tcp";             ///< Connection type (currently only \"tcp\" is supported).
      std::string ip              = "192.168.52.209";  ///< Target device IP address.
      int network_port            = 502;               ///< Modbus TCP port.
      int response_timeout_ms     = 500;               ///< Modbus response timeout in milliseconds.
      int slave_id                = 1;                 ///< Modbus slave/unit ID (1–247).
    } connection;

    /**
     * @brief Declare all parameters on the given ROS2 node.
     * @param node Pointer to the ROS2 node.
     */
    void declare(rclcpp::Node* node);

    /**
     * @brief Read current parameter values from the ROS2 node.
     * @param node Pointer to the ROS2 node.
     */
    void update(rclcpp::Node* node);
  };

  /**
   * @brief Construct a ModbusNode with the given node name.
   *
   * Declares and reads parameters, starts the poll and connect timers, and
   * attempts an initial connection.
   *
   * @param name ROS2 node name (default: `"modbus_connection"`).
   */
  ModbusNode(std::string name = "modbus_connection");

  /**
   * @brief Read input registers from the device into `block`.
   *
   * Calls `modbus_.read_input_registers(block)` and returns the result.
   * On failure, logs a warning and sets `connected_` to false.
   *
   * @param block Block describing the register range and buffer to fill.
   * @return `true` on success, `false` if the read failed.
   */
  bool readInputRegisters(Block& block);

  /**
   * @brief Read holding registers from the device into `block`.
   *
   * @param block Block describing the register range and buffer to fill.
   * @return `true` on success, `false` if the read failed.
   */
  bool readRegisters(Block& block);

  /**
   * @brief Write holding registers from `block` to the device.
   *
   * @param block Block containing the data and address to write.
   * @return `true` on success, `false` if the write failed.
   */
  bool writeRegisters(Block& block);

protected:
  /**
   * @brief Poll callback invoked on each `timers.poll_ms` tick.
   *
   */
  virtual void onPoll() {}

  roship_io::modbus::Modbus modbus_;         ///< Underlying Modbus TCP client.
  rclcpp::TimerBase::SharedPtr timer_;       ///< Periodic poll timer.
  rclcpp::TimerBase::SharedPtr connect_timer_; ///< Periodic reconnect timer.

  /**
   * @brief Attempt to connect to the Modbus device.
   *
   * Reads `params_.connection` settings and calls `modbus_.connect_tcp()`.
   * If already connected, returns immediately.
   */
  void connect();

  bool connected_ = false; ///< True while the TCP connection to the device is active.
  Params params_;          ///< Runtime parameters.

private:
  /**
   * @brief Internal timer callback. Calls `onPoll()` when connected.
   */
  void timer_callback();
};

MODBUS_NS_FOOT
