#pragma once

#include "modbus_defs.hpp"
#include "blocks/block.hpp"

#include <modbus.h>
#include <cerrno>
#include <stdexcept>
#include <iomanip>
#include <iostream>
#include <memory>

MODBUS_NS_HEAD

/**
 * @brief C++ wrapper around the libmodbus TCP client.
 */
class Modbus {
public:
  /**
   * @brief Default constructor. Allocates the libmodbus context.
   */
  Modbus();

  /**
   * @brief Destructor. Closes the connection and frees the libmodbus context.
   */
  ~Modbus();

  /**
   * @brief Establish a TCP/IP connection to a Modbus server.
   *
   * Throws `std::runtime_error` if the connection cannot be established.
   *
   * @param ip_address Server IP address string (e.g. `"192.168.1.10"`).
   * @param port       TCP port (standard Modbus port is 502).
   */
  void connect_tcp(const char* ip_address, int port);

  /**
   * @brief Set the Modbus slave/unit ID for subsequent requests.
   *
   * @param id Slave identifier (1–247).
   */
  void set_slave(int id);

  /**
   * @brief Read `nb` input registers starting at `addr` into the internal buffer.
   *
   * Input registers are read-only (Modbus function code 0x04).
   *
   * @param addr Starting register address.
   * @param nb   Number of registers to read (must be ≤ `REGISTER_MAX_WORDS`).
   */
  void read_input_registers(int addr, int nb);

  /**
   * @brief Read input registers described by `block` into the internal buffer.
   *
   * Uses `block.modbusAddress()` and `block.size()` to determine address and count,
   * then populates `block` from the returned register values.
   *
   * @param block Block describing the register range to read.
   */
  void read_input_registers(Block& block);

  /**
   * @brief Read `nb` holding registers starting at `addr` into the internal buffer.
   *
   * Holding registers support read/write (Modbus function code 0x03).
   *
   * @param addr Starting register address.
   * @param nb   Number of registers to read (must be ≤ `REGISTER_MAX_WORDS`).
   */
  void read_registers(int addr, int nb);

  /**
   * @brief Read holding registers described by `block` into the internal buffer.
   *
   * @param block Block describing the register range to read.
   */
  void read_registers(Block& block);

  /**
   * @brief Write `nb` holding registers starting at `addr` from `src`.
   *
   * @param addr Starting register address.
   * @param nb   Number of registers to write.
   * @param src  Source array of `uint16_t` words (must contain at least `nb` elements).
   */
  void write_registers(int addr, int nb, const uint16_t* src);

  /**
   * @brief Write holding registers described by `block` to the device.
   *
   * Serialises `block` to its word buffer and writes starting at `block.modbusAddress()`.
   *
   * @param block Block containing the register data and address to write.
   */
  void write_registers(Block& block);

  /**
   * @brief Print the internal register buffer as a hex dump to stdout.
   *
   * Useful during development to inspect raw register values.
   */
  void print_buffer();

  /**
   * @brief Configure the response timeout for Modbus requests.
   *
   * @param to_sec  Timeout seconds component.
   * @param to_usec Timeout microseconds component.
   */
  void set_response_timeout(uint32_t to_sec, uint32_t to_usec);

  /**
   * @brief Return a pointer to the internal register buffer.
   *
   * Valid between calls; contains the most recently read register values.
   *
   * @return Pointer to the `uint16_t` register buffer.
   */
  uint16_t* buffer();

  /** @brief Maximum number of registers that can be read in a single request. */
  static const int REGISTER_MAX_WORDS = 128;

  /** @brief Convenience shared_ptr alias. */
  typedef std::shared_ptr<Modbus> Ptr;

private:
  uint16_t tab_reg_[REGISTER_MAX_WORDS]; ///< Internal register read/write buffer.
  modbus_t* mb_;                         ///< libmodbus context handle.
};

MODBUS_NS_FOOT
