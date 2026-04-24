#pragma once

#include "modbus_primitives.hpp"
#include "blocks_defs.hpp"

#include <stddef.h>
#include <memory>
#include <sstream>
#include <iomanip>

MODBUS_NS_HEAD

/**
 * @brief Abstract base class representing a block of Modbus registers.
 *
 */
class Block {
public:
  /** @brief Convenience shared_ptr alias. */
  std::shared_ptr<Block> SharedPtr;

  /**
   * @brief Serialise the block and return a pointer to the word buffer.
   *
   * @return Pointer to the `uint16_t` register data.
   */
  virtual primitives::WordBuffer buffer() = 0;

  /**
   * @brief Return the size of this block in Modbus words (16-bit registers).
   *
   * @return Number of 16-bit words occupied by this block.
   */
  virtual size_t size() = 0;

  /**
   * @brief Return the starting Modbus register address for this block.
   *
   * @return Modbus register address (0-based).
   */
  virtual int modbusAddress() = 0;

  /**
   * @brief Return the buffer contents as a comma-separated hex string.
   *    
   * @return String representation such as `"0x0001, 0x00FF, 0x0100"`.
   */
  virtual std::string hexString() {
    std::stringstream ss;
    for (size_t i = 0; i < size(); ++i) {
      ss << "0x" << std::hex << std::uppercase << std::setfill('0') << std::setw(4) << buffer()[i];
      if (i < size() - 1) {
        ss << ", ";
      }
    }
    return ss.str();
  }
};

MODBUS_NS_FOOT
