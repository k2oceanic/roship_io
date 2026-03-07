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

class Modbus{
public:
  Modbus();
  ~Modbus();
  void connect_tcp(const char *ip_address, int port);
  void set_slave(int id);
  void read_input_registers(int addr, int nb);
  void read_input_registers(Block & block);
  void read_registers(int addr, int nb);
  void read_registers(Block & block);
  void write_registers(int addr, int nb, const uint16_t *src);
  void write_registers(Block & block);
  void print_buffer();
  void set_response_timeout(uint32_t to_sec, uint32_t to_usec);
  uint16_t * buffer();

  static const int REGISTER_MAX_WORDS = 128;
  typedef std::shared_ptr<Modbus> Ptr;
private:
  uint16_t tab_reg_[REGISTER_MAX_WORDS];
  modbus_t *mb_;
};

MODBUS_NS_FOOT
