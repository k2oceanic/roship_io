#pragma once

#include "transport_interface.hpp"

#include <boost/asio.hpp>
#include <vector>
#include <functional>
#include <thread>
#include <iostream>

TRANSPORT_NS_HEAD

class AsioSerial : public TransportInterface {
public:
  struct Params{
    std::string port = "/dev/ttyUSB0";
    int baud_rate = 9600;
    int buffer_size = 1024;
    int character_size = 8;
  };
  AsioSerial(Params params);
  ~AsioSerial();

  // TransportInterface

  void send(const std::vector<byte>& message) override;
  void spinOnce() override;
  void addCallback(const MessageCallback& callback) override;

private:
  boost::asio::io_service io_service_;
  boost::asio::serial_port serial_port_;
  MessageCallback message_callback_;
  std::shared_ptr<std::thread> io_thread_;
  Params params_;
  bool is_reading_ = false;

  std::vector<byte> read_buffer_;
  void doRead();
  void readHandler(const boost::system::error_code& ec, std::size_t bytes_transferred);
};
TRANSPORT_NS_FOOT
