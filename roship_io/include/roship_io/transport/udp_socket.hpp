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

class UdpSocket : public TransportInterface {
 public:
  struct Params{
    std::vector<std::string> dst_hosts;
    std::vector<long int>         dst_ports;
    int buffer_size;
    int port;
  };
  UdpSocket(Params params);
  ~UdpSocket();
  void send(const std::vector<byte>& message);
  void sendTo(const std::string& ip, int port, const std::vector<byte>& message);
  void spinOnce();
  void addCallback(const MessageCallback& callback);

 private:
  Params params_;
  int sockfd_;
  size_t buffer_size_;
  struct sockaddr_in addr_;
  std::vector<MessageCallback> callbacks_;
  byte* buffer_;

};

TRANSPORT_NS_FOOT
