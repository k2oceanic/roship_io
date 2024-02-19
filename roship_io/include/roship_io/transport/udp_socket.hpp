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

//using MessageCallback = std::function<void(const std::vector<byte>&)>;

class UdpSocket : public TransportInterface {
 public:
  UdpSocket(int port, size_t buffer_size = 1024);
  ~UdpSocket();
  void SendTo(const std::string& ip, int port, const std::vector<byte>& message);
  void Receive();
  void AddCallback(const MessageCallback& callback);

 private:
  int sockfd_;
  int port_;
  size_t buffer_size_;
  struct sockaddr_in addr_;
  std::vector<MessageCallback> callbacks_;
  byte* buffer_;

};

TRANSPORT_NS_FOOT
