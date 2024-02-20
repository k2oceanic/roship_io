#pragma once

#include "trasnport_defs.hpp"
#include "primitives.hpp"

#include <functional>

TRANSPORT_NS_HEAD

using MessageCallback = std::function<void(const std::vector<byte>&)>;

class TransportInterface {
 public:
  virtual void send(const std::vector<byte>&) = 0;
  virtual void spinOnce() = 0;
  virtual void addCallback(const MessageCallback& callback) = 0;
};

TRANSPORT_NS_FOOT
