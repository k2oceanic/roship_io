#pragma once

#include "trasnport_defs.hpp"
#include "primitives.hpp"

#include <functional>

TRANSPORT_NS_HEAD

using MessageCallback = std::function<void(const std::vector<byte>&)>;

class TransportInterface {
 public:
  virtual void Receive() = 0;
  virtual void AddCallback(const MessageCallback& callback) = 0;
};

TRANSPORT_NS_FOOT
