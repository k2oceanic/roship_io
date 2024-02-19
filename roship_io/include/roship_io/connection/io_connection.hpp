#pragma once  // Favor using this over the #ifndef, #define method


// First include your local package stuff
#include "connection_defs.hpp"  //  This is where we include all our namespace stuff for the package

#include "transport/transport_interface.hpp"

#include <rclcpp/rclcpp.hpp>



CONNECTION_NS_HEAD
template <typename transport_T>
class IoConnection
{
public:
  typedef std::shared_ptr<IoConnection> SharedPtr;
  struct Params{
    virtual void declare(rclcpp::Node::SharedPtr node){return;}
    virtual void update(rclcpp::Node::SharedPtr node){return;}
  };

  IoConnection(rclcpp::Node::SharedPtr node): node_ptr_(node){return;}
protected:
  rclcpp::Node::SharedPtr       node_ptr_;
  std::shared_ptr<transport_T>  trasnport_ptr_;
};

CONNECTION_NS_FOOT
