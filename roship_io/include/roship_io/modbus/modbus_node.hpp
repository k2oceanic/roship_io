#pragma once  // Favor using this over the #ifndef, #define method


// First include your local package stuff
#include "package_defs.hpp"  //  This is where we include all our namespace stuff for the package

// then include external libary stuff
#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

#include "modbus.hpp"



MODBUS_NS_HEAD  // macro for consistantly defining our namespace for the package

class ModbusNode : public rclcpp::Node
{
public:
  struct Params{
    struct{
      int poll_ms = 500;
      int connect_ms = 1000;
      int write_ms = 10;
    }timers;
    struct{
      std::string type = "tcp";
      std::string ip   = "192.168.52.209";
      int network_port = 502;
      int response_timeout_ms = 500;
      int slave_id = 1;
    }connection;

    void declare(rclcpp::Node * node);
    void update(rclcpp::Node * node);
  };
  ModbusNode(std::string name = "modbus_connection");
  bool readInputRegisters(Block & block);
  bool readRegisters(Block & block);
  bool writeRegisters(Block & block);

protected:
  void timer_callback();
  virtual void onPoll(){};
  roship_io::modbus::Modbus modbus_;
  rclcpp::TimerBase::SharedPtr timer_; ///< Shared pointer to the timer
  rclcpp::TimerBase::SharedPtr connect_timer_;
  void connect();
  bool connected_ = false;
  Params params_;
};

MODBUS_NS_FOOT
