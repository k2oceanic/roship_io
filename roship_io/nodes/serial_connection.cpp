#include "connection/serial_connection.hpp"

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::Node::SharedPtr node(new rclcpp::Node("serial_connection"));
  roship_io::connection::SerialConnection::SharedPtr connection(new roship_io::connection::SerialConnection(node));
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}