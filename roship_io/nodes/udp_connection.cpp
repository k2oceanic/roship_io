#include "connection/udp_connection.hpp"

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::Node::SharedPtr node(new rclcpp::Node("udp_connection"));
  roship_io::connection::UdpConnection::SharedPtr connection(new roship_io::connection::UdpConnection(node));
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;

}
