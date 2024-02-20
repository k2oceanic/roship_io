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


// #include <boost/asio.hpp>
// #include <iostream>
// #define BUFSIZE 256
// int main() {
//     boost::asio::io_service io;
//     // Open serial port
//     boost::asio::serial_port serial(io, "/dev/ttyUSB0");
//     // Configure basic serial port parameters: 115.2kBaud, 8N1
//     serial.set_option(boost::asio::serial_port_base::baud_rate(9600));
//     serial.set_option(boost::asio::serial_port_base::character_size(8 /* data bits */));
//     serial.set_option(boost::asio::serial_port_base::parity(boost::asio::serial_port_base::parity::none));
//     serial.set_option(boost::asio::serial_port_base::stop_bits(boost::asio::serial_port_base::stop_bits::one));
//     // Read data in a loop and copy to stdout
//     while(true) {
//         char data[BUFSIZE];
//         size_t n = serial.read_some(boost::asio::buffer(data, BUFSIZE));
//         // Write data to stdout
//         std::cout.write(data, n);
//     }
// }




// #include <boost/asio.hpp>
// #include <iostream>
// #include <string>



// void readHandler(const boost::system::error_code& error, std::size_t bytes_transferred) {
//     if (!error) {
//         std::cout << "Read " << bytes_transferred << " bytes." << std::endl;
//     } else {
//         std::cerr << "Read error: " << error.message() << std::endl;
//     }
// }


// int main() {
//     try {
//         boost::asio::io_service io;
//         boost::asio::serial_port serial(io, "/dev/ttyUSB0"); // Adjust the device name as needed

//         // Set serial port parameters
//         serial.set_option(boost::asio::serial_port_base::baud_rate(9600));
//         serial.set_option(boost::asio::serial_port_base::character_size(8));
//         serial.set_option(boost::asio::serial_port_base::flow_control(boost::asio::serial_port_base::flow_control::none));
//         serial.set_option(boost::asio::serial_port_base::parity(boost::asio::serial_port_base::parity::none));
//         serial.set_option(boost::asio::serial_port_base::stop_bits(boost::asio::serial_port_base::stop_bits::one));

//         // Write to serial port
//         std::string data = "Hello, serial port!";
//         boost::asio::write(serial, boost::asio::buffer(data.c_str(), data.size()));

//         // Prepare a buffer for reading
//         boost::asio::streambuf read_buf;

//         // Start an asynchronous read
//         boost::asio::async_read(serial, read_buf, boost::asio::transfer_at_least(1), readHandler);

//         // Run the I/O service to perform the asynchronous operation
//         io.run();
//     } catch (std::exception& e) {
//         std::cerr << "Exception: " << e.what() << std::endl;
//     }

//     return 0;
// }
