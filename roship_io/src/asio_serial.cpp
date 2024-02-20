// #include "transport/asio_serial.hpp" // Ensure this path is correct

// TRANSPORT_NS_HEAD

// AsioSerial::AsioSerial(Params params)
//     : io_service_(),
//       serial_port_(io_service_),
//       params_(params),
//       read_buffer_(params.buffer_size, 0) {

//     serial_port_.open(params.port);
//     serial_port_.set_option(boost::asio::serial_port_base::baud_rate(params.baud_rate));
//     serial_port_.set_option(boost::asio::serial_port_base::character_size(params.character_size));
//     serial_port_.set_option(boost::asio::serial_port_base::flow_control(boost::asio::serial_port_base::flow_control::none));
//     serial_port_.set_option(boost::asio::serial_port_base::parity(boost::asio::serial_port_base::parity::none));
//     serial_port_.set_option(boost::asio::serial_port_base::stop_bits(boost::asio::serial_port_base::stop_bits::one));

//     io_thread_ = std::make_shared<std::thread>([this]() {
//         io_service_.run();
//     });
// }

// AsioSerial::~AsioSerial() {
//     if (serial_port_.is_open()) {
//         serial_port_.close();
//     }
//     io_service_.stop();
//     if (io_thread_ && io_thread_->joinable()) {
//         io_thread_->join();
//     }
// }

// void AsioSerial::send(const std::vector<byte>& message) {
//     boost::asio::write(serial_port_, boost::asio::buffer(message));
// }

// void AsioSerial::spinOnce() {
//     if (!is_reading_) {
//         doRead();
//     }
// }

// void AsioSerial::addCallback(const MessageCallback& callback) {
//     message_callback_ = callback;
// }

// void AsioSerial::doRead() {
//     is_reading_ = true;
//     serial_port_.async_read_some(boost::asio::buffer(read_buffer_),
//                                  [this](const boost::system::error_code& ec, std::size_t bytes_transferred) {
//                                      is_reading_ = false;
//                                      readHandler(ec, bytes_transferred);
//                                  });
// }

// void AsioSerial::readHandler(const boost::system::error_code& ec, std::size_t bytes_transferred) {
//     if (!ec) {
//         if (message_callback_ && bytes_transferred > 0) {
//             std::vector<byte> message(read_buffer_.begin(), read_buffer_.begin() + bytes_transferred);
//             message_callback_(message);
//         }
//     } else {
//         std::cerr << "Read error: " << ec.message() << std::endl;
//     }
// }

// TRANSPORT_NS_FOOT


#include "transport/asio_serial.hpp"

TRANSPORT_NS_HEAD

AsioSerial::AsioSerial(Params params)
    : serial_port_(io_service_, params.port) {

    serial_port_.set_option(boost::asio::serial_port_base::baud_rate(params.baud_rate));
    serial_port_.set_option(boost::asio::serial_port_base::character_size(params.character_size));
    serial_port_.set_option(boost::asio::serial_port_base::flow_control(boost::asio::serial_port_base::flow_control::none));
    serial_port_.set_option(boost::asio::serial_port_base::parity(boost::asio::serial_port_base::parity::none));
    serial_port_.set_option(boost::asio::serial_port_base::stop_bits(boost::asio::serial_port_base::stop_bits::one));
    read_buffer_.resize(params.buffer_size); // Adjust buffer size as needed
    doRead();
}

AsioSerial::~AsioSerial() {
    serial_port_.close();
    io_service_.stop();
}

void AsioSerial::send(const std::vector<byte>& message) {
    boost::asio::write(serial_port_, boost::asio::buffer(message.data(), message.size()));
}

void AsioSerial::spinOnce() {
  io_service_.run_one();
}

void AsioSerial::addCallback(const MessageCallback& callback) {
  message_callback_ = callback;
}

void AsioSerial::doRead() {
  serial_port_.async_read_some(boost::asio::buffer(read_buffer_),
                                std::bind(&AsioSerial::readHandler, this,
                                          std::placeholders::_1, std::placeholders::_2));
  //io_service_.run_one();
  //auto bytes_transferred = serial_port_.read_some(boost::asio::buffer(read_buffer_));
  // std::vector<byte> message(read_buffer_.begin(), read_buffer_.begin() + bytes_transferred);
  // if (message_callback_) {
  //     message_callback_(message);
  // }
  // std::cout << "msg start" << std::endl;
  // for(size_t i = 0 ; i<bytes_transferred; i++){

  //   std::cout << read_buffer_[i];
  // }
  //doRead(); // Continue reading
}

void AsioSerial::readHandler(const boost::system::error_code& ec, std::size_t bytes_transferred) {
    if (!ec) {
      std::vector<byte> message;
      message.resize(bytes_transferred);
      std::cout << "bytes: " << bytes_transferred << std::endl;
      for(size_t i = 0 ; i<bytes_transferred; i++){
        std::cout << read_buffer_[i];
        message[i] = read_buffer_[i];
      }
      if (message_callback_) {
        message_callback_(message);
      }
      // std::vector<byte> message(read_buffer_.begin(), read_buffer_.begin() + bytes_transferred);
      // if (message_callback_) {
      //     message_callback_(message);
      // }
      doRead(); // Continue reading
    } else {
        std::cerr << "Read error: " << ec.message() << std::endl;
    }
}

TRANSPORT_NS_FOOT
