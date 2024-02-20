#include "transport/asio_serial.hpp"

#include <iostream>
#include <chrono>
#include <thread>

int main(int argc, char *argv[])
{
  roship_io::transport::AsioSerial::Params params;
  params.port = "/dev/ttyUSB1";
  params.baud_rate = 9600;
  roship_io::transport::AsioSerial serial(params);
  std::vector<roship_io::byte> data = {'H','e','l','l','o','\n','\r'};
  while(true){
    serial.send(data);
    std::this_thread::sleep_for(std::chrono::seconds(1));
  }

}
