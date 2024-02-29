#include "transport/lsp_serial.hpp"

#include <iostream>
#include <chrono>
#include <thread>
#include <vector>

int main()
{
    roship_io::transport::LspSerial::Params params;
    params.port = "/dev/ttyUSB0";
    params.baud_rate = 9600;
    params.buffer_size = 1024; // Adjust as needed
    params.character_size = 8; // Adjust as needed
    params.stop_bits = 1;      // Adjust as needed

    roship_io::transport::LspSerial serial(params);

    std::vector<roship_io::byte> data = {'H', 'e', 'l', 'l', 'o', '\n', '\r'};
    while (true)
    {
        serial.send(data);
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }

    return 0;
}

