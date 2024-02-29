#pragma once

#include "transport_interface.hpp"

#include <libserialport.h>
#include <vector>
#include <functional>
#include <thread>
#include <atomic>
#include <iostream>

TRANSPORT_NS_HEAD

class LspSerial : public TransportInterface {
public:
    using byte = uint8_t;
    using MessageCallback = std::function<void(const std::vector<byte>&)>;

    struct Params {
        std::string port = "/dev/ttyUSB0";
        int baud_rate = 9600;
        int buffer_size = 256;
        int character_size = 8;
        int stop_bits = 1;
    };

    LspSerial(Params params);
    ~LspSerial();

    // TransportInterface
    void send(const std::vector<byte>& message) override;
    void spinOnce() override;
    void addCallback(const MessageCallback& callback) override;

private:
    void startReadThread();
    void stopReadThread();
    void readLoop();

    struct sp_port* serial_port_;
    Params params_;
    std::vector<byte> read_buffer_;
    MessageCallback message_callback_;
    std::thread read_thread_;
    std::atomic<bool> continue_reading_;
};

TRANSPORT_NS_FOOT
