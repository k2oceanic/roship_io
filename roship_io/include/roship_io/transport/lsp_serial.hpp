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
        int baud_rate = 38400;
        int buffer_size = 1024;
        int character_size = 8;
        int stop_bits = 1;
        int read_timeout_ms = 50;
        byte end_of_frame_byte = 0xAA;
        std::string end_of_frame_ascii = "\n";
        bool use_end_of_frame_byte = false;
        bool use_end_of_frame_ascii = false;
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
