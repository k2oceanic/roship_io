#include "transport/lsp_serial.hpp"

TRANSPORT_NS_HEAD

LspSerial::LspSerial(Params params) : params_(params), continue_reading_(false) {
    sp_get_port_by_name(params.port.c_str(), &serial_port_);
    sp_open(serial_port_, SP_MODE_READ_WRITE);
    sp_set_baudrate(serial_port_, params.baud_rate);
    sp_set_bits(serial_port_, params.character_size);
    sp_set_parity(serial_port_, SP_PARITY_NONE);
    sp_set_stopbits(serial_port_, params.stop_bits);
    read_buffer_.resize(params.buffer_size);
    startReadThread();
}

LspSerial::~LspSerial() {
    stopReadThread();
    sp_close(serial_port_);
    sp_free_port(serial_port_);
}

void LspSerial::send(const std::vector<byte>& message) {
    sp_nonblocking_write(serial_port_, message.data(), message.size());
}

void LspSerial::addCallback(const MessageCallback& callback) {
    message_callback_ = callback;
}

void LspSerial::startReadThread() {
    continue_reading_ = true;
    read_thread_ = std::thread(&LspSerial::readLoop, this);
}

void LspSerial::stopReadThread() {
    continue_reading_ = false;
    if (read_thread_.joinable()) {
        read_thread_.join();
    }
}

void LspSerial::readLoop() {
    while (continue_reading_) {
        int bytes_read = sp_nonblocking_read(serial_port_, read_buffer_.data(), read_buffer_.size());
        if (bytes_read > 0) {
            std::vector<byte> message(read_buffer_.begin(), read_buffer_.begin() + bytes_read);
            if (message_callback_) {
                message_callback_(message);
            }
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(10)); // Adjust as needed
    }
}

void LspSerial::spinOnce() {
    if (!read_buffer_.empty()) {
        if (message_callback_) {
            message_callback_(read_buffer_);
            read_buffer_.clear();
        }
    }
}

TRANSPORT_NS_FOOT