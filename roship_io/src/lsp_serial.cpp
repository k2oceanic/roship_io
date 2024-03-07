#include "transport/lsp_serial.hpp"

TRANSPORT_NS_HEAD

LspSerial::LspSerial(Params params) : params_(params), continue_reading_(false) {
    sp_get_port_by_name(params.port.c_str(), &serial_port_);
    sp_open(serial_port_, SP_MODE_READ_WRITE);
    sp_set_baudrate(serial_port_, params.baud_rate);
    sp_set_bits(serial_port_, params.character_size);
    sp_set_stopbits(serial_port_, params.stop_bits);
    sp_set_parity(serial_port_, SP_PARITY_NONE);
    sp_set_flowcontrol(serial_port_, SP_FLOWCONTROL_NONE);
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
    std::vector<byte> message;
    auto last_byte_time = std::chrono::steady_clock::now();

    while (continue_reading_) {
        int bytes_read = sp_nonblocking_read(serial_port_, read_buffer_.data(), read_buffer_.size());

        if (bytes_read > 0) {
            message.insert(message.end(), read_buffer_.begin(), read_buffer_.begin() + bytes_read);
            last_byte_time = std::chrono::steady_clock::now(); // Update the time of the last received byte

            // Check for end-of-frame byte or ASCII delimiter
            bool message_end_found = false;
            std::vector<byte>::iterator eof_iter;
            if (params_.use_end_of_frame_byte) {
                eof_iter = std::find(message.begin(), message.end(), params_.end_of_frame_byte);
                if (eof_iter != message.end()) {
                    message_end_found = true;
                }
            } else if (params_.use_end_of_frame_ascii) {
                eof_iter = std::search(message.begin(), message.end(), params_.end_of_frame_ascii.begin(), params_.end_of_frame_ascii.end());
                if (eof_iter != message.end()) {
                    message_end_found = true;
                    eof_iter += params_.end_of_frame_ascii.length(); // Move iterator to the end of the ASCII delimiter
                }
            }

            if (message_end_found) {
                std::vector<byte> complete_message(message.begin(), eof_iter + 1);
                if (message_callback_) {
                    message_callback_(complete_message);
                }
                message.clear();
            }
        }

        if (!message.empty() && std::chrono::steady_clock::now() - last_byte_time >= std::chrono::milliseconds(params_.read_timeout_ms)) {
            if (message_callback_) {
                message_callback_(message);
            }
            message.clear();
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
}

void LspSerial::spinOnce() {
    // Right now does nothing, might replace while loop functionality of read loop moving forward
}

TRANSPORT_NS_FOOT
