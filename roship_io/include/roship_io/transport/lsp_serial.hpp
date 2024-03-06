#pragma once

#include "transport_interface.hpp"

#include <libserialport.h>
#include <vector>
#include <functional>
#include <thread>
#include <atomic>
#include <iostream>
#include <thread>


TRANSPORT_NS_HEAD

/**
 * @class LspSerial
 * @brief A class for serial communication using libserialport.
 *
 * This class provides an interface for serial communication using the libserialport library.
 * It implements the TransportInterface for sending and receiving messages over a serial connection.
 */
class LspSerial : public TransportInterface {
public:
    using byte = uint8_t;
    using MessageCallback = std::function<void(const std::vector<byte>&)>;

    /**
     * @struct Params
     * @brief Configuration parameters for the serial connection.
     */
    struct Params {
        std::string port = "/dev/ttyUSB0"; ///< Serial port device name.
        int baud_rate = 38400;             ///< Baud rate for the serial connection.
        int buffer_size = 1024;            ///< Size of the read buffer.
        int character_size = 8;            ///< Number of data bits in each character.
        int stop_bits = 1;                 ///< Number of stop bits.
        int read_timeout_ms = 50;          ///< Timeout for reading in milliseconds.
        byte end_of_frame_byte = 0xAA;     ///< Byte indicating the end of a frame.
        std::string end_of_frame_ascii = "\n"; ///< ASCII string indicating the end of a frame.
        bool use_end_of_frame_byte = false; ///< Flag to use end_of_frame_byte for frame detection.
        bool use_end_of_frame_ascii = false; ///< Flag to use end_of_frame_ascii for frame detection.
    };

    /**
     * @brief Constructor for LspSerial.
     * @param params Configuration parameters for the serial connection.
     */
    LspSerial(Params params);

    /**
     * @brief Destructor for LspSerial.
     */
    ~LspSerial();

    // TransportInterface
    void send(const std::vector<byte>& message) override; ///< Send a message over the serial connection.
    void spinOnce() override;                             ///< does nothing, legacy from asio boost implementation.
    void addCallback(const MessageCallback& callback) override; ///< Set the callback function for received messages.

private:
    void startReadThread(); ///< Start the thread for reading serial data.
    void stopReadThread();  ///< Stop the thread for reading serial data.
    void readLoop();        ///< The loop for reading serial data and calling the callback function.

    struct sp_port* serial_port_; ///< Pointer to the libserialport port structure.
    Params params_;               ///< Configuration parameters for the serial connection.
    std::vector<byte> read_buffer_; ///< Buffer for reading serial data.
    MessageCallback message_callback_; ///< Callback function for received messages.
    std::thread read_thread_;          ///< Thread for reading serial data.
    std::atomic<bool> continue_reading_; ///< Flag to control the read thread loop.
};

TRANSPORT_NS_FOOT
