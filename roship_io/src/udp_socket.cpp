#include "transport/udp_socket.hpp"

TRANSPORT_NS_HEAD

UdpSocket::UdpSocket(int port, size_t buffer_size) : port_(port), buffer_size_(buffer_size) {
    sockfd_ = socket(AF_INET, SOCK_DGRAM, 0);
    if (sockfd_ < 0) {
        throw std::runtime_error("Error opening socket");
    }

    // Set the socket to non-blocking mode
    int flags = fcntl(sockfd_, F_GETFL, 0);
    if (flags < 0) {
        throw std::runtime_error("Error getting socket flags");
    }
    flags |= O_NONBLOCK;
    if (fcntl(sockfd_, F_SETFL, flags) < 0) {
        throw std::runtime_error("Error setting socket to non-blocking mode");
    }

    memset(&addr_, 0, sizeof(addr_));
    addr_.sin_family = AF_INET;
    addr_.sin_addr.s_addr = INADDR_ANY;
    addr_.sin_port = htons(port_);

    if (bind(sockfd_, reinterpret_cast<struct sockaddr*>(&addr_), sizeof(addr_)) < 0) {
        std::ostringstream msg;
        msg << "Error binding socket to port " << port_ << ": " << strerror(errno);
        throw std::runtime_error(msg.str());
    }

    buffer_ = new byte[buffer_size_];
}

UdpSocket::~UdpSocket() {
    delete[] buffer_;
    close(sockfd_);
}

void UdpSocket::SendTo(const std::string& ip, int port, const std::vector<byte>& message) {
    struct sockaddr_in dest_addr;
    memset(&dest_addr, 0, sizeof(dest_addr));
    dest_addr.sin_family = AF_INET;
    dest_addr.sin_port = htons(port);

    if (inet_pton(AF_INET, ip.c_str(), &dest_addr.sin_addr) <= 0) {
        throw std::runtime_error("Invalid address/ Address not supported");
    }

    sendto(sockfd_, message.data(), message.size(), 0, reinterpret_cast<struct sockaddr*>(&dest_addr), sizeof(dest_addr));
}

void UdpSocket::Receive() {
    struct sockaddr_in sender_addr;
    socklen_t sender_len = sizeof(sender_addr);

    ssize_t n = recvfrom(sockfd_, buffer_, buffer_size_, 0, reinterpret_cast<struct sockaddr*>(&sender_addr), &sender_len);

    if (n > 0) {
        std::vector<byte> message(buffer_, buffer_ + n);
        for (const auto& callback : callbacks_) {
            callback(message);
        }
    } else if (n < 0) {
        if (errno != EWOULDBLOCK && errno != EAGAIN) {
            throw std::runtime_error("Error receiving data: " + std::string(strerror(errno)));
        }
    }


}

void UdpSocket::AddCallback(const MessageCallback& callback) {
    callbacks_.push_back(callback);
}

TRANSPORT_NS_FOOT
