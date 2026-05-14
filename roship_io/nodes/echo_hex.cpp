#include <rclcpp/rclcpp.hpp>
#include <io_interfaces/msg/raw_packet.hpp>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>

// Render one byte as a UTF-8 string for the ASCII column.
// Control chars (< 0x20, 0x7F, C1 range 0x80-0x9F) → "."
// Printable ASCII (0x20-0x7E) → the character itself.
// Latin-1 supplement (0xA0-0xFF) → two-byte UTF-8 encoding of U+00A0..U+00FF.
static std::string byte_char(unsigned char c)
{
    if (c < 0x20 || c == 0x7F || (c >= 0x80 && c <= 0x9F))
        return ".";
    if (c < 0x80)
        return std::string(1, static_cast<char>(c));
    // 0xA0-0xFF: encode as two-byte UTF-8
    return std::string({ static_cast<char>(0xC0 | (c >> 6)),
                         static_cast<char>(0x80 | (c & 0x3F)) });
}

int main(int argc, char* argv[])
{
    if (argc < 2) {
        std::cerr << "Usage: ros2 run roship_io echo_hex <topic>\n";
        return 1;
    }

    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("echo_hex");
    std::string topic = argv[1];

    // Best-effort so it connects to any publisher QoS
    auto qos = rclcpp::QoS(10).best_effort();

    auto sub = node->create_subscription<io_interfaces::msg::RawPacket>(
        topic, qos,
        [](const io_interfaces::msg::RawPacket::SharedPtr msg) {
            std::ostringstream hex, asc;
            for (auto b : msg->data) {
                unsigned char c = static_cast<unsigned char>(b);
                hex << std::hex << std::uppercase
                    << std::setw(2) << std::setfill('0') << static_cast<int>(c) << ' ';
                asc << byte_char(c) << "  ";
            }
            std::cout << hex.str() << '\n' << asc.str() << "\n\n";
        });

    std::cout << "Echoing " << topic << '\n';
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
