#pragma once

#include "connection_defs.hpp"
#include "io_connection.hpp"
#include "transport/mqtt_client.hpp"
#include <io_interfaces/msg/raw_packet.hpp>

#include <rclcpp/rclcpp.hpp>

CONNECTION_NS_HEAD

/**
 * @brief ROS2 connection node wrapper for MQTT transport.
 *
 * `MqttConnection` owns a `transport::MqttClient` and dynamically creates one
 * ROS2 publisher **and** one ROS2 subscriber per configured MQTT topic:
 * 
 */
class MqttConnection : public IoConnection<transport::MqttClient>
{
public:
    /**
     * @brief Configuration parameters for MqttConnection.
     */
    struct Params
    {
        /**
         * @brief Initialise with safe localhost defaults.
         */
        Params();

        /**
         * @brief Declare all MQTT parameters on the given ROS2 node.
         * @param node Shared pointer to the ROS2 node.
         */
        void declare(rclcpp::Node::SharedPtr node);

        /**
         * @brief Read current parameter values from the ROS2 node.
         * @param node Shared pointer to the ROS2 node.
         */
        void update(rclcpp::Node::SharedPtr node);

        transport::MqttClient::Params client; ///< MQTT client configuration (host, port, topics, keep-alive).
    };

    /**
     * @brief Construct an MqttConnection, declare/read parameters, and create per-topic publishers/subscribers.
     *
     * @param node Shared pointer to the ROS2 node that owns topics and parameters.
     */
    MqttConnection(rclcpp::Node::SharedPtr node);

    /**
     * @brief Callback invoked by MqttClient for each received MQTT message.
     *
     * Routes the payload to the ROS2 publisher whose key matches `topic`.
     *
     * @param message Raw MQTT payload bytes.
     * @param topic   MQTT topic string on which the message arrived.
     */
    void mqttCallback(const std::vector<byte>& message, const std::string& topic);

    /**
     * @brief Forward a `RawPacket` from a ROS2 topic to the specified MQTT topic.
     *
     * @param msg   Message whose `data` field is published to the broker.
     * @param topic Destination MQTT topic string.
     */
    void sendToDevice(const io_interfaces::msg::RawPacket& msg, const std::string& topic);

    /**
     * @brief Not Used for MqttConnection — the MQTT loop runs in a background thread.
     *
     * Kept for interface consistency with other connection types.
     */
    void spin_once();

protected:
    rclcpp::TimerBase::SharedPtr timer_; ///< Periodic timer (currently unused; retained for future polling use).
    std::shared_ptr<transport::MqttClient> client_ptr_; ///< Underlying MQTT transport client.

    /** @brief Per-MQTT-topic ROS2 publishers (key = MQTT topic string). */
    std::map<std::string, rclcpp::Publisher<io_interfaces::msg::RawPacket>::SharedPtr> ros_publishers_;

    /** @brief Per-MQTT-topic ROS2 subscribers (key = MQTT topic string). */
    std::map<std::string, rclcpp::Subscription<io_interfaces::msg::RawPacket>::SharedPtr> ros_subscribers_;

    rclcpp::Publisher<io_interfaces::msg::RawPacket>::SharedPtr raw_pub_;     ///< Fallback publisher (optional legacy use).
    rclcpp::Subscription<io_interfaces::msg::RawPacket>::SharedPtr raw_sub_;  ///< Fallback subscriber (optional legacy use).
    Params params_; ///< Runtime parameters.
};

CONNECTION_NS_FOOT
