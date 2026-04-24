#pragma once

#include "transport_interface.hpp"
#include <mosquitto.h>
#include <string>
#include <vector>
#include <functional>
#include <memory>
#include <iostream>
#include <stdexcept>

TRANSPORT_NS_HEAD

/**
 * @brief MQTT transport client implementing TransportInterface.
 *
 * `MqttClient` wraps the libmosquitto C library to provide publish/subscribe
 * MQTT messaging.  On construction it connects to the broker, subscribes to
 * the configured topic list, and starts the mosquitto network loop.
 */
class MqttClient : public TransportInterface {
public:
    /**
     * @brief Configuration parameters for MqttClient.
     */
    struct Params {
        std::string host;                    ///< Broker hostname or IP address.
        int port;                            ///< Broker port (typically 1883).
        std::vector<std::string> topics;     ///< MQTT topics to subscribe to on connect.
        int keep_alive;                      ///< Keep-alive interval in seconds sent to the broker.
    };

    /**
     * @brief Callback type that receives both payload and originating MQTT topic.
     */
    using MqttMessageCallback = std::function<void(const std::vector<byte>&, const std::string&)>;

    /**
     * @brief Construct an MqttClient and connect to the broker.
     * 
     * @param params Broker address, port, topic list, and keep-alive interval.
     */
    MqttClient(const Params& params);

    /**
     * @brief Destructor. Disconnects from the broker and cleans up libmosquitto.
     */
    ~MqttClient();

    /**
     * @brief Publish `message` to the *first* configured topic.
     *
     * @param message Raw payload bytes to publish.
     */
    void send(const std::vector<byte>& message) override;

    /**
     * @brief Publish `message` to an explicit MQTT topic.
     *
     * @param topic   MQTT topic string to publish on.
     * @param message Raw payload bytes to publish.
     */
    void send(const std::string& topic, const std::vector<byte>& message);

    /**
     * @brief Not-used for MqttClient — handled by the mosquitto background loop.
     *
     */
    void spinOnce() override;

    /**
     * @brief Register a topic-agnostic callback invoked for every received message.
     *
     * @param callback Function called with the raw payload bytes.
     */
    void addCallback(const MessageCallback& callback) override;

    /**
     * @brief Register a topic-aware callback invoked for every received message.
     *
     * @param callback Function called with the raw payload bytes and the MQTT topic string.
     */
    void addMqttCallback(const MqttMessageCallback& callback);

private:
    /**
     * @brief libmosquitto connect event handler.
     * @param mosq     mosquitto instance.
     * @param userdata Pointer to the owning MqttClient.
     * @param result   Connection result code (0 = success).
     */
    static void on_connect(struct mosquitto* mosq, void* userdata, int result);

    /**
     * @brief libmosquitto message receive handler.
     *
     * @param mosq     mosquitto instance.
     * @param userdata Pointer to the owning MqttClient.
     * @param message  Received message struct (topic + payload).
     */
    static void on_message(struct mosquitto* mosq, void* userdata, const struct mosquitto_message* message);

    /**
     * @brief libmosquitto disconnect event handler.
     * @param mosq     mosquitto instance.
     * @param userdata Pointer to the owning MqttClient.
     * @param reason   Disconnect reason code.
     */
    static void on_disconnect(struct mosquitto* mosq, void* userdata, int reason);

    Params params_;                                     ///< Broker and topic configuration.
    struct mosquitto* mosq_;                            ///< libmosquitto client handle.
    std::vector<MessageCallback> callbacks_;            ///< Topic-agnostic receive callbacks.
    std::vector<MqttMessageCallback> mqtt_callbacks_;   ///< Topic-aware receive callbacks.
};

TRANSPORT_NS_FOOT
