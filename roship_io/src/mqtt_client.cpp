/** Copyright © 2025 Seaward Science. */

#include "transport/mqtt_client.hpp"

static bool mosquitto_initialized = false;

TRANSPORT_NS_HEAD

MqttClient::MqttClient(const Params& params) : params_(params) {
    if (!mosquitto_initialized) {
        mosquitto_lib_init();
        mosquitto_initialized = true;
    }
    mosq_ = mosquitto_new(nullptr, true, this);
    if (!mosq_) {
        throw std::runtime_error("Failed to create Mosquitto instance");
    }

    mosquitto_connect_callback_set(mosq_, on_connect);
    mosquitto_message_callback_set(mosq_, on_message);
    mosquitto_disconnect_callback_set(mosq_, on_disconnect);

    int ret = mosquitto_connect_async(mosq_, params_.host.c_str(), params_.port, params_.keep_alive);
    if (ret != MOSQ_ERR_SUCCESS) {
        throw std::runtime_error("Failed to connect to MQTT broker: " + std::string(mosquitto_strerror(ret)));
    }
}

MqttClient::~MqttClient() {
    if (mosq_) {
        mosquitto_disconnect(mosq_);
        mosquitto_destroy(mosq_);
    }
    if (mosquitto_initialized) {
        mosquitto_lib_cleanup();
        mosquitto_initialized = false;
    }
}

void MqttClient::send(const std::vector<byte>& message) {
    if (!params_.topics.empty()) {
        int ret = mosquitto_publish(
            mosq_, nullptr, params_.topics[0].c_str(), message.size(), message.data(), 0, false);
        if (ret != MOSQ_ERR_SUCCESS) {
            throw std::runtime_error("Failed to publish message: " + std::string(mosquitto_strerror(ret)));
        }
    }
}

void MqttClient::send(const std::string& topic, const std::vector<byte>& message) {
    int ret = mosquitto_publish(mosq_, nullptr, topic.c_str(), message.size(), message.data(), 0, false);
    if (ret != MOSQ_ERR_SUCCESS) {
        throw std::runtime_error("Failed to publish message: " + std::string(mosquitto_strerror(ret)));
    }
}

void MqttClient::spinOnce() {
    int ret = mosquitto_loop(mosq_, 0, 1);
    if (ret != MOSQ_ERR_SUCCESS) {
        throw std::runtime_error("Failed to process MQTT events: " + std::string(mosquitto_strerror(ret)));
    }
}

void MqttClient::addCallback(const MessageCallback& callback) {
    callbacks_.push_back(callback);
}

void MqttClient::addMqttCallback(const MqttMessageCallback& callback) {
    mqtt_callbacks_.push_back(callback);
}

void MqttClient::on_connect(struct mosquitto* mosq, void* userdata, int result) {
    MqttClient* client = static_cast<MqttClient*>(userdata);
    if (client && result == MOSQ_ERR_SUCCESS) {
        for (const auto& topic : client->params_.topics) {
            mosquitto_subscribe(mosq, nullptr, topic.c_str(), 0);
        }
    }
}

void MqttClient::on_message(struct mosquitto* mosq, void* userdata, const struct mosquitto_message* message) {
    (void)mosq;
    MqttClient* client = static_cast<MqttClient*>(userdata);
    if (client) {
        std::vector<byte> payload(
            static_cast<byte*>(message->payload),
            static_cast<byte*>(message->payload) + message->payloadlen);
        std::string topic(message->topic);
        for (const auto& callback : client->mqtt_callbacks_) {
            callback(payload, topic);
        }
    }
}

void MqttClient::on_disconnect(struct mosquitto* mosq, void* userdata, int reason) {
    (void)mosq;
    MqttClient* client = static_cast<MqttClient*>(userdata);
    if (client) {
        std::cerr << "MQTT client disconnected: " << mosquitto_strerror(reason) << std::endl;

        int ret = mosquitto_reconnect_async(client->mosq_);
        if (ret != MOSQ_ERR_SUCCESS) {
            std::cerr << "Failed to reconnect to MQTT broker: " << mosquitto_strerror(ret) << std::endl;
        }
    }
}

TRANSPORT_NS_FOOT
