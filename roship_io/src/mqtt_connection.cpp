/** Copyright © 2025 Seaward Science. */

#include "connection/mqtt_connection.hpp"

CONNECTION_NS_HEAD

using namespace std::chrono_literals;

MqttConnection::Params::Params()
{
    client.host = "localhost";
    client.port = 1883;
    client.topics = {"topic1", "topic2", "topic3"};
    client.keep_alive = 60;
}

void MqttConnection::Params::declare(rclcpp::Node::SharedPtr node)
{
    node->declare_parameter("client.host", client.host);
    node->declare_parameter("client.port", client.port);
    node->declare_parameter("client.topics", client.topics);
    node->declare_parameter("client.keep_alive", client.keep_alive);
}

void MqttConnection::Params::update(rclcpp::Node::SharedPtr node)
{
    node->get_parameter("client.host", client.host);
    node->get_parameter("client.port", client.port);
    node->get_parameter("client.topics", client.topics);
    node->get_parameter("client.keep_alive", client.keep_alive);
}

MqttConnection::MqttConnection(rclcpp::Node::SharedPtr node) :
    IoConnection<transport::MqttClient>(node)
{
    params_.declare(node_ptr_);
    params_.update(node_ptr_);

    client_ptr_.reset(new transport::MqttClient(params_.client));
    client_ptr_->addMqttCallback(
        std::bind(&MqttConnection::mqttCallback, this, std::placeholders::_1, std::placeholders::_2));

    for (const auto& topic : params_.client.topics) {
        std::string ros_to_device_topic = "~/from_device/" + topic;
        std::string ros_from_device_topic = "~/to_device/" + topic;

        ros_publishers_[ros_to_device_topic] =
            node_ptr_->create_publisher<io_interfaces::msg::RawPacket>(ros_to_device_topic, 10);
        ros_subscribers_[ros_from_device_topic] = node_ptr_->create_subscription<io_interfaces::msg::RawPacket>(
            ros_from_device_topic, 1, [this, topic](const io_interfaces::msg::RawPacket::SharedPtr msg) {
                sendToDevice(*msg, topic);
            });
    }

    timer_ = node_ptr_->create_wall_timer(
        1ms, std::bind(&MqttConnection::spin_once, this));

    RCLCPP_INFO(node_ptr_->get_logger(), "MQTT client connected to %s:%d",
                params_.client.host.c_str(), params_.client.port);
}

void MqttConnection::mqttCallback(const std::vector<byte>& message, const std::string& topic)
{
    std::string message_str(message.begin(), message.end());
    RCLCPP_DEBUG(node_ptr_->get_logger(), "Recieved message %s from topic %s", message_str.c_str(), topic.c_str());
    auto ros_topic = "~/from_device/" + topic;
    auto it = ros_publishers_.find(ros_topic);
    if (it != ros_publishers_.end()) {
        auto rx_time = node_ptr_->now();
        io_interfaces::msg::RawPacket msg;
        msg.header.stamp = rx_time;
        msg.data = message;
        it->second->publish(msg);
    }
}

void MqttConnection::sendToDevice(const io_interfaces::msg::RawPacket& msg, const std::string& topic)
{
    std::string message_str(msg.data.begin(), msg.data.end());
    RCLCPP_DEBUG(
        node_ptr_->get_logger(), "Sending message %s to topic %s", message_str.c_str(), topic.c_str());
    client_ptr_->send(topic, msg.data);
}

void MqttConnection::spin_once()
{
    client_ptr_->spinOnce();
}

CONNECTION_NS_FOOT
