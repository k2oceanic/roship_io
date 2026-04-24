#pragma once  // Favor using this over the #ifndef, #define method

#include "connection_defs.hpp"  //  This is where we include all our namespace stuff for the package
#include "transport/transport_interface.hpp"

#include <rclcpp/rclcpp.hpp>

CONNECTION_NS_HEAD

/**
 * @brief base class for all ROS2 transport connection wrappers.
 *
 * `IoConnection` provides a shared ROS2 node pointer for each
 *  connection type (UDP, serial, MQTT).
 *
 * @tparam transport_T Concrete transport type (must implement
 *         `transport::TransportInterface`).
 */
template <typename transport_T>
class IoConnection
{
public:
  /** @brief Convenience alias for a shared pointer to this connection. */
  typedef std::shared_ptr<IoConnection> SharedPtr;

  /**
   * @brief Base parameter struct for IoConnection subclasses.
   *
   * Derived classes should extend this struct and override `declare()`/`update()`
   * to expose transport-specific parameters through the ROS2 parameter server.
   */
  struct Params {
    /**
     * @brief Declare ROS2 parameters on the given node.
     *
     * Override in derived classes to call `node->declare_parameter()`.
     *
     * @param node Shared pointer to the ROS2 node.
     */
    virtual void declare(rclcpp::Node::SharedPtr node) { return; }

    /**
     * @brief Read current ROS2 parameter values into this struct.
     *
     * Override in derived classes to call `node->get_parameter()`.
     *
     * @param node Shared pointer to the ROS2 node.
     */
    virtual void update(rclcpp::Node::SharedPtr node) { return; }
  };

  /**
   * @brief Construct an IoConnection associated with the given ROS2 node.
   *
   * @param node Shared pointer to the owning ROS2 node used for parameter
   *             declaration and publisher/subscriber creation.
   */
  IoConnection(rclcpp::Node::SharedPtr node) : node_ptr_(node) { return; }

protected:
  rclcpp::Node::SharedPtr       node_ptr_;       ///< Owning ROS2 node.
  std::shared_ptr<transport_T>  transport_ptr_;  ///< Underlying transport instance.
};

CONNECTION_NS_FOOT
