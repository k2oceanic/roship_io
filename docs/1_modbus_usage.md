# Modbus Usage

`ModbusNode` is a base class — you subclass it to build a node for a specific Modbus device.
The base class handles TCP connect/reconnect and periodic polling. You just define:

1. A `Block` struct that mirrors your device's register layout.
2. A node class that overrides `onPoll()` to read/write/publish.

---

## Step 1 — Define a register Block

A `Block` is a plain struct whose fields map directly onto the device's Modbus register layout.
Use `BE_WORD_f32`, `BE_WORD_s32`, etc. to handle the big-endian word order that Modbus devices use.

```cpp
#include <roship_io/modbus/blocks/block.hpp>

// Example: a sensor that exposes two float registers at address 0x0100
struct SensorBlock : public roship_io::modbus::Block
{
  roship_io::primitives::BE_WORD_f32 temperature;  // registers 0x0100–0x0101
  roship_io::primitives::BE_WORD_f32 pressure;     // registers 0x0102–0x0103

  roship_io::primitives::WordBuffer buffer() override
  {
    return reinterpret_cast<roship_io::primitives::WordBuffer>(&temperature);
  }

  size_t size() override { return sizeof(SensorBlock) / 2; }  // size in 16-bit words
  int modbusAddress() override { return 0x0100; }
};
```

---

## Step 2 — Subclass ModbusNode

```cpp
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32.hpp>
#include <roship_io/modbus/modbus_node.hpp>
#include "sensor_block.hpp"

class SensorNode : public roship_io::modbus::ModbusNode
{
public:
  SensorNode() : ModbusNode("sensor_node")
  {
    temp_pub_ = create_publisher<std_msgs::msg::Float32>("~/temperature", 10);
  }

protected:
  void onPoll() override
  {
    if (readRegisters(block_))
    {
      std_msgs::msg::Float32 msg;
      msg.data = block_.temperature;  // implicit big-endian cast to float
      temp_pub_->publish(msg);
    }
  }

private:
  SensorBlock block_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr temp_pub_;
};

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SensorNode>());
  rclcpp::shutdown();
}
```

---

## Step 3 — Configure with a YAML file

```yaml
sensor_node:
  ros__parameters:
    connection.ip: "192.168.1.20"
    connection.network_port: 502
    connection.slave_id: 1
    timers.poll_ms: 100
```

Launch with:
```bash
ros2 run my_pkg sensor_node --ros-args --params-file sensor_node.yaml
```

---

## Write example

To write registers back to the device, populate the block fields and call `writeRegisters()`:

```cpp
void onPoll() override
{
  // Read first
  if (readRegisters(block_))
  {
    float temp = block_.temperature;

    // Conditionally write a setpoint
    if (temp > 50.0f)
    {
      block_.pressure.set(0.0f);  // explicit set for writes
      writeRegisters(block_);
    }
  }
}
```

> **Note:** `readRegisters()` and `writeRegisters()` both return `false` on failure and
> log a warning automatically. The node will attempt to reconnect on the next
> `timers.connect_ms` tick.
