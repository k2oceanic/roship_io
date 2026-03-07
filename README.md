# roship_io

A ROS2 package providing generic I/O transport nodes and a Modbus TCP client library for connecting to serial, UDP, and MQTT devices.

Complete API documentation: [https://oceanexplorationtrust.bitbucket.io/rovws](https://oceanexplorationtrust.bitbucket.io/rovws)

---

## Dependencies

Install system libraries before building:

```bash
sudo apt install libserialport-dev libmodbus-dev libmosquitto-dev
```

Then install ROS dependencies:

```bash
rosdep install --from-paths src --ignore-src --rosdistro humble -y
```

---

## Build

```bash
colcon build --symlink-install --packages-select roship_io
source install/setup.bash
```

---

## Connection Nodes

Each node bridges a physical transport to a pair of ROS2 topics using `io_interfaces/msg/RawPacket`.

| Executable          | Transport  | `~/from_device`      | `~/to_device`        |
|---------------------|------------|----------------------|----------------------|
| `udp_connection`    | UDP        | incoming datagrams   | outgoing datagrams   |
| `serial_connection` | Serial     | received frames      | bytes to send        |
| `mqtt_connection`   | MQTT       | per-topic (broker→ROS2) | per-topic (ROS2→broker) |

### Launch

```bash
# UDP
ros2 launch roship_io udp_connection.launch.xml ip:=192.168.1.50 port:=9000

# Serial
ros2 launch roship_io serial_connection.launch.xml port:=/dev/ttyUSB0 baud_rate:=115200

# MQTT
ros2 launch roship_io mqtt_connection.launch.xml
```

See `config/*.yaml` for the full parameter list for each node.

---

## Modbus Library

`roship_io` provides a `ModbusNode` base class for building device-specific Modbus TCP polling nodes. It handles connection, reconnection, and poll timing. Subclass it and override `onPoll()`: See `docs/1_modbus_usage.md` for a step-by-step example.

---

## License

Apache 2.0 — see [LICENSE](LICENSE).

## Maintainer

Kristopher Krasnosky — [support@seaward.science](mailto:support@seaward.science)

