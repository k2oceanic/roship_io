# roship_io

A ROS2 package providing generic I/O transport nodes and a Modbus TCP client library for connecting to serial, UDP, and MQTT devices. The RawPacket message type allows for transport nodes to be used intercahnagbley as a bridge between devices and ROS2 topics, while recording all raw data traffic to and from devices.

Complete API documentation: [https://seawardscience.github.io/roship_io/](https://seawardscience.github.io/roship_io/)

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

## Parameters

### UDP Connection

| Parameter Name      | Type                       | Default Value     | Description                                       |
|---------------------|----------------------------|-------------------|---------------------------------------------------|
| `sock.port`         | `int`                      | `1234`            | Local UDP port used to receive datagrams.         |
| `sock.buffer_size`  | `int`                      | `1024`            | Receive buffer size in bytes.                     |
| `sock.dst_hosts`    | `std::vector<std::string>` | `["127.0.0.1"]`   | Destination host list used for outgoing packets.  |
| `sock.dst_ports`    | `std::vector<int>`         | `[4321]`          | Destination UDP port list (must match hosts size).|

### Serial Connection

| Parameter Name                | Type          | Default Value  | Description                                                  |
|------------------------------|---------------|----------------|--------------------------------------------------------------|
| `serial.port`                | `std::string` | `"/dev/ttyUSB0"` | Serial device path.                                        |
| `serial.baud_rate`           | `int`         | `38400`        | UART baud rate.                                              |
| `serial.buffer_size`         | `int`         | `1024`         | Read buffer size in bytes.                                   |
| `serial.character_size`      | `int`         | `8`            | Number of data bits per frame.                               |
| `serial.stop_bits`           | `int`         | `1`            | Number of stop bits.                                         |
| `serial.read_timeout_ms`     | `int`         | `50`           | Timeout before partial frame publish (milliseconds).         |
| `serial.end_of_frame_byte`   | `byte`        | `0xAA`         | Byte delimiter when byte framing is enabled.                 |
| `serial.end_of_frame_ascii`  | `std::string` | `"\n"`         | ASCII delimiter when ASCII framing is enabled.               |
| `serial.use_end_of_frame_byte`  | `bool`     | `true`         | Use byte delimiter framing mode.                             |
| `serial.use_end_of_frame_ascii` | `bool`     | `false`        | Use ASCII delimiter framing mode.                            |

### MQTT Connection

| Parameter Name      | Type                       | Default Value                     | Description                                |
|---------------------|----------------------------|-----------------------------------|--------------------------------------------|
| `client.host`       | `std::string`              | `"localhost"`                     | MQTT broker hostname/IP.                   |
| `client.port`       | `int`                      | `1883`                            | MQTT broker TCP port.                      |
| `client.topics`     | `std::vector<std::string>` | `["topic1", "topic2", "topic3"]`  | Topics to subscribe/publish through bridge.|
| `client.keep_alive` | `int`                      | `60`                              | MQTT keep-alive period in seconds.         |

### Modbus Connection

`ModbusNode` is a base class (for derived nodes) and defines these defaults:

| Parameter Name                    | Type          | Default Value      | Description                                          |
|-----------------------------------|---------------|--------------------|------------------------------------------------------|
| `timers.poll_ms`                  | `int`         | `500`              | Poll interval in milliseconds.                       |
| `timers.connect_ms`               | `int`         | `1000`             | Reconnect attempt interval in milliseconds.          |
| `timers.write_ms`                 | `int`         | `10`               | Write interval in milliseconds (for derived logic).  |
| `connection.type`                 | `std::string` | `"tcp"`            | Connection type (`tcp` supported).                   |
| `connection.ip`                   | `std::string` | `"192.168.52.209"` | Modbus TCP server IP address.                        |
| `connection.network_port`         | `int`         | `502`              | Modbus TCP port.                                     |
| `connection.response_timeout_ms`  | `int`         | `500`              | Modbus response timeout in milliseconds.             |
| `connection.slave_id`             | `int`         | `1`                | Modbus slave/unit ID.                                |

---

## Modbus Library

`roship_io` provides a `ModbusNode` base class for building device-specific Modbus TCP polling nodes. It handles connection, reconnection, and poll timing. Subclass it and override `onPoll()`: See `docs/1_modbus_usage.md` for a step-by-step example.

---

## License

Apache 2.0 — see [LICENSE](LICENSE).

## Maintainer

Kristopher Krasnosky — [support@seaward.science](mailto:support@seaward.science)
