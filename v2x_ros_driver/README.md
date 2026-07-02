# v2x_ros_driver

The v2x-ros-driver is a ros2 package currently implemented in [ros2-humble](https://docs.ros.org/en/humble/Installation.html), but previous releases support [ros2-foxy](https://docs.ros.org/en/foxy/Installation.html). It provides the bidirectional communication between ROS and a connected V2X radio (both OBUs and RSUs) using [carma_driver_msgs/msg/ByteArray](https://github.com/usdot-fhwa-stol/carma-msgs/blob/develop/carma_driver_msgs/msg/ByteArray.msg) ROS messages carrying UPER-encoded V2X data.

---

## ROS Topics

### Published

- `/inbound_binary_msg` : carma_driver_msgs/msg/ByteArray - An UPER-encoded V2X message received by the V2X radio.

### Subscribed

- `/outbound_binary_msg` : carma_driver_msgs/msg/ByteArray - An UPER encoded V2X message intended to be transmitted by the V2X radio.

---

## Parameters

Parameters can be set in config/params.yaml.

`protocol` (string) - The communication backend to use. This should be one of "udp" (for Cohda/Commsignia/Kapsch OBUs) or "mqtt" (for Ettifos OBUs). Defaults to "udp".

### When `protocol` is set to "udp":

- `v2x_radio_address` (string) - The IP address that the V2X radio is on. Defaults to "192.168.88.40".
- `v2x_radio_listening_port` (int) - The port on which the V2X radio receives outbound messages. Defaults to 1516.
- `listening_port` (int) - The port on which to receive incoming messages from the V2X radio. Defaults to 5398.

### When `protocol` is set to "mqtt":

- `broker_address` (string) - The IP address or hostname of the MQTT broker. Defaults to "192.168.88.40".
- `broker_port` (int) - The port on which to connect to the MQTT broker. Defaults to 1883.
- `client_id` (string) - The MQTT client ID to use for the connection. Defaults to "".
- `qos_default` (int) - The default Quality of Service level to use for MQTT. Defaults to 0.
- `reconnect_interval_sec` (int) - The time to wait, in seconds, before reconnecting to the MQTT broker after a disconnect. Defaults to 5.
- `mqtt_topic_prefix` (string) - The topic prefix to use for MQTT messages. Defaults to "Ettifos/V2X".

---

## IEEE 1609.2 Security Header Stripping

Incoming messages may be wrapped in an IEEE 1609.2 security envelope (signed SPDU containing a certificate and signature). The driver scans each datagram for a recognisable envelope and, if found, strips it to expose the inner J2735 `MessageFrame` before further processing. **The cryptographic signature is not validated** — the envelope is removed purely to allow message-ID matching and size checks to proceed on the bare payload.

### Latency measurement

Because this scan runs on every inbound message, `BaseRadioClient` records per-call timing via `std::chrono::steady_clock`. A running summary (call count, average, and maximum duration in nanoseconds) is emitted at `DEBUG` log level, throttled to once every 5 seconds. There is no output at the default `WARN` level. To enable it:

```sh
ros2 launch v2x_ros_driver v2x_ros_driver.launch.py log_level:=DEBUG
```

---

## Deployment Instructions

### Deploy using docker (recommended)

1. Pull the latest docker image for driver from dockerhub.

```sh
docker pull usdotfhwastol/v2x-ros-driver:<latest-release-tag>
```

*Latest release tag can be obtained from github tags. Docker images are tagged with the same tag.*

#### Note: Versions up to carma-system-4.5.0 (ros2-foxy supported) can all be found under [usdotfhwastol/carma-cohda-dsrc-driver<release-tag>](https://hub.docker.com/r/usdotfhwastol/carma-cohda-dsrc-driver)

2. Run the Docker image.

```sh
docker run -it --network host usdotfhwastol/v2x-ros-driver:<latest_release_tag_from_github>
```

---

### Build from source

#### Note: Assumption here is that user is building on a ros2 humble development environment

1. Clone the repository into workspace.

```sh
git clone https://github.com/usdot-fhwa-stol/v2x-ros-driver.git
```

2. Clone the dependencies into workspace.

```sh
chmod +x <path_to_workspace>/docker/checkout.bash
./<path_to_workspace>/docker/checkout.bash -r <path_to_workspace> -b <latest_release_tag_from_github>
```

3. Build the package.

```sh
source /opt/ros/humble/setup.bash
colcon build --packages-up-to v2x_ros_driver
```

4. Launch the node.

```sh
source <path_to_package_install_directory>/install/setup.bash
ros2 launch v2x_ros_driver v2x_ros_driver.launch.py
```
