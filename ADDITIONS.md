# Fork additions — LiDAR control link for the field dashboard

This is a fork of [Livox-SDK/livox_ros2_driver](https://github.com/Livox-SDK/livox_ros2_driver)
with one addition: a **control link** so the field mapping dashboard can read the
Avia's live device status and push configuration to it. The Livox SDK allows a
single client connection to the LiDAR, and the driver owns it — so this has to
live inside the driver.

Everything else is unchanged and it builds/runs exactly like upstream.

## What was added

All in `livox_ros2_driver/livox_ros2_driver/`:

- **`lidar_link.h` / `lidar_link.cpp`** — a small self-contained bridge. Holds a
  mutex-protected snapshot of the device status, serialises it to JSON, and
  turns dashboard commands into real SDK calls.
- **`lds_lidar.cpp`** — the existing SDK callbacks now also feed the bridge:
  work state (`OnDeviceChange`), firmware (`DeviceInformationCb`) and the health
  bits — temperature, voltage, motor, dust, service life, PPS/time-sync
  (`LidarErrorStatusCb`). The data was already arriving; upstream only printed it
  every 100th message.
- **`livox_ros2_driver.cpp` / `.h`** — the node now:
  - publishes `std_msgs/String` JSON on **`/livox/lidar_status`** at ~2 Hz;
  - subscribes to **`/livox/lidar_cmd`** and applies commands via
    `LidarSetMode` (work mode / spin up-down), `LidarSetPointCloudReturnMode`
    (echo), `LidarSetImuPushFrequency`, `LidarSetScanPattern` (Avia),
    `SetCartesian/SphericalCoordinate`, and high-sensitivity enable/disable.

## Command format

Publish a JSON object on `/livox/lidar_cmd`; any subset of keys is applied:

```json
{"work_mode":"Standby","echo_type":"Dual Return","imu_freq":"200 Hz",
 "scan_mode":"Repetitive Line","coordinate":"Cartesian",
 "high_sensitivity":"Enabled"}
```

Values are matched loosely (case-insensitive substrings) so the dashboard's
label variants all map correctly.

Upstream license (MIT) and all original headers are retained.
