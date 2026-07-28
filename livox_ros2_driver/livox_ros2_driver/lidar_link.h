//
// lidar_link.h - control + status bridge added to the Livox driver.
//
// This is the extra piece the field dashboard needs. The driver owns the one
// and only connection to the Avia (the Livox SDK allows a single client), so
// both the live device status and the config commands have to travel through
// here.
//
//   * The SDK callbacks in lds_lidar.cpp feed the latest device status in
//     (LidarLinkSet* functions below) - no SDK types leak into this header so
//     the rest of the node stays clean.
//   * The driver node reads it back out as JSON (LidarLinkStatusJson) and
//     publishes it on /livox/lidar_status ~2 Hz.
//   * The driver node hands config commands received on /livox/lidar_cmd to
//     LidarLinkApply(), which calls the real SDK set-mode / return-mode /
//     imu-rate / scan-pattern / coordinate / high-sensitivity functions.
//
#ifndef LIVOX_ROS2_DRIVER_LIDAR_LINK_H_
#define LIVOX_ROS2_DRIVER_LIDAR_LINK_H_

#include <cstdint>
#include <string>

namespace livox_ros {

/** ---- status ingest: called from the SDK callbacks in lds_lidar.cpp ---- */

// Remember which handle / broadcast code is the live device.
void LidarLinkSetHandle(uint8_t handle, const char *broadcast_code);

// Work state, values are \ref LidarState (0 Init,1 Normal,2 PowerSaving,
// 3 Standby,4 Error,5 Unknown).
void LidarLinkSetWorkState(uint8_t handle, int state);

// Firmware version bytes from DeviceInformationCb.
void LidarLinkSetFirmware(uint8_t handle, uint8_t a, uint8_t b, uint8_t c,
                          uint8_t d);

// Health bits out of the periodic ErrorMessage. Each is the raw Livox
// severity (0 = normal, 1 = warning, 2 = severe) except time_sync (0 none,
// 1 PTP, 2 GPS, 3 PPS, 4 abnormal) and the 1-bit flags.
void LidarLinkSetError(uint8_t handle, uint8_t temp, uint8_t volt,
                       uint8_t motor, uint8_t dirty, uint8_t firmware_err,
                       uint8_t service_life, uint8_t fan, uint8_t self_heating,
                       uint8_t ptp, uint8_t time_sync, uint8_t system);

/** ---- status egress: called from the driver node's publish timer ---- */

// A compact JSON snapshot of the live device status for /livox/lidar_status.
std::string LidarLinkStatusJson();

/** ---- control: called from the driver node's /livox/lidar_cmd sub ---- */

// cmd_json is e.g. {"work_mode":"Standby","echo_type":"Dual Return",
// "imu_freq":"200 Hz","scan_mode":"Repetitive Line","coordinate":"Cartesian",
// "high_sensitivity":"Enabled"}. Any subset of keys is allowed. Returns a
// human-readable summary in `result`; false if nothing could be applied (no
// device yet, or a bad payload).
bool LidarLinkApply(const std::string &cmd_json, std::string &result);

}  // namespace livox_ros

#endif  // LIVOX_ROS2_DRIVER_LIDAR_LINK_H_
