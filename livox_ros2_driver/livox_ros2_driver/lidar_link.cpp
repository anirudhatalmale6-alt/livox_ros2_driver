//
// lidar_link.cpp - see lidar_link.h.
//
// Keeps one mutex-protected snapshot of the Avia's live status, serialises it
// to JSON for the dashboard, and turns dashboard config commands into real
// Livox SDK calls on the connected handle.
//
#include "lidar_link.h"

#include <cstdio>
#include <cctype>
#include <mutex>

#include "livox_sdk.h"
#include "rapidjson/document.h"

namespace livox_ros {

namespace {

struct LinkState {
  std::mutex m;
  bool present = false;         // have we ever seen this device?
  int handle = -1;
  std::string bcode;
  int work_state = 5;           // 5 = Unknown until the driver tells us
  bool fw_valid = false;
  uint8_t fw[4] = {0, 0, 0, 0};
  bool err_valid = false;
  uint8_t temp = 0, volt = 0, motor = 0, dirty = 0, firmware_err = 0,
          service_life = 0, fan = 0, self_heating = 0, ptp = 0, time_sync = 0,
          system = 0;
};

LinkState g_link;

// ---- control-ack callbacks (the SDK calls these back asynchronously) ----
void CommonCb(livox_status status, uint8_t handle, uint8_t response, void *) {
  printf("[lidar_link] command ack handle=%u status=%d response=%u\n", handle,
         status, response);
}
void ParamCb(livox_status status, uint8_t handle, DeviceParameterResponse *,
             void *) {
  printf("[lidar_link] param ack handle=%u status=%d\n", handle, status);
}
void StartSampleCb(livox_status status, uint8_t handle, uint8_t response,
                   void *) {
  printf("[lidar_link] restart sampling ack handle=%u status=%d response=%u\n",
         handle, status, response);
}
// When we command the lidar back to Normal, it spins the head up but does NOT
// resume point-cloud sampling on its own - so restart sampling once Normal is
// acknowledged. This is what lets "Working Normally" + APPLY recover a unit that
// was put into Standby/Power Saving, with no power-cycle.
void SetNormalCb(livox_status status, uint8_t handle, uint8_t response, void *) {
  printf("[lidar_link] set-mode(Normal) ack handle=%u status=%d response=%u\n",
         handle, status, response);
  if (status == kStatusSuccess) {
    LidarStartSampling(handle, StartSampleCb, nullptr);
  }
}

std::string Lower(const std::string &s) {
  std::string o = s;
  for (char &c : o) c = static_cast<char>(std::tolower((unsigned char)c));
  return o;
}
bool Has(const std::string &h, const char *needle) {
  return h.find(needle) != std::string::npos;
}

// Pull a string value out of the command doc (accepts a couple of aliases).
bool GetStr(const rapidjson::Document &d, const char *key, std::string &out) {
  if (d.HasMember(key) && d[key].IsString()) {
    out = d[key].GetString();
    return true;
  }
  return false;
}

}  // namespace

/** ---- status ingest ---- */

void LidarLinkSetHandle(uint8_t handle, const char *broadcast_code) {
  std::lock_guard<std::mutex> lock(g_link.m);
  g_link.present = true;
  g_link.handle = handle;
  if (broadcast_code) g_link.bcode = broadcast_code;
}

void LidarLinkSetWorkState(uint8_t handle, int state) {
  std::lock_guard<std::mutex> lock(g_link.m);
  g_link.present = true;
  g_link.handle = handle;
  g_link.work_state = state;
}

void LidarLinkSetFirmware(uint8_t handle, uint8_t a, uint8_t b, uint8_t c,
                          uint8_t d) {
  std::lock_guard<std::mutex> lock(g_link.m);
  g_link.handle = handle;
  g_link.fw_valid = true;
  g_link.fw[0] = a; g_link.fw[1] = b; g_link.fw[2] = c; g_link.fw[3] = d;
}

void LidarLinkSetError(uint8_t handle, uint8_t temp, uint8_t volt,
                       uint8_t motor, uint8_t dirty, uint8_t firmware_err,
                       uint8_t service_life, uint8_t fan, uint8_t self_heating,
                       uint8_t ptp, uint8_t time_sync, uint8_t system) {
  std::lock_guard<std::mutex> lock(g_link.m);
  g_link.present = true;
  g_link.handle = handle;
  g_link.err_valid = true;
  g_link.temp = temp; g_link.volt = volt; g_link.motor = motor;
  g_link.dirty = dirty; g_link.firmware_err = firmware_err;
  g_link.service_life = service_life; g_link.fan = fan;
  g_link.self_heating = self_heating; g_link.ptp = ptp;
  g_link.time_sync = time_sync; g_link.system = system;
}

/** ---- status egress ---- */

std::string LidarLinkStatusJson() {
  std::lock_guard<std::mutex> lock(g_link.m);
  char buf[512];
  char fw[32] = "unknown";
  if (g_link.fw_valid) {
    snprintf(fw, sizeof(fw), "%u.%u.%u.%u", g_link.fw[0], g_link.fw[1],
             g_link.fw[2], g_link.fw[3]);
  }
  snprintf(buf, sizeof(buf),
           "{\"present\":%s,\"handle\":%d,\"broadcast_code\":\"%s\","
           "\"work_state\":%d,\"firmware\":\"%s\",\"err_valid\":%s,"
           "\"temp\":%u,\"volt\":%u,\"motor\":%u,\"dirty\":%u,"
           "\"firmware_err\":%u,\"service_life\":%u,\"fan\":%u,"
           "\"self_heating\":%u,\"ptp\":%u,\"time_sync\":%u,\"system\":%u}",
           g_link.present ? "true" : "false", g_link.handle,
           g_link.bcode.c_str(), g_link.work_state, fw,
           g_link.err_valid ? "true" : "false", g_link.temp, g_link.volt,
           g_link.motor, g_link.dirty, g_link.firmware_err, g_link.service_life,
           g_link.fan, g_link.self_heating, g_link.ptp, g_link.time_sync,
           g_link.system);
  return std::string(buf);
}

/** ---- control ---- */

bool LidarLinkApply(const std::string &cmd_json, std::string &result) {
  uint8_t handle;
  {
    std::lock_guard<std::mutex> lock(g_link.m);
    if (!g_link.present || g_link.handle < 0) {
      result = "no LiDAR connected yet - command ignored";
      return false;
    }
    handle = static_cast<uint8_t>(g_link.handle);
  }

  rapidjson::Document d;
  if (d.Parse(cmd_json.c_str()).HasParseError() || !d.IsObject()) {
    result = "bad command payload (not JSON object)";
    return false;
  }

  std::string applied;
  int count = 0;
  std::string v;

  // Work mode: Normal / Power Saving / Standby (spin the head up/down).
  if (GetStr(d, "work_mode", v)) {
    std::string lv = Lower(v);
    LidarMode mode = kLidarModeNormal;
    if (Has(lv, "standby")) mode = kLidarModeStandby;
    else if (Has(lv, "power")) mode = kLidarModePowerSaving;
    else mode = kLidarModeNormal;  // "normal" / "working"
    // Returning to Normal also needs sampling restarted (SetNormalCb does it);
    // Standby / Power Saving just change mode.
    CommonCommandCallback cb = (mode == kLidarModeNormal) ? SetNormalCb : CommonCb;
    livox_status st = LidarSetMode(handle, mode, cb, nullptr);
    applied += (count++ ? ", " : "") + std::string("work_mode=") + v +
               (st == kStatusSuccess ? "" : "(send-fail)");
  }

  // Echo / return type.
  if (GetStr(d, "echo_type", v)) {
    std::string lv = Lower(v);
    PointCloudReturnMode rm = kFirstReturn;
    if (Has(lv, "triple")) rm = kTripleReturn;
    else if (Has(lv, "dual") || Has(lv, "double")) rm = kDualReturn;
    else if (Has(lv, "strong")) rm = kStrongestReturn;
    else rm = kFirstReturn;  // "single" / "first"
    livox_status st =
        LidarSetPointCloudReturnMode(handle, rm, CommonCb, nullptr);
    applied += (count++ ? ", " : "") + std::string("echo_type=") + v +
               (st == kStatusSuccess ? "" : "(send-fail)");
  }

  // IMU push frequency: 0 Hz (off) or 200 Hz.
  if (GetStr(d, "imu_freq", v)) {
    std::string lv = Lower(v);
    ImuFreq f = Has(lv, "200") ? kImuFreq200Hz : kImuFreq0Hz;
    livox_status st = LidarSetImuPushFrequency(handle, f, CommonCb, nullptr);
    applied += (count++ ? ", " : "") + std::string("imu_freq=") + v +
               (st == kStatusSuccess ? "" : "(send-fail)");
  }

  // Scan pattern (Avia): non-repetitive (circular) or repetitive (line).
  if (GetStr(d, "scan_mode", v)) {
    std::string lv = Lower(v);
    // Check "non" first - "Non-repetitive" also contains "repet".
    LidarScanPattern p = Has(lv, "non") ? kNoneRepetitiveScanPattern
                                        : (Has(lv, "repet")
                                               ? kRepetitiveScanPattern
                                               : kNoneRepetitiveScanPattern);
    livox_status st = LidarSetScanPattern(handle, p, ParamCb, nullptr);
    applied += (count++ ? ", " : "") + std::string("scan_mode=") + v +
               (st == kStatusSuccess ? "" : "(send-fail)");
  }

  // Coordinate system.
  if (GetStr(d, "coordinate", v)) {
    std::string lv = Lower(v);
    livox_status st;
    if (Has(lv, "spher")) st = SetSphericalCoordinate(handle, CommonCb, nullptr);
    else st = SetCartesianCoordinate(handle, CommonCb, nullptr);
    applied += (count++ ? ", " : "") + std::string("coordinate=") + v +
               (st == kStatusSuccess ? "" : "(send-fail)");
  }

  // High sensitivity mode.
  if (GetStr(d, "high_sensitivity", v)) {
    std::string lv = Lower(v);
    livox_status st;
    if (Has(lv, "disable")) st = LidarDisableHighSensitivity(handle, ParamCb, nullptr);
    else st = LidarEnableHighSensitivity(handle, ParamCb, nullptr);
    applied += (count++ ? ", " : "") + std::string("high_sensitivity=") + v +
               (st == kStatusSuccess ? "" : "(send-fail)");
  }

  if (!count) {
    result = "no recognised settings in command";
    return false;
  }
  result = "sent to LiDAR: " + applied;
  return true;
}

}  // namespace livox_ros
