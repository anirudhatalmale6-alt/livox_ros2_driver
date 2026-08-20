// The heap overflow that killed the driver on every return-mode change.
//
// From the client's unit, 20 Aug 2026, twice in ten minutes:
//   lidar_cmd: sent to LiDAR: echo_type=Double Return
//   realloc(): invalid old size            -> exit code -6  (SIGABRT)
//   ...
//   lidar_cmd: sent to LiDAR: echo_type=Double Return
//   process has died ...                      exit code -11 (SIGSEGV)
//
// Lddc chose the point-convert handler from lidar->raw_data_type, but each
// handler sizes its write loop from eth_packet->data_type. Those are the same
// thing right up until the moment a return-mode change is in flight: the
// LiDAR's type has already moved to dual while the queue still holds packets
// of the old single-return type.
//
// Feed a SINGLE-return packet to the DUAL handler and it reads 96 points per
// packet from the packet, then doubles it because it is the dual handler -
// 192 writes into a buffer budgeted at 100. That is the corruption.
//
// This models the arithmetic exactly, with no ROS or PCL needed.
#include <cstdint>
#include <cstdio>
#include <string>
#include <vector>

namespace {

const uint32_t kMaxPointPerEthPacket = 100;
const uint32_t kMaxPointDataType = 9;

struct Info { uint32_t points_per_packet; uint32_t echo_num; };

// data_type_info_pair_table from lds.h (points_per_packet, echo_num only).
const Info kTable[kMaxPointDataType] = {
    {100, 1},   // 0 cartesian raw
    {100, 1},   // 1 spherical
    { 96, 1},   // 2 extend cartesian   <- the client's single-return type
    { 96, 1},   // 3 extend spherical
    { 48, 2},   // 4 dual extend cart   <- Double Return
    { 48, 2},   // 5 dual extend spher
    {  1, 1},   // 6 imu
    { 30, 3},   // 7 triple extend cart
    { 30, 3},   // 8 triple extend spher
};

uint32_t PointsPerPacket(uint8_t t) { return kTable[t].points_per_packet; }
uint32_t EchoNum(uint8_t t) { return kTable[t].echo_num; }

// How many points a handler writes. Each handler reads points_per_packet from
// the PACKET, then multiplies by its own fixed echo count - the dual handler
// does `points_per_packet * 2`, the triple `* 3`. So the write count is
// (points-per-packet of the PACKET) x (echo of the HANDLER).
uint32_t WritesOf(uint8_t handler_type, uint8_t packet_type) {
  return PointsPerPacket(packet_type) * EchoNum(handler_type);
}

// OLD: handler picked from the lidar's current type.
uint32_t OldWrites(uint8_t lidar_type, uint8_t packet_type) {
  return WritesOf(lidar_type, packet_type);
}

// NEW: handler picked from the packet's own type, so the two always agree.
uint32_t NewWrites(uint8_t /*lidar_type*/, uint8_t packet_type) {
  return WritesOf(packet_type, packet_type);
}

int failures = 0;
void check(const std::string & label, bool cond) {
  std::printf("  [%s] %s\n", cond ? "ok" : "FAIL", label.c_str());
  if (!cond) { ++failures; }
}

}  // namespace

int main() {
  std::printf("the return-mode change overflow\n\n");

  std::printf("steady state - both agree, and nothing overflows\n");
  for (uint8_t t = 0; t < kMaxPointDataType; ++t) {
    if (t == 6) { continue; }                 // imu has no convert handler
    char buf[96];
    std::snprintf(buf, sizeof(buf),
                  "type %u writes %u of %u budget", t,
                  NewWrites(t, t), kMaxPointPerEthPacket);
    check(buf, NewWrites(t, t) <= kMaxPointPerEthPacket);
  }

  std::printf("\nmid-change - the LiDAR has moved on, the queue has not\n");
  const uint8_t kSingle = 2, kDual = 4, kTriple = 7;

  std::printf("  OLD behaviour:\n");
  check("single packet + dual handler overruns the budget",
        OldWrites(kDual, kSingle) > kMaxPointPerEthPacket);
  std::printf("        (writes %u into a %u-point budget - %.1fx over)\n",
              OldWrites(kDual, kSingle), kMaxPointPerEthPacket,
              double(OldWrites(kDual, kSingle)) / kMaxPointPerEthPacket);
  check("single packet + triple handler overruns too",
        OldWrites(kTriple, kSingle) > kMaxPointPerEthPacket);
  std::printf("        (writes %u - %.1fx over)\n",
              OldWrites(kTriple, kSingle),
              double(OldWrites(kTriple, kSingle)) / kMaxPointPerEthPacket);

  std::printf("  NEW behaviour:\n");
  check("single packet mid-change stays in budget",
        NewWrites(kDual, kSingle) <= kMaxPointPerEthPacket);
  check("dual packet arriving while lidar still says single stays in budget",
        NewWrites(kSingle, kDual) <= kMaxPointPerEthPacket);
  check("triple packet mid-change stays in budget",
        NewWrites(kSingle, kTriple) <= kMaxPointPerEthPacket);

  std::printf("\nevery mixed pair is safe under the new rule\n");
  bool all_safe = true;
  uint32_t worst_old = 0;
  for (uint8_t l = 0; l < kMaxPointDataType; ++l) {
    for (uint8_t p = 0; p < kMaxPointDataType; ++p) {
      if (l == 6 || p == 6) { continue; }
      if (NewWrites(l, p) > kMaxPointPerEthPacket) { all_safe = false; }
      if (OldWrites(l, p) > worst_old) { worst_old = OldWrites(l, p); }
    }
  }
  check("all 64 lidar/packet combinations stay within budget", all_safe);
  std::printf("        (worst case under the OLD rule was %u points into %u)\n",
              worst_old, kMaxPointPerEthPacket);

  std::printf("\n%s (%d failure(s))\n",
              failures ? "FAILED" : "all passed", failures);
  return failures ? 1 : 0;
}
