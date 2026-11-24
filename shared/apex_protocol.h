#pragma once

// APEX META-1 protocol v1. Wire format is defined byte-by-byte; never transmit
// a C/C++ struct directly because padding and alignment vary by compiler.

#include <stddef.h>
#include <stdint.h>

namespace apex {

constexpr size_t kFrameSize = 32;
constexpr uint8_t kMagic0 = 'A';
constexpr uint8_t kMagic1 = 'X';
constexpr uint8_t kProtocolVersion = 1;
constexpr uint8_t kMotionCommand = 0x10;
constexpr uint8_t kTelemetryFast = 0x80;
constexpr uint8_t kTelemetryLeg = 0x81;

enum class ControlMode : uint8_t {
  Idle = 0,
  ManualServo = 1,
  Gait = 2,
  Auto = 3,
  Calibration = 4,
};

enum CommandFlags : uint8_t {
  RequestArm = 1u << 0,
  ClearFailsafe = 1u << 1,
  GimbalEnable = 1u << 2,
  OutputDisable = 1u << 3,
};

struct MotionCommand {
  uint16_t sequence;
  uint16_t session_id;
  uint8_t flags;
  ControlMode mode;
  int16_t joy_x_milli;
  int16_t joy_y_milli;
  int16_t joy_r_milli;
  int16_t gait_z_mm;
  uint16_t step_length_mm;
  uint16_t step_lift_mm;
  uint16_t cycle_ms;
  int16_t body_pitch_cdeg;
  int16_t body_roll_cdeg;
  int16_t body_yaw_cdeg;
};

inline uint16_t read_u16_le(const uint8_t* p) {
  return static_cast<uint16_t>(p[0]) |
         static_cast<uint16_t>(static_cast<uint16_t>(p[1]) << 8);
}

inline int16_t read_i16_le(const uint8_t* p) {
  return static_cast<int16_t>(read_u16_le(p));
}

inline void write_u16_le(uint8_t* p, uint16_t value) {
  p[0] = static_cast<uint8_t>(value & 0xFFu);
  p[1] = static_cast<uint8_t>((value >> 8) & 0xFFu);
}

inline uint16_t crc16_ccitt(const uint8_t* data, size_t length) {
  uint16_t crc = 0xFFFFu;
  for (size_t i = 0; i < length; ++i) {
    crc ^= static_cast<uint16_t>(data[i]) << 8;
    for (uint8_t bit = 0; bit < 8; ++bit) {
      crc = (crc & 0x8000u) ? static_cast<uint16_t>((crc << 1) ^ 0x1021u)
                            : static_cast<uint16_t>(crc << 1);
    }
  }
  return crc;
}

inline bool is_newer_sequence(uint16_t candidate, uint16_t previous) {
  const uint16_t delta = static_cast<uint16_t>(candidate - previous);
  return delta != 0u && delta < 0x8000u;
}

inline bool valid_frame(const uint8_t* frame, uint8_t expected_type) {
  if (frame[0] != kMagic0 || frame[1] != kMagic1 ||
      frame[2] != kProtocolVersion || frame[3] != expected_type) {
    return false;
  }
  return read_u16_le(frame + 30) == crc16_ccitt(frame, 30);
}

inline bool decode_motion(const uint8_t* frame, MotionCommand& out) {
  if (!valid_frame(frame, kMotionCommand)) return false;
  const uint8_t mode = frame[9];
  if (mode > static_cast<uint8_t>(ControlMode::Calibration)) return false;

  out.sequence = read_u16_le(frame + 4);
  out.session_id = read_u16_le(frame + 6);
  out.flags = frame[8];
  out.mode = static_cast<ControlMode>(mode);
  out.joy_x_milli = read_i16_le(frame + 10);
  out.joy_y_milli = read_i16_le(frame + 12);
  out.joy_r_milli = read_i16_le(frame + 14);
  out.gait_z_mm = read_i16_le(frame + 16);
  out.step_length_mm = read_u16_le(frame + 18);
  out.step_lift_mm = read_u16_le(frame + 20);
  out.cycle_ms = read_u16_le(frame + 22);
  out.body_pitch_cdeg = read_i16_le(frame + 24);
  out.body_roll_cdeg = read_i16_le(frame + 26);
  out.body_yaw_cdeg = read_i16_le(frame + 28);

  return out.joy_x_milli >= -1000 && out.joy_x_milli <= 1000 &&
         out.joy_y_milli >= -1000 && out.joy_y_milli <= 1000 &&
         out.joy_r_milli >= -1000 && out.joy_r_milli <= 1000 &&
         out.gait_z_mm >= -300 && out.gait_z_mm <= 0 &&
         out.step_length_mm <= 250 && out.step_lift_mm <= 180 &&
         out.cycle_ms >= 250 && out.cycle_ms <= 5000 &&
         out.body_pitch_cdeg >= -3000 && out.body_pitch_cdeg <= 3000 &&
         out.body_roll_cdeg >= -3000 && out.body_roll_cdeg <= 3000 &&
         out.body_yaw_cdeg >= -4500 && out.body_yaw_cdeg <= 4500;
}

}  // namespace apex
