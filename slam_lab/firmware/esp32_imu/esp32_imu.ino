#include <Wire.h>

// Generic ESP32 defaults. Explicit pins avoid board-package ambiguity.
constexpr int SDA_PIN = 21;
constexpr int SCL_PIN = 22;
constexpr uint32_t SERIAL_BAUD = 115200;
constexpr uint32_t SAMPLE_PERIOD_US = 10000;  // 100 Hz

constexpr uint8_t MAG_ADDR = 0x0C;  // AK8963 inside MPU-9250/9255

constexpr float GRAVITY = 9.80665f;
constexpr float ACCEL_LSB_PER_G = 8192.0f;   // +/-4 g
constexpr float GYRO_LSB_PER_DPS = 65.5f;    // +/-500 deg/s
constexpr float DEG_TO_RAD_F = 0.01745329251994329577f;

uint8_t imuWhoAmI = 0;
uint8_t imuAddress = 0;
bool imuReady = false;
bool magReady = false;
float magAdjust[3] = {1.0f, 1.0f, 1.0f};
uint32_t nextSampleUs = 0;
uint32_t nextProbeUs = 0;

enum class ProbeResult {
  READY,
  NO_ACK,
  READ_FAILED,
  UNSUPPORTED_ID,
  CONFIG_FAILED,
};

const char* probeName(ProbeResult result) {
  switch (result) {
    case ProbeResult::READY: return "ready";
    case ProbeResult::NO_ACK: return "no_ack_0x68_0x69";
    case ProbeResult::READ_FAILED: return "who_am_i_read_failed";
    case ProbeResult::UNSUPPORTED_ID: return "unsupported_who_am_i";
    case ProbeResult::CONFIG_FAILED: return "register_config_failed";
  }
  return "unknown";
}

bool i2cAck(uint8_t address) {
  Wire.beginTransmission(address);
  return Wire.endTransmission(true) == 0;
}

bool writeReg(uint8_t address, uint8_t reg, uint8_t value) {
  Wire.beginTransmission(address);
  Wire.write(reg);
  Wire.write(value);
  return Wire.endTransmission(true) == 0;
}

bool readRegs(uint8_t address, uint8_t reg, uint8_t* out, size_t count) {
  Wire.beginTransmission(address);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom(address, static_cast<uint8_t>(count), true) != count) {
    return false;
  }
  for (size_t i = 0; i < count; ++i) out[i] = Wire.read();
  return true;
}

bool readReg(uint8_t address, uint8_t reg, uint8_t& value) {
  return readRegs(address, reg, &value, 1);
}

int16_t be16(const uint8_t* p) {
  return static_cast<int16_t>((static_cast<uint16_t>(p[0]) << 8) | p[1]);
}

int16_t le16(const uint8_t* p) {
  return static_cast<int16_t>((static_cast<uint16_t>(p[1]) << 8) | p[0]);
}

bool initMagnetometer() {
  uint8_t who = 0;
  if (!readReg(MAG_ADDR, 0x00, who) || who != 0x48) return false;
  writeReg(MAG_ADDR, 0x0A, 0x00);
  delay(10);
  writeReg(MAG_ADDR, 0x0A, 0x0F);  // Fuse-ROM access
  delay(10);
  uint8_t asa[3]{};
  if (!readRegs(MAG_ADDR, 0x10, asa, 3)) return false;
  for (int i = 0; i < 3; ++i) {
    magAdjust[i] = ((static_cast<float>(asa[i]) - 128.0f) / 256.0f) + 1.0f;
  }
  writeReg(MAG_ADDR, 0x0A, 0x00);
  delay(10);
  return writeReg(MAG_ADDR, 0x0A, 0x16);  // 16-bit, continuous 100 Hz
}

ProbeResult initImu() {
  imuReady = false;
  magReady = false;
  imuAddress = 0;
  imuWhoAmI = 0;
  const bool ack68 = i2cAck(0x68);
  const bool ack69 = i2cAck(0x69);
  Serial.printf("I2C,scan,0x68=%s,0x69=%s\n",
                ack68 ? "ACK" : "NACK", ack69 ? "ACK" : "NACK");
  if (ack68) imuAddress = 0x68;
  else if (ack69) imuAddress = 0x69;
  else return ProbeResult::NO_ACK;

  if (!readReg(imuAddress, 0x75, imuWhoAmI)) {
    return ProbeResult::READ_FAILED;
  }
  Serial.printf("I2C,who_am_i,address=0x%02X,value=0x%02X\n",
                imuAddress, imuWhoAmI);
  // 0x70 is the documented MPU-6500 identity. Some modules sold as
  // "MPU-9250" contain this six-axis part; magnetometer remains optional.
  if (imuWhoAmI != 0x70 && imuWhoAmI != 0x71 && imuWhoAmI != 0x73) {
    return ProbeResult::UNSUPPORTED_ID;
  }

  if (!writeReg(imuAddress, 0x6B, 0x80)) return ProbeResult::CONFIG_FAILED;
  delay(100);
  if (!writeReg(imuAddress, 0x6B, 0x01) ||
      !writeReg(imuAddress, 0x1A, 0x03) ||
      !writeReg(imuAddress, 0x19, 0x09) ||  // 1 kHz / 10 = 100 Hz
      !writeReg(imuAddress, 0x1B, 0x08) ||  // Gyro +/-500 deg/s
      !writeReg(imuAddress, 0x1C, 0x08) ||  // Accel +/-4 g
      !writeReg(imuAddress, 0x1D, 0x03) ||
      !writeReg(imuAddress, 0x37, 0x02)) { // Host can reach AK8963
    return ProbeResult::CONFIG_FAILED;
  }
  delay(20);
  magReady = initMagnetometer();
  imuReady = true;
  return ProbeResult::READY;
}

void setup() {
  Serial.begin(SERIAL_BAUD);
  delay(300);
  // Use 100 kHz for first wiring diagnosis; 100 Hz packets fit comfortably.
  Wire.begin(SDA_PIN, SCL_PIN, 100000);
  Wire.setTimeOut(20);
  const ProbeResult result = initImu();

  const char* identity = imuWhoAmI == 0x70 ? "mpu6500_compatible" :
                         imuWhoAmI == 0x71 ? "mpu9250_compatible" :
                         imuWhoAmI == 0x73 ? "mpu9255_compatible" : "unknown";
  Serial.printf("META,imu_bridge_v3,address=0x%02X,who_am_i=0x%02X,identity=%s,imu=%s,mag=%s,"
                "accel=m/s2,gyro=rad/s,mag_unit=uT,timestamp=esp_us\n",
                imuAddress, imuWhoAmI, identity, probeName(result),
                magReady ? "ready" : "not_detected");
  if (!imuReady) {
    Serial.printf("ERROR,%s,check_3v3_gnd_gpio21_sda_gpio22_scl_ncs_high\n",
                  probeName(result));
  }
  nextSampleUs = micros();
  nextProbeUs = nextSampleUs + 5000000UL;
}

void loop() {
  const uint32_t now = micros();
  if (static_cast<int32_t>(now - nextSampleUs) < 0) return;
  nextSampleUs += SAMPLE_PERIOD_US;
  if (!imuReady) {
    if (static_cast<int32_t>(now - nextProbeUs) >= 0) {
      const ProbeResult result = initImu();
      Serial.printf("DIAG,imu=%s,address=0x%02X,who_am_i=0x%02X\n",
                    probeName(result), imuAddress, imuWhoAmI);
      nextProbeUs = micros() + 5000000UL;
      nextSampleUs = micros() + SAMPLE_PERIOD_US;
    }
    return;
  }

  uint8_t raw[14]{};
  if (!readRegs(imuAddress, 0x3B, raw, sizeof(raw))) {
    Serial.printf("ERROR,%lu,imu_i2c_read\n", static_cast<unsigned long>(now));
    return;
  }

  const int16_t rax = be16(raw + 0);
  const int16_t ray = be16(raw + 2);
  const int16_t raz = be16(raw + 4);
  const int16_t rt = be16(raw + 6);
  const int16_t rgx = be16(raw + 8);
  const int16_t rgy = be16(raw + 10);
  const int16_t rgz = be16(raw + 12);

  int16_t rmx = 0, rmy = 0, rmz = 0;
  float mx = NAN, my = NAN, mz = NAN;
  uint8_t mag[7]{};
  uint8_t st1 = 0;
  if (magReady && readReg(MAG_ADDR, 0x02, st1) && (st1 & 0x01) &&
      readRegs(MAG_ADDR, 0x03, mag, sizeof(mag)) && !(mag[6] & 0x08)) {
    rmx = le16(mag + 0);
    rmy = le16(mag + 2);
    rmz = le16(mag + 4);
    mx = rmx * 0.15f * magAdjust[0];
    my = rmy * 0.15f * magAdjust[1];
    mz = rmz * 0.15f * magAdjust[2];
  }

  const float ax = (rax / ACCEL_LSB_PER_G) * GRAVITY;
  const float ay = (ray / ACCEL_LSB_PER_G) * GRAVITY;
  const float az = (raz / ACCEL_LSB_PER_G) * GRAVITY;
  const float gx = (rgx / GYRO_LSB_PER_DPS) * DEG_TO_RAD_F;
  const float gy = (rgy / GYRO_LSB_PER_DPS) * DEG_TO_RAD_F;
  const float gz = (rgz / GYRO_LSB_PER_DPS) * DEG_TO_RAD_F;
  const float tempC = (rt / 333.87f) + 21.0f;

  Serial.printf("IMU,%lu,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,"
                "%.3f,%.3f,%.3f,%.3f,%d,%d,%d,%d,%d,%d,%d,%d,%d\n",
                static_cast<unsigned long>(now), ax, ay, az, gx, gy, gz,
                mx, my, mz, tempC, rax, ray, raz, rgx, rgy, rgz, rmx, rmy, rmz);
}
