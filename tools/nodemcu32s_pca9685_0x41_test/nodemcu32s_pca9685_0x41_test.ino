#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

// NodeMCU-32S / klasik ESP32 I2C pinleri.
// META-1 ESP32-S3 firmware'indeki GPIO8/9 bu kart icin kullanilmaz.
constexpr uint8_t I2C_SDA_PIN = 21;
constexpr uint8_t I2C_SCL_PIN = 22;
constexpr uint8_t PCA_ADDRESS = 0x41;
constexpr uint8_t TEST_CHANNEL = 0;

constexpr float SERVO_FREQUENCY_HZ = 50.0f;
constexpr uint16_t SERVO_MIN_US = 500;
constexpr uint16_t SERVO_MAX_US = 2500;
constexpr float MIN_TEST_ANGLE = 5.0f;
constexpr float MAX_TEST_ANGLE = 175.0f;

Adafruit_PWMServoDriver pca(PCA_ADDRESS);
bool pcaOnline = false;

uint16_t microsecondsToTick(uint16_t pulseUs) {
  // 50 Hz'de bir PCA9685 tick'i: 20,000 us / 4096 = 4.8828125 us.
  const float ticks = pulseUs * SERVO_FREQUENCY_HZ * 4096.0f / 1000000.0f;
  return (uint16_t)constrain((int)lroundf(ticks), 1, 4095);
}

void outputOff() {
  pca.setPWM(TEST_CHANNEL, 0, 0);
  Serial.println("[OFF] CH0 PWM kapali");
}

void setPulseUs(uint16_t pulseUs) {
  pulseUs = constrain(pulseUs, SERVO_MIN_US, SERVO_MAX_US);
  const uint16_t tick = microsecondsToTick(pulseUs);
  pca.setPWM(TEST_CHANNEL, 0, tick);
  Serial.printf("[PWM] CH0 = %u us, tick=%u\n", pulseUs, tick);
}

void setAngle(float angleDeg) {
  angleDeg = constrain(angleDeg, MIN_TEST_ANGLE, MAX_TEST_ANGLE);
  const uint16_t pulseUs = SERVO_MIN_US +
    (uint16_t)((angleDeg / 180.0f) * (SERVO_MAX_US - SERVO_MIN_US));
  Serial.printf("[ANGLE] %.1f derece -> ", angleDeg);
  setPulseUs(pulseUs);
}

bool probeAddress(uint8_t address) {
  Wire.beginTransmission(address);
  return Wire.endTransmission() == 0;
}

void scanI2C() {
  Serial.println("[SCAN] I2C taraniyor...");
  uint8_t found = 0;
  for (uint8_t address = 1; address < 127; ++address) {
    if (probeAddress(address)) {
      Serial.printf("  bulundu: 0x%02X%s\n", address,
                    address == PCA_ADDRESS ? " <- beklenen PCA9685" : "");
      ++found;
    }
  }
  if (found == 0) Serial.println("  HIC CIHAZ YOK: SDA/SCL/VCC/GND kontrol et");
}

void printHelp() {
  Serial.println();
  Serial.println("Komutlar (115200 baud, Newline):");
  Serial.println("  scan       I2C adreslerini tara");
  Serial.println("  center     CH0'a 1500 us gonder");
  Serial.println("  a 90       CH0'i dereceyle sur (5..175)");
  Serial.println("  u 1500     CH0'i mikrosaniyeyle sur (500..2500)");
  Serial.println("  off        CH0 PWM'i kapat");
  Serial.println("Acilista servo hareketi YOKTUR; once scan komutunu kullan.");
}

void setup() {
  Serial.begin(115200);
  delay(500);

  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN, 100000);
  scanI2C();
  pcaOnline = probeAddress(PCA_ADDRESS);
  if (!pcaOnline) {
    Serial.println("[HATA] PCA9685 0x41 bulunamadi. PWM baslatilmadi.");
    printHelp();
    return;
  }

  pca.begin();
  pca.setPWMFreq(SERVO_FREQUENCY_HZ);
  delay(10);
  outputOff();
  Serial.println("[OK] PCA9685 0x41 online, 50 Hz, CH0 guvenli OFF");
  printHelp();
}

void loop() {
  if (!Serial.available()) return;
  String command = Serial.readStringUntil('\n');
  command.trim();
  command.toLowerCase();

  if (command == "scan") {
    scanI2C();
    pcaOnline = probeAddress(PCA_ADDRESS);
    return;
  }
  if (!pcaOnline) {
    Serial.println("[HATA] 0x41 offline; yalniz scan kullanilabilir");
    return;
  }
  if (command == "off") {
    outputOff();
  } else if (command == "center") {
    setPulseUs(1500);
  } else if (command.startsWith("a ")) {
    setAngle(command.substring(2).toFloat());
  } else if (command.startsWith("u ")) {
    setPulseUs((uint16_t)command.substring(2).toInt());
  } else {
    printHelp();
  }
}
