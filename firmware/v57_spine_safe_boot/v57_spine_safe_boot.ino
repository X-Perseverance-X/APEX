/* =====================================================================
   APEX ROBOTICS - META-1 SPINE SAFE BOOT (v57.4)
   Base: v56_radio_sil_foundation

   Safety invariants:
     - Boot never enables a leg/arm PWM output.
     - Stored HOME data never starts motion by itself.
     - /goHome?confirm=start is the only compatibility path that starts the
       CAPTURE -> STAND sequence.
     - CAPTURE uses the repeatable chassis-rest pose described by the operator;
       the chassis must be resting on its mechanical support protrusions.
     - Pitch stabilization sign is independent from the field-proven roll sign.

   v53 Eksen standardı:
     ROBOT BODY FRAME: +X ileri, +Y sol, +Z yukarı (sağ-el kuralı).
     Bacak sırası: RF, RM, RR, LF, LM, LR.
     PCA 0x40 = sağ taraf, PCA 0x41 = sol taraf.
     Coxa: sağ aynalı, sol doğrudan.
     Femur: sağ doğrudan, sol aynalı.
     Tibia: mevcut davranış korunmuştur (sağ doğrudan, sol aynalı) — fiziksel testle doğrula.
     Body IK: ZYX Euler için doğru inverse rotation sırası kullanılır.
     Gimbal roll: ref - measured negatif geri besleme.
     Gait: joystick ileri = +X, joystick sağ = -Y.
===================================================================== */

#include <Arduino.h>
#include <Wire.h>
#include <WiFi.h>
#include <WebServer.h>
#include <Adafruit_PWMServoDriver.h>
#include <Preferences.h>
#include <TinyGPSPlus.h>
#include <math.h>
#include "esp32-hal-rgb-led.h"
#include "DFRobotDFPlayerMini.h"
#include "Adafruit_VL53L1X.h"

#define APEX_FIRMWARE_VERSION "v57.7-smooth-feedback"
#define APEX_BUILD_PROFILE    "esp32s3-n16r8-generic"

const char* ssid = "APEX-HUB";
const char* password = "apex_secure_2025";

WebServer server(80);

// --- ÇİFT SÜRÜCÜ & HAFIZA ---
Adafruit_PWMServoDriver pwmRight = Adafruit_PWMServoDriver(0x40);
Adafruit_PWMServoDriver pwmLeft = Adafruit_PWMServoDriver(0x41);
Preferences preferences;

// --- PIN TANIMLAMALARI ---
#define SDA_PIN 8
#define SCL_PIN 9
#define GPS_RX_PIN 16
#define GPS_TX_PIN 17

// --- BATARYA VOLTAJ MONITORU (ESP32-S3 ADC1) ---
// Sağ 3S LiPo voltage divider çıkışı -> GPIO1
// Sol 3S LiPo voltage divider çıkışı -> GPIO2
// Kullanıcının verdiği ölçek: yaklaşık 12.6 V pack -> 3.0 V ADC, oran ~= 4.2.
// 2026-07-05 saha kalibrasyonu: multimetre 12.60 V iken eski okuma 12.17 V.
// Düzeltme katsayısı = 12.60 / 12.17 = 1.03533. Sağ/sol ayrı sabit bırakıldı.
#define BAT_RIGHT_ADC_PIN 1
#define BAT_LEFT_ADC_PIN  2
const float BAT_RIGHT_DIVIDER_RATIO = 4.20f;
const float BAT_LEFT_DIVIDER_RATIO  = 4.20f;
float BAT_RIGHT_CAL_FACTOR          = 1.03533f;
float BAT_LEFT_CAL_FACTOR           = 1.03533f;

// --- TOF400C / VL53L1X ---
// Aynı I2C hattı: SDA GPIO8, SCL GPIO9, varsayılan adres 0x29.
Adafruit_VL53L1X tof400c;
bool tofReady = false;
int16_t tofDistanceMm = -1;
unsigned long lastTofUpdateMs = 0;

// Batarya telemetrisi
float batteryRightV = 0.0f;
float batteryLeftV  = 0.0f;
uint16_t batteryRightAdcMv = 0;
uint16_t batteryLeftAdcMv  = 0;
unsigned long lastBatteryUpdateMs = 0;

// I2C actuation prerequisites. scanI2C() owns these flags.
bool pcaRightOnline = false;
bool pcaLeftOnline  = false;
bool mpuOnline      = false;
bool tofAddressOnline = false;
unsigned long lastI2CRecoveryMs = 0;
const unsigned long I2C_RECOVERY_INTERVAL_MS = 2000UL;

// --- DFPlayer Mini (UART1: GPIO4=TX→DFPlayer RX, GPIO5=RX←DFPlayer TX) ---
#define DF_TX_PIN 4
#define DF_RX_PIN 5
HardwareSerial dfSerial(1);
DFRobotDFPlayerMini dfPlayer;
bool dfReady    = false;
int  dfVolume   = 15;
int  dfTrack    = 1;

// --- TELEMETRİ & GPS DEĞİŞKENLERİ ---
HardwareSerial gpsSerial(2);
TinyGPSPlus gps;
double currentLat = 41.0256, currentLng = 28.8895;
bool gpsHasFix = false;
String sysLogs = "[SYS] APEX OS BOOT SEQUENCE INITIATED...\n";
String bootDiagLog = "";  // Kalıcı boot tanısı — sysLogs gibi 2000 karakterde kırpılmaz, hep görünür kalır

// --- MPU6050 (GİMBAL) DEĞİŞKENLERİ ---
bool gimbalActive = false;
float mpuPitch = 0.0, mpuRoll = 0.0, mpuYaw = 0.0;
float s_mpuPitch = 0.0, s_mpuRoll = 0.0;
unsigned long lastMpuTime = 0;

float gimbalFilterLevel = 20.0f;  // eski: 80 (yavaş/yumuşak ucuna yakın) — bu,
                                    // deadband'i 1.66°'ye, alpha'yı 0.15'e çekip
                                    // küçük/orta tilt'lerde tepkiyi zayıflatıyordu.
                                    // 20 ile deadband ~0.64°, alpha ~0.30 — Stewart
                                    // platform benzeri daha keskin/hızlı tepki.
                                    // Artık apex_ui.html'de slider ile ayarlanabilir.

// --- HEXAPOD FİZİKSEL SABİTLERİ ---
const float a = 103.24;
const float b = 168.41;
const float l = 69.66;
const float MIN_DEGREE = 5.0;
const float MAX_DEGREE = 175.0;

const uint8_t legPins[6][3] = {
  {0, 1, 2},    {4, 5, 6},    {8, 10, 9},
  {0, 1, 2},    {4, 5, 6},    {8, 9, 10}
};

const float legPhase[6] = {0.0, 0.5, 0.0, 0.5, 0.0, 0.5};

// ROBOT BODY FRAME: +X ileri, +Y sol, +Z yukarı.
// Bacak sırası: RF, RM, RR, LF, LM, LR.
const float bodyOffsetX[6] = {120.0, 0.0, -120.0, 120.0, 0.0, -120.0};
const float bodyOffsetY[6] = {-80.0, -90.0, -80.0, 80.0, 90.0, 80.0};

// IK geometrik açısını elektriksel servo açısına çeviren yön haritası.
// +1: servo = IK, -1: servo = 180 - IK (90 derece etrafında ayna).
// Kullanıcının nihai mekanik tarifi:
//   COXA  sağ: 90->180 = +X  => aynalı (-1)
//   COXA  sol : 90->0   = +X  => doğrudan (+1)
//   FEMUR sağ: 0->180 yukarı  => doğrudan (+1)
//   FEMUR sol : tam tersi     => aynalı (-1)
// TIBIA yönü kullanıcı tarafından henüz fiziksel olarak tarif edilmedi; eski davranış korunuyor.
const int8_t jointServoDir[6][3] = {
  {-1, +1, +1}, {-1, +1, +1}, {-1, +1, +1},
  {+1, -1, -1}, {+1, -1, -1}, {+1, -1, -1}
};

static inline float mapIkToServoDeg(int leg, int joint, float ikDeg) {
  return 90.0f + (float)jointServoDir[leg][joint] * (ikDeg - 90.0f);
}


// --- 4 EKSEN ROBOT KOL (PCA9685 0x40 / SAĞ PCA BOŞ KANALLAR) ---
// Bacak kanallarına DOKUNULMADI. Sağ PCA (0x40) üzerinde bacaklar 0,1,2 / 4,5,6 / 8,9,10
// kullanıyor; 12-15 bloğu kol için ayrıldı.
// Kanal eşlemesi:
//   BASE    -> PCA 0x40 CH12  (90° = robot/kamera önü merkez, 0/180 sağ-sol uç)
//   JOINT1  -> PCA 0x40 CH13  (0° = örümceğin önüne, 180° = arkaya/gövde üstüne)
//   JOINT2  -> PCA 0x40 CH14  (0° = bekleme/katlı, 180° = dışarı açılma)
//   GRIPPER -> PCA 0x40 CH15  (varsayılan dar ve güvenli; kullanıcı ayarlar)
const uint8_t ARM_JOINT_COUNT = 4;
const uint8_t ARM_BASE    = 0;
const uint8_t ARM_JOINT1  = 1;
const uint8_t ARM_JOINT2  = 2;
const uint8_t ARM_GRIPPER = 3;
const uint8_t armPins[ARM_JOINT_COUNT] = {12, 13, 14, 15};
const char*   armNames[ARM_JOINT_COUNT] = {"base", "joint1", "joint2", "gripper"};

// Ana eklemler için kullanıcının istediği mutlak güvenli aralık: [5,175].
// Gripper rack-and-pinion: 90° açık, kapatmak için 90° -> 180° yönüne gider.
// Kapalı açı, kullanıcının bulup kaydettiği MAX değeridir.
float armMinDeg[ARM_JOINT_COUNT] = {5.0f, 5.0f, 5.0f, 90.0f};
float armMaxDeg[ARM_JOINT_COUNT] = {175.0f, 175.0f, 175.0f, 120.0f};

// Standart hobi servo pulse aralığı. Bacak kalibrasyonundan bağımsız tutuldu.
int armPwmMinUs[ARM_JOINT_COUNT] = {500, 500, 500, 500};
int armPwmMaxUs[ARM_JOINT_COUNT] = {2500, 2500, 2500, 2500};

// Elektriksel kalibrasyon pozu: horn takarken hepsi 90°.
float armCalibrationPose[ARM_JOINT_COUNT] = {90.0f, 90.0f, 90.0f, 90.0f};
// Mekanik bekleme: base önde, joint1 gövde üstüne, joint2 bekleme/katlı.
float armHomePose[ARM_JOINT_COUNT]        = {90.0f, 5.0f, 5.0f, 90.0f};

float armTargetAngle[ARM_JOINT_COUNT]  = {90.0f, 90.0f, 90.0f, 90.0f};
float armCurrentAngle[ARM_JOINT_COUNT] = {90.0f, 90.0f, 90.0f, 90.0f};
bool  armOutputEnabled = false;   // İlk komut gelene kadar kol servolarına PWM basılmaz.
unsigned long lastArmUpdateMs = 0;
const float ARM_MAX_STEP_DEG = 1.2f;  // 20ms döngüde ~60°/s; ani vuruş/dişli zorlamasını azaltır.

float armGripperOpenDeg  = 90.0f;
float armGripperCloseDeg = 120.0f;

// Gerçek hexapod duruş açıları (leg-local IK çerçevesinde bacak nötr X ofseti):
// Tüm bacaklar 90° nötr: simetrik duruş, maksimum yürüyüş hızı
const float legNeutralFwdOffset[6] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

int pwmMin[6][3], pwmMax[6][3], offset[6][3];

float gaitX = 0.0, gaitY = 150.0, gaitZ = -100.0;
// HIZ GÜNCELLEMESİ — eski (50,40,2000) ile tam joystick'te bile teorik max
// hız sadece ~25mm/s idi (stepLen/baseSpeed). IK uzanma sınırı (~271mm)
// hesaplanıp doğrulandı: en kötü durumda (tam ileri + tam dönüş üst üste)
// bile yeni değerler limitin sadece ~%42'sini kullanıyor — büyük pay var.
// Yeni teorik max hız: 100mm/1000ms = ~100mm/s (4 katı). ⚠️ Servo motorların
// bu yeni hızı fiziksel olarak ne kadar düzgün/sarsıntısız karşıladığı
// SADECE gerçek donanımda görülebilir — IK/geometri güvenli ama servo
// slew rate'i yazılımdan doğrulayamadım, ilk testte dikkatle izle.
float stepLen = 100.0, stepLift = 50.0, baseSpeed = 1000.0;
float joyX = 0.0, joyY = 0.0, joyR = 0.0;

float bodyPitch = 0.0, bodyRoll = 0.0, bodyYaw = 0.0;

// --- SIFIR ATIM AMORTİSÖRLERİ ---
float s_gaitZ = -100.0, s_pitch = 0.0, s_roll = 0.0, s_yaw = 0.0;
float s_len = 50.0, s_lift = 40.0;
float s_joyX = 0.0, s_joyY = 0.0, s_joyR = 0.0;

float currentPhase = 0.0;
bool gaitCommandWasActive = false;
unsigned long lastUpdate = 0;

// Gyro sıfır kayması (bias) kalibrasyonu — boot'ta bir kez ölçülür, MPU6050
// üretim toleransından kaynaklanan sabit ofseti giderir. Bu olmadan robot
// tamamen hareketsizken bile gyro entegrasyonu küçük bir sürüklenme üretir
// (araştırılan tüm kaynaklar bunu "flawless IMU" için zorunlu kabul ediyor).
float gyroOffsetX = 0.0f, gyroOffsetY = 0.0f, gyroOffsetZ = 0.0f;

enum Mode { IDLE, CAL, MANUAL, BEZIER_JOY, HOME_RISE };
Mode currentMode = IDLE;

enum SpineState {
  SPINE_BOOT,
  SPINE_DISARMED,
  SPINE_CAPTURE,
  SPINE_RISING,
  SPINE_STAND,
  SPINE_WALK,
  SPINE_FAILSAFE,
  SPINE_FAULT
};

SpineState spineState = SPINE_BOOT;
bool legOutputsArmed = false;
const unsigned long CAPTURE_HOLD_MS = 1500UL;
const unsigned long STAND_RAMP_MS   = 3000UL;
// Zaman tabanlı S-eğrisi, kalkış boyunca en fazla 30 deg/s.
// Uzayan HTTP/I2C karesi telafi edilerek büyük bir açı sıçraması üretilmez.
const float STAND_MAX_SPEED_DEG_S = 30.0f;
const float LEG_TRACK_MAX_SPEED_DEG_S = 120.0f;
const float SAFE_TRANSITION_MAX_SPEED_DEG_S = 60.0f;
float controlFrameDtS = 0.020f;
int lastLegPwmTick[6][3] = {{-1,-1,-1},{-1,-1,-1},{-1,-1,-1},
                           {-1,-1,-1},{-1,-1,-1},{-1,-1,-1}};

float currentPhysicalAngle[6][3];
bool firstRun = true;
bool safeTransitionActive = true;
unsigned long safeTransitionStartMs = 0;
bool bootSoftStart = true;
unsigned long bootStartMs = 0;
float bootMotionBlend = 0.0f;
float maxDiffThisFrame = 0.0;

// --- KALKIŞ FAZ KONTROLÜ ---
int           homeRisePhase   = 0;
unsigned long homeRisePhaseMs = 0;

// --- HOME POZİSYONU ---
float homeAngles[6][3];  // NVS'den yüklenen başlangıç açıları
bool  homeSet = false;   // home hiç kaydedildi mi?
bool  configValid = false;

// --- LEG SERVO LAB / MANUAL TARGETS ---
// MANUAL modunda sliderdan gönderilen hedefler her kontrol frame'inde tekrar uygulanır;
// böylece tek HTTP isteği yalnızca bir küçük smoothing adımı yapıp yarıda kalmaz.
float manualTargetAngle[6][3];
bool manualTargetsInitialized = false;

// --- COMMAND LINK SAFETY / PERSISTENT OUTPUT MASK ---
// HTTP compatibility path sends a 10 Hz heartbeat. The future nRF24 path will
// use the same state with a 50 Hz command frame. A dead browser/Pi must never
// leave the last non-zero joystick command active indefinitely.
// ESP32 WebServer is synchronous and Wi-Fi connection setup occasionally
// exceeds 300 ms even with a healthy 10 Hz controller. The Pi controller
// explicitly sends STOP+IDLE after two failed 300 ms attempts; this 750 ms
// watchdog remains an independent last-resort link-loss stop.
const unsigned long MOTION_COMMAND_TIMEOUT_MS = 750UL;
unsigned long lastMotionCommandMs = 0;
bool motionCommandSeen = false;
bool linkFailsafeActive = false;
uint32_t motionFailsafeCount = 0;

bool legPwmEnabled[6][3] = {
  {false, false, false}, {false, false, false}, {false, false, false},
  {false, false, false}, {false, false, false}, {false, false, false}
};

const float LEG_MOUNT_RIGHT[3] = {90.0f, 5.0f, 5.0f};
const float LEG_MOUNT_LEFT[3]  = {90.0f, 175.0f, 175.0f};

// --- GELİŞMİŞ GİMBAL ---
float gimbalRefPitch = 0.0f, gimbalRefRoll = 0.0f; // gimbal sıfır referansı
float s_gimbalPitch  = 0.0f, s_gimbalRoll  = 0.0f; // yumuşatılmış düzeltme
bool  gimbalRefCaptured = false; // boot sonrası otomatik yakalandı mı?

const char* spineStateName() {
  switch (spineState) {
    case SPINE_BOOT:     return "BOOT";
    case SPINE_DISARMED: return "DISARMED";
    case SPINE_CAPTURE:  return "CAPTURE";
    case SPINE_RISING:   return "RISING";
    case SPINE_STAND:    return "STAND";
    case SPINE_WALK:     return "WALK";
    case SPINE_FAILSAFE: return "FAILSAFE";
    case SPINE_FAULT:    return "FAULT";
  }
  return "UNKNOWN";
}

void setStatusLed(uint8_t red, uint8_t green, uint8_t blue) {
#ifdef RGB_BUILTIN
  // Values intentionally stay <= 8/255: visible status without a bright glare.
  const uint8_t r = (red   > 8) ? 8 : red;
  const uint8_t g = (green > 8) ? 8 : green;
  const uint8_t b = (blue  > 8) ? 8 : blue;
  rgbLedWrite(RGB_BUILTIN, r, g, b);
#else
  (void)red; (void)green; (void)blue;
#endif
}

void updateStatusLed() {
  static SpineState lastState = SPINE_FAULT;
  static bool lastWifi = false;
  const bool wifiNow = WiFi.status() == WL_CONNECTED;
  if (lastState == spineState && lastWifi == wifiNow) return;
  lastState = spineState;
  lastWifi = wifiNow;

  switch (spineState) {
    case SPINE_BOOT:     setStatusLed(0, 0, 5); break; // dim blue
    case SPINE_DISARMED: setStatusLed(0, wifiNow ? 4 : 2, wifiNow ? 4 : 6); break;
    case SPINE_CAPTURE:  setStatusLed(7, 2, 0); break; // amber
    case SPINE_RISING:   setStatusLed(5, 0, 7); break; // violet
    case SPINE_STAND:    setStatusLed(0, 6, 1); break; // green
    case SPINE_WALK:     setStatusLed(0, 2, 7); break; // blue
    case SPINE_FAILSAFE: setStatusLed(8, 2, 0); break; // orange-red
    case SPINE_FAULT:    setStatusLed(8, 0, 0); break; // red
  }
}

float approach(float current, float target, float maxDelta) {
  if (current < target) return min(current + maxDelta, target);
  if (current > target) return max(current - maxDelta, target);
  return current;
}

void addLog(String msg) {
  sysLogs += msg;
  if (sysLogs.length() > 2000) sysLogs = sysLogs.substring(sysLogs.length() - 2000);
}

void markMotionCommand() {
  lastMotionCommandMs = millis();
  motionCommandSeen = true;
}

void triggerMotionFailsafe() {
  if (linkFailsafeActive) return;
  joyX = joyY = joyR = 0.0f;
  // A link loss must not launch a fresh IDLE IK trajectory. Freeze the last
  // commanded physical angles and keep applying exactly those PWM values.
  // This is a controlled hold, not an attempt to re-center moving legs.
  for (int leg = 0; leg < 6; leg++)
    for (int joint = 0; joint < 3; joint++)
      manualTargetAngle[leg][joint] = currentPhysicalAngle[leg][joint];
  manualTargetsInitialized = true;
  currentMode = MANUAL;
  safeTransitionActive = false;
  linkFailsafeActive = true;
  spineState = legOutputsArmed ? SPINE_FAILSAFE : SPINE_DISARMED;
  motionFailsafeCount++;
  addLog("[SAFETY] Motion timeout: joystick sifirlandi, son eklem komutlari HOLD.\n");
}

// --- TOF400C / VL53L1X ---
void initTOF400C() {
  // Adafruit VL53L1X InitSensor() has an unbounded boot-state loop. Never call
  // it unless a bounded I2C probe has already confirmed 0x29 is responding.
  if (!tofAddressOnline) {
    tofReady = false;
    addLog("[-] TOF400C/VL53L1X BULUNAMADI (0x29); init atlandi.\n");
    bootDiagLog += "[-] TOF400C/VL53L1X YOK; INIT ATLANDI\n";
    return;
  }
  if (!tof400c.begin(0x29, &Wire)) {
    tofReady = false;
    addLog("[-] TOF400C/VL53L1X BULUNAMADI (0x29).\n");
    bootDiagLog += "[-] TOF400C/VL53L1X YOK\n";
    return;
  }
  if (!tof400c.startRanging()) {
    tofReady = false;
    addLog("[-] TOF400C ranging baslatilamadi. Status=" + String(tof400c.vl_status) + "\n");
    bootDiagLog += "[-] TOF400C RANGING HATASI\n";
    return;
  }
  // Geçerli timing budget değerlerinden 50 ms: ~20 Hz sınıfı, kontrol döngüsünü bloklamaz.
  tof400c.setTimingBudget(50);
  tofReady = true;
  addLog("[TOF] TOF400C/VL53L1X ONLINE @0x29, ranging basladi.\n");
  bootDiagLog += "[+] 0x29 -> TOF400C_VL53L1X ONLINE\n";
}

void updateTOF400C() {
  if (!tofReady) return;
  if (millis() - lastTofUpdateMs < 20) return;
  lastTofUpdateMs = millis();
  if (tof400c.dataReady()) {
    int16_t d = tof400c.distance();
    if (d >= 0) tofDistanceMm = d;
    tof400c.clearInterrupt();
  }
}

// --- BATARYA VOLTAJ MONITORU ---
uint16_t readAdcAverageMilliVolts(uint8_t pin, uint8_t samples = 12) {
  uint32_t sum = 0;
  for (uint8_t i = 0; i < samples; i++) {
    sum += analogReadMilliVolts(pin);
    delayMicroseconds(180);
  }
  return (uint16_t)(sum / samples);
}

void initBatteryMonitor() {
  analogReadResolution(12);
  analogSetPinAttenuation(BAT_RIGHT_ADC_PIN, ADC_11db);
  analogSetPinAttenuation(BAT_LEFT_ADC_PIN,  ADC_11db);
  pinMode(BAT_RIGHT_ADC_PIN, INPUT);
  pinMode(BAT_LEFT_ADC_PIN, INPUT);

  // Önceki saha kalibrasyonunu NVS'den geri yükle; yoksa 12.60/12.17
  // saha katsayısı olan 1.03533 kullanılır. Sağ ve sol kanal ayrı tutulur.
  preferences.begin("apex_power", true);
  BAT_RIGHT_CAL_FACTOR = preferences.getFloat("batRCal", 1.03533f);
  BAT_LEFT_CAL_FACTOR  = preferences.getFloat("batLCal", 1.03533f);
  preferences.end();

  addLog("[POWER] Batarya ADC: RIGHT=GPIO1, LEFT=GPIO2.\n");
  bootDiagLog += "[POWER] ADC RIGHT GPIO1 / LEFT GPIO2\n";
}

void updateBatteryMonitor() {
  if (millis() - lastBatteryUpdateMs < 250) return;
  lastBatteryUpdateMs = millis();

  batteryRightAdcMv = readAdcAverageMilliVolts(BAT_RIGHT_ADC_PIN);
  batteryLeftAdcMv  = readAdcAverageMilliVolts(BAT_LEFT_ADC_PIN);

  float rightNow = (batteryRightAdcMv / 1000.0f) * BAT_RIGHT_DIVIDER_RATIO * BAT_RIGHT_CAL_FACTOR;
  float leftNow  = (batteryLeftAdcMv  / 1000.0f) * BAT_LEFT_DIVIDER_RATIO  * BAT_LEFT_CAL_FACTOR;

  // Basit IIR low-pass; ilk okumada doğrudan seed et.
  if (batteryRightV < 0.1f) batteryRightV = rightNow;
  else batteryRightV += (rightNow - batteryRightV) * 0.20f;
  if (batteryLeftV < 0.1f) batteryLeftV = leftNow;
  else batteryLeftV += (leftNow - batteryLeftV) * 0.20f;
}

void initManualTargetsFromCurrent() {
  for (int leg = 0; leg < 6; leg++) {
    for (int joint = 0; joint < 3; joint++) {
      float v = currentPhysicalAngle[leg][joint];
      if (firstRun || !isfinite(v) || v < MIN_DEGREE || v > MAX_DEGREE) {
        if (homeSet) v = homeAngles[leg][joint];
        else if (joint == 0) v = 90.0f;
        else if (joint == 1) v = (leg < 3) ? 131.0f : 49.0f;
        else v = (leg < 3) ? 125.0f : 55.0f;
      }
      manualTargetAngle[leg][joint] = constrain(v, MIN_DEGREE, MAX_DEGREE);
    }
  }
  manualTargetsInitialized = true;
}

void setManualTarget(int leg, int joint, float angle) {
  if (leg < 0 || leg >= 6 || joint < 0 || joint >= 3) return;
  // Her yeni MANUAL oturumunda diger 17 eklemi o anki komut-acisinda tut.
  if (currentMode != MANUAL || !manualTargetsInitialized) initManualTargetsFromCurrent();
  manualTargetAngle[leg][joint] = constrain(angle, MIN_DEGREE, MAX_DEGREE);
  currentMode = MANUAL;
}

void setLegMountTargets(int leg) {
  if (currentMode != MANUAL || !manualTargetsInitialized) initManualTargetsFromCurrent();
  int startLeg = (leg < 0) ? 0 : leg;
  int endLeg   = (leg < 0) ? 5 : leg;
  for (int i = startLeg; i <= endLeg; i++) {
    const float* pose = (i < 3) ? LEG_MOUNT_RIGHT : LEG_MOUNT_LEFT;
    for (int j = 0; j < 3; j++) manualTargetAngle[i][j] = pose[j];
  }
  currentMode = MANUAL;
}

void setLegHomeTargets(int leg) {
  if (currentMode != MANUAL || !manualTargetsInitialized) initManualTargetsFromCurrent();
  int startLeg = (leg < 0) ? 0 : leg;
  int endLeg   = (leg < 0) ? 5 : leg;
  for (int i = startLeg; i <= endLeg; i++)
    for (int j = 0; j < 3; j++) manualTargetAngle[i][j] = constrain(homeAngles[i][j], MIN_DEGREE, MAX_DEGREE);
  currentMode = MANUAL;
}

void disableLegPwm(int leg, int joint) {
  int legStart = (leg < 0) ? 0 : leg;
  int legEnd   = (leg < 0) ? 5 : leg;
  for (int i = legStart; i <= legEnd; i++) {
    int jointStart = (joint < 0) ? 0 : joint;
    int jointEnd   = (joint < 0) ? 2 : joint;
    for (int j = jointStart; j <= jointEnd; j++) {
      legPwmEnabled[i][j] = false;
      uint8_t ch = legPins[i][j];
      if (i < 3) pwmRight.setPWM(ch, 0, 0);
      else       pwmLeft.setPWM(ch, 0, 0);
      lastLegPwmTick[i][j] = -1;
    }
  }
  if (leg < 0 && joint < 0) {
    legOutputsArmed = false;
    spineState = SPINE_DISARMED;
  }
}

void enableLegPwm(int leg, int joint) {
  int legStart = (leg < 0) ? 0 : leg;
  int legEnd   = (leg < 0) ? 5 : leg;
  for (int i = legStart; i <= legEnd; i++) {
    int jointStart = (joint < 0) ? 0 : joint;
    int jointEnd   = (joint < 0) ? 2 : joint;
    for (int j = jointStart; j <= jointEnd; j++) {
      legPwmEnabled[i][j] = true;
    }
  }
  if (leg < 0 && joint < 0) legOutputsArmed = true;
}

// --- MPU6050 ---
void initMPU6050() {
  if (!mpuOnline) {
    addLog("[-] MPU6050 BULUNAMADI; init atlandi.\n");
    bootDiagLog += "[-] MPU6050 YOK; INIT ATLANDI\n";
    return;
  }
  Wire.beginTransmission(0x68);
  Wire.write(0x6B); Wire.write(0x00);
  if (Wire.endTransmission() == 0) {
    mpuOnline = true;
    addLog("[MPU] MPU6050 Basariyla Uyandirildi!\n");
    bootDiagLog += "[MPU] MPU6050 Uyandirildi\n";
    delay(200);
    // Filtre başlangıcını ivmemetre ile to̊hum: 0°'den yakınsama sorunu ortadan kalkar
    Wire.beginTransmission(0x68);
    Wire.write(0x3B);
    Wire.endTransmission(false);
    Wire.requestFrom(0x68, 6, true);
    if (Wire.available() >= 6) {
      int16_t ax0 = (int16_t)(Wire.read() << 8 | Wire.read());
      int16_t ay0 = (int16_t)(Wire.read() << 8 | Wire.read());
      int16_t az0 = (int16_t)(Wire.read() << 8 | Wire.read());
      // DÜZELTME: pitch eskiden -az0 (DİKEY eksen) kullanıyordu, düz zeminde
      // ~-90° okunmasına sebep oluyordu (az≈g, ax≈ay≈0 → atan2(-g,~0)=-90°).
      // Standart konvansiyon: pitch = İLERİ eksen (ax) ile hesaplanır — roll
      // formülü (ay tabanlı) zaten bu kalıbı doğru kullanıyordu, pitch'i ona
      // eşledik.
      mpuPitch   = atan2f(-(float)ax0, sqrtf((float)ay0*ay0 + (float)az0*az0)) * 180.0f / (float)PI;
      mpuRoll    = atan2f((float)ay0,  sqrtf((float)ax0*ax0 + (float)az0*az0)) * 180.0f / (float)PI;
      s_mpuPitch = mpuPitch;
      s_mpuRoll  = mpuRoll;
      lastMpuTime = millis();
      addLog("[MPU] FILTRE TOHUM: P=" + String(mpuPitch,1) + " R=" + String(mpuRoll,1) + "\n");
    }
  } else {
    mpuOnline = false;
    addLog("[-] MPU6050 BULUNAMADI! Lutfen baglantilari kontrol edin.\n");
    bootDiagLog += "[-] MPU6050 BULUNAMADI!\n";
  }
}

void calibrateMPU6050Gyro() {
  if (!mpuOnline) {
    addLog("[MPU] Gyro kalibrasyonu atlandi: MPU cevrimdisi.\n");
    return;
  }
  // Robot bu sırada HAREKETSİZ olmalı (boot sırasında çağrılır). 200 örnek
  // alıp ortalamasını "sıfır kayması" (bias) olarak kaydeder, sonraki her
  // okumadan çıkarılır. Manuel kalibrasyon yapılmadan gyro entegrasyonu
  // zamanla sürükler (tüm IMU kaynakları bunu doğruluyor).
  addLog("[MPU] Gyro kalibrasyonu basliyor (~0.6s, ROBOTU HAREKET ETTIRME)...\n");
  const int N = 200;
  long sumX = 0, sumY = 0, sumZ = 0;
  int validSamples = 0;
  for (int i = 0; i < N; i++) {
    Wire.beginTransmission(0x68);
    Wire.write(0x43);  // GYRO_XOUT_H — gyro veri bloğunun başlangıcı
    Wire.endTransmission(false);
    Wire.requestFrom(0x68, 6, true);
    if (Wire.available() >= 6) {
      int16_t gx = (int16_t)(Wire.read() << 8 | Wire.read());
      int16_t gy = (int16_t)(Wire.read() << 8 | Wire.read());
      int16_t gz = (int16_t)(Wire.read() << 8 | Wire.read());
      sumX += gx; sumY += gy; sumZ += gz;
      validSamples++;
    }
    delay(3);
  }
  if (validSamples > 0) {
    gyroOffsetX = (float)sumX / validSamples;
    gyroOffsetY = (float)sumY / validSamples;
    gyroOffsetZ = (float)sumZ / validSamples;
    addLog("[MPU] Gyro kalibre edildi. OfsX=" + String(gyroOffsetX,1) +
           " OfsY=" + String(gyroOffsetY,1) + " OfsZ=" + String(gyroOffsetZ,1) + "\n");
  } else {
    addLog("[-] Gyro kalibrasyonu basarisiz - hic ornek okunamadi.\n");
  }
}

void readMPU6050() {
  if (!mpuOnline) return;
  Wire.beginTransmission(0x68);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom(0x68, 14, true);
  if(Wire.available() < 14) return;

  int16_t ax = Wire.read()<<8|Wire.read();
  int16_t ay = Wire.read()<<8|Wire.read();
  int16_t az = Wire.read()<<8|Wire.read();
  Wire.read()<<8|Wire.read();
  int16_t gx = Wire.read()<<8|Wire.read();
  int16_t gy = Wire.read()<<8|Wire.read();
  int16_t gz = Wire.read()<<8|Wire.read();

  // DÜZELTME: pitch eskiden -az (DİKEY eksen) kullanıyordu — düz zeminde
  // ~-90° okunmasına sebep oluyordu. Standart konvansiyon: pitch İLERİ
  // eksen (ax) ile hesaplanır (roll'un ay ile yaptığı gibi).
  float accelPitch = atan2(-ax, sqrt(ay*ay + az*az)) * 180.0 / PI;
  float accelRoll  = atan2(ay,  sqrt(ax*ax + az*az)) * 180.0 / PI;

  // Gyro kalibrasyon ofseti uygulanıyor (bkz. calibrateMPU6050Gyro)
  float gxF = gx - gyroOffsetX;
  float gyF = gy - gyroOffsetY;
  float gzF = gz - gyroOffsetZ;

  float gyroPitchRate = gyF / 131.0;  // Y ekseni etrafında dönüş = pitch hızı (DOĞRUYDU, değişmedi)
  // DÜZELTME: roll hızı eskiden gz (DİKEY/yaw eksen) kullanıyordu. Roll, İLERİ
  // eksen (X) etrafında dönüştür — bu yüzden gx kullanılmalı, gz değil.
  float gyroRollRate  = gxF / 131.0;
  float gyroYawRate   = gzF / 131.0;

  unsigned long now = millis();
  float dt = (now - lastMpuTime) / 1000.0;
  lastMpuTime = now;
  if (dt > 0.1) dt = 0.02;

  mpuPitch = 0.96 * (mpuPitch + gyroPitchRate * dt) + 0.04 * accelPitch;
  mpuRoll  = 0.96 * (mpuRoll  + gyroRollRate * dt)  + 0.04 * accelRoll;

  // MPU6050'de manyetometre yoktur; bu nedenle yaw mutlak pusula yönü değil,
  // boot/zero anına göre gyro-Z entegrasyonlu RELATİF açıdır. Bias boot'ta
  // gyroOffsetZ ile çıkarılır. -180..180 aralığında tutulur.
  mpuYaw += gyroYawRate * dt;
  while (mpuYaw > 180.0f) mpuYaw -= 360.0f;
  while (mpuYaw < -180.0f) mpuYaw += 360.0f;
}

void scanI2C() {
  pcaRightOnline = false;
  pcaLeftOnline  = false;
  mpuOnline      = false;
  tofAddressOnline = false;
  addLog("[SYS] INITIATING HARDWARE DIAGNOSTICS...\n");
  addLog("[SYS] SCANNING I2C BUS [SDA:8, SCL:9]\n");
  bootDiagLog += "[SYS] I2C TARAMASI [SDA:8, SCL:9]\n";
  byte error, address; int found = 0;
  for (address = 1; address < 127; address++) {
    Wire.beginTransmission(address);
    error = Wire.endTransmission();
    if (error == 0) {
      if (address == 0x40) { pcaRightOnline = true; addLog("[+] 0x40 -> [PCA9685_RIGHT_LEGS] ONLINE\n"); bootDiagLog += "[+] 0x40 -> PCA9685_RIGHT_LEGS ONLINE\n"; }
      else if (address == 0x41) { pcaLeftOnline = true; addLog("[+] 0x41 -> [PCA9685_LEFT_LEGS] ONLINE\n"); bootDiagLog += "[+] 0x41 -> PCA9685_LEFT_LEGS ONLINE\n"; }
      else if (address == 0x68) { mpuOnline = true; addLog("[+] 0x68 -> [MPU6050_IMU] ONLINE\n"); bootDiagLog += "[+] 0x68 -> MPU6050_IMU ONLINE\n"; }
      else if (address == 0x29) { tofAddressOnline = true; addLog("[+] 0x29 -> [TOF400C_VL53L1X] ONLINE\n"); bootDiagLog += "[+] 0x29 -> TOF400C_VL53L1X ONLINE\n"; }
      else { addLog("[+] 0x" + String(address, HEX) + " -> [UNKNOWN_DEVICE] ONLINE\n"); bootDiagLog += "[+] 0x" + String(address, HEX) + " -> UNKNOWN_DEVICE ONLINE\n"; }
      found++;
    }
  }
  if (found == 0) { addLog("[-] CRITICAL FAILURE: NO I2C HARDWARE DETECTED!\n"); bootDiagLog += "[-] KRITIK: I2C DONANIMI YOK!\n"; }
  else { addLog("[SYS] DIAGNOSTICS COMPLETE. ALL SYSTEMS NOMINAL.\n"); bootDiagLog += "[SYS] TANI TAMAMLANDI. SISTEM NOMINAL.\n"; }
}

bool probeI2CAddress(uint8_t address) {
  Wire.beginTransmission(address);
  return Wire.endTransmission() == 0;
}

void recoverLateI2CDevices() {
  if (millis() - lastI2CRecoveryMs < I2C_RECOVERY_INTERVAL_MS) return;
  lastI2CRecoveryMs = millis();

  const bool rightNow = probeI2CAddress(0x40);
  const bool leftNow  = probeI2CAddress(0x41);
  const bool mpuNow   = probeI2CAddress(0x68);
  const bool tofNow   = probeI2CAddress(0x29);

  // A live actuator board disappearing invalidates every held command.
  if (legOutputsArmed && (!rightNow || !leftNow)) {
    disableLegPwm(-1, -1);
    joyX = joyY = joyR = 0.0f;
    currentMode = IDLE;
    addLog("[SAFETY] PCA kaybi: tum bacak PWM cikislari kapatildi.\n");
  }

  if (rightNow && !pcaRightOnline) {
    pwmRight.begin();
    pwmRight.setPWMFreq(50);
    for (uint8_t ch = 0; ch < 16; ch++) pwmRight.setPWM(ch, 0, 0);
    for (int leg = 0; leg < 3; leg++) for (int joint = 0; joint < 3; joint++) {
      legPwmEnabled[leg][joint] = false;
      lastLegPwmTick[leg][joint] = -1;
    }
    armOutputEnabled = false;
    addLog("[RECOVERY] SAG PCA 0x40 sonradan bulundu; tum kanallar guvenli OFF.\n");
  }
  if (leftNow && !pcaLeftOnline) {
    pwmLeft.begin();
    pwmLeft.setPWMFreq(50);
    for (uint8_t ch = 0; ch < 16; ch++) pwmLeft.setPWM(ch, 0, 0);
    for (int leg = 3; leg < 6; leg++) for (int joint = 0; joint < 3; joint++) {
      legPwmEnabled[leg][joint] = false;
      lastLegPwmTick[leg][joint] = -1;
    }
    addLog("[RECOVERY] SOL PCA 0x41 sonradan bulundu; tum kanallar guvenli OFF.\n");
  }
  pcaRightOnline = rightNow;
  pcaLeftOnline = leftNow;

  if (mpuNow && !mpuOnline) {
    mpuOnline = true;
    initMPU6050();
    calibrateMPU6050Gyro();
    gimbalRefCaptured = false;
    addLog("[RECOVERY] MPU6050 sonradan bulundu ve yeniden baslatildi.\n");
  } else if (!mpuNow) {
    mpuOnline = false;
  }

  tofAddressOnline = tofNow;
  if (tofNow && !tofReady) {
    initTOF400C();
    addLog("[RECOVERY] TOF sonradan bulundu ve yeniden baslatildi.\n");
  } else if (!tofNow) {
    tofReady = false;
    tofDistanceMm = -1;
  }
}

const char INDEX_HTML[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html lang="tr">
<head>
    <meta charset="UTF-8"><meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>APEX COMMAND CENTER</title>
    <link rel="stylesheet" href="https://unpkg.com/leaflet@1.9.4/dist/leaflet.css"/>
    <script src="https://unpkg.com/leaflet@1.9.4/dist/leaflet.js"></script>
    <style>
        body { font-family: 'Segoe UI', sans-serif; background: #080808; color: #fff; text-align: center; margin: 0; padding: 15px; user-select: none; touch-action: manipulation;}
        h1 { color: #e60000; letter-spacing: 3px; border-bottom: 2px solid #e60000; display: inline-block; padding-bottom: 5px; margin-top: 0; font-size:24px;}

        .dashboard { display: grid; grid-template-columns: repeat(auto-fit, minmax(320px, 1fr)); gap: 15px; max-width: 1400px; margin: 0 auto; text-align:left;}
        .panel { background: #151515; padding: 15px; border-radius: 8px; border-top: 4px solid #e60000; box-shadow: 0 4px 10px rgba(0,0,0,0.8); display: flex; flex-direction: column;}
        .panel-wide { grid-column: 1 / -1; }

        .label { font-size: 14px; font-weight: bold; color: #aaa; display: block; margin-bottom: 5px; text-transform: uppercase;}
        .val-text { color: #00ffcc; font-weight: bold; float: right; }

        button { background: #222; color: white; border: 1px solid #444; padding: 12px; font-size: 14px; border-radius: 4px; cursor: pointer; font-weight: bold; transition: 0.2s; width: 100%; margin-top:8px;}
        button:active { background: #e60000; border-color: #e60000; }
        .btn-cal { background: #e60000; color: #fff; }
        .btn-gimbal { background: #00ffcc; color: #000; border: none; font-size:16px; margin-bottom:10px; }
        .btn-gimbal.off { background: #444; color: #aaa; }

        input[type=range] { width: 100%; accent-color: #00ffcc; margin-bottom: 12px; }
        input[type=number] { width: 100%; padding: 5px; background: #111; color: #0f0; border: 1px solid #444; text-align:center;}
        select { background: #333; color: white; border: none; padding: 8px; border-radius: 4px; font-size: 14px; width: 100%; font-weight:bold; margin-bottom: 10px;}

        .status-box { background: #0a0a0a; border: 1px solid #333; border-radius: 8px; padding: 10px; margin-bottom: 10px; display: flex; justify-content: space-around; align-items: center; text-align:center;}
        .status-data { font-size: 12px; color: #aaa; font-weight: bold;}
        .status-highlight { color: #00ffcc; font-weight: bold; font-size: 18px; display:block; margin-top:4px;}

        .hud-panel { background: #050505; border: 1px solid #333; border-radius: 4px; padding: 10px 15px; margin-bottom: 12px; display: flex; justify-content: space-between; font-family: monospace; text-align: center; flex-wrap: wrap; gap: 10px;}
        .hud-label { font-size: 11px; color: #777; font-weight: bold; letter-spacing: 1px; }
        .hud-value { font-size: 18px; color: #00ffcc; font-weight: bold; display: block; margin-top: 2px;}
        .hud-mpu { color: #ffaa00; }

        .terminal { background: #000; color: #0f0; font-family: monospace; padding: 10px; flex-grow: 1; height: 250px; overflow-y: auto; border-radius: 5px; border: 1px solid #333; font-size:12px; line-height: 1.4; white-space: pre-wrap;}
        #map { height: 350px; width: 100%; border-radius: 8px; border: 1px solid #333; margin-bottom: 10px;}
        .telemetry-container { display: flex; gap: 15px; flex-wrap: wrap; }
        .telemetry-col { flex: 1; min-width: 300px; display: flex; flex-direction: column; }

        .joy-container { display:flex; flex-direction: column; align-items:center; margin: 10px 0; gap: 15px;}
        #joyArea { width: 220px; height: 220px; background: #222; border-radius: 50%; position: relative; border: 2px solid #555; box-shadow: inset 0 0 20px #000;}
        #joyKnob { width: 70px; height: 70px; background: #e60000; border-radius: 50%; position: absolute; top: 75px; left: 75px; pointer-events: none; transition: 0.1s ease-out; box-shadow: 0 4px 10px rgba(0,0,0,0.5);}
        #joyAreaR { width: 220px; height: 60px; background: #222; border-radius: 30px; position: relative; border: 2px solid #00ffcc; box-shadow: inset 0 0 10px #000;}
        #joyKnobR { width: 60px; height: 60px; background: #00ffcc; border-radius: 50%; position: absolute; top: 0px; left: 80px; pointer-events: none; transition: 0.1s ease-out; box-shadow: 0 4px 10px rgba(0,0,0,0.5);}
        .grid-3 { display: grid; grid-template-columns: 1fr 1fr 1fr; gap: 5px; text-align:center;}

        #vectorCanvas { display:block; margin:0 auto; background:#0d0d0d; border-radius:50%; border:1px solid #333; }
        .vec-label { font-size:10px; color:#555; letter-spacing:1px; text-align:center; margin-bottom:3px; }
    </style>
</head>
<body>
    <h1>APEX COMMAND CENTER</h1>

    <div class="dashboard">
        <div class="panel panel-wide">
            <span class="label">SENSÖR AĞI VE NAVİGASYON</span>
            <div class="status-box">
                <div class="status-data">MOD<span id="dispMode" class="status-highlight">STAND</span></div>
                <div class="status-data">Z-AXIS<span id="dispZ" class="status-highlight">-100</span></div>
                <div class="status-data">GPS<span id="dispGPS" class="status-highlight" style="color:#ffaa00;">ARANIYOR</span></div>
            </div>

            <div class="hud-panel">
                <div><span class="hud-label">LATITUDE (ENLEM)</span><span id="hudLat" class="hud-value">--.------</span></div>
                <div><span class="hud-label">LONGITUDE (BOYLAM)</span><span id="hudLng" class="hud-value">--.------</span></div>
                <div><span class="hud-label">SATS (UYDU)</span><span id="hudSats" class="hud-value">0</span></div>
                <div style="border-left: 1px solid #333; padding-left:15px;"><span class="hud-label">MPU PITCH</span><span id="hudMpuP" class="hud-value hud-mpu">0.0&deg;</span></div>
                <div><span class="hud-label">MPU ROLL</span><span id="hudMpuR" class="hud-value hud-mpu">0.0&deg;</span></div>
                <div style="border-left:1px solid #333;padding-left:15px;"><span class="hud-label">GİMBAL DÜZ P</span><span id="hudGP" class="hud-value" style="color:#cc66ff;">0.0&deg;</span></div>
                <div><span class="hud-label">GİMBAL DÜZ R</span><span id="hudGR" class="hud-value" style="color:#cc66ff;">0.0&deg;</span></div>
            </div>

            <div class="telemetry-container">
                <div class="telemetry-col"><div id="map"></div></div>
                <div class="telemetry-col"><div id="logBox" class="terminal">Sistem başlatılıyor...</div></div>
            </div>
        </div>

        <div class="panel" style="border-top-color: #00ffcc;">
            <span class="label" style="color:#00ffcc;">BODY IK VE GİMBAL KONTROLÜ</span>
            <button id="btnGimbal" class="btn-gimbal off" onclick="toggleGimbal()">GİMBAL (OTO-DENGE): KAPALI</button>

            <div>
            <div style="margin-bottom: 12px;">
                <span class="label">GİMBAL FİLTRE (Yumuşatma)<span id="vGf" class="val-text">80</span></span>
                <input type="range" id="gfilter" min="0" max="100" value="80" oninput="sendGimbalFilter()">
            </div>

                <span class="label">PITCH (Eğilme)<span id="vP" class="val-text">0&deg;</span></span>
                <input type="range" id="pitch" min="-35" max="35" value="0" oninput="sendIK()">
                <span class="label">ROLL (Yatma)<span id="vR" class="val-text">0&deg;</span></span>
                <input type="range" id="roll" min="-35" max="35" value="0" oninput="sendIK()">
                <span class="label">YAW (Dönme)<span id="vYw" class="val-text">0&deg;</span></span>
                <input type="range" id="yaw" min="-20" max="20" value="0" oninput="sendIK()">
                <button onclick="resetIK()" style="background:#333; color:#fff;">AÇILARI SIFIRLA</button>
            </div>
            <div style="border-top: 1px solid #333; margin: 15px 0;"></div>
            <span class="label" style="color:#00ffcc;">YÜRÜYÜŞ DİNAMİKLERİ (GAIT)</span>
            <div>
                <span class="label">Yerden Yükseklik (-Z)<span id="vGz" class="val-text">-100</span></span>
                <input type="range" id="gz" min="-250" max="-50" value="-100" oninput="sendParams()">
                <span class="label">Adım Uzunluğu<span id="vSl" class="val-text">50</span></span>
                <input type="range" id="sl" min="10" max="150" value="50" oninput="sendParams()">
                <span class="label">Adım Yüksekliği<span id="vSh" class="val-text">40</span></span>
                <input type="range" id="sh" min="10" max="150" value="40" oninput="sendParams()">
                <span class="label">Maks Hız (ms)<span id="vSp" class="val-text">2000</span></span>
                <input type="range" id="sp" min="500" max="4000" value="2000" oninput="sendParams()">
            </div>
        </div>

        <div class="panel">
            <span class="label">HAREKET & DÖNÜŞ KONTROLÜ</span>
            <div class="joy-container">
                <div style="text-align:center; font-size:11px; color:#aaa; margin-bottom:-10px;">ÇAPRAZ YÜRÜYÜŞ (OMNI)</div>
                <div id="joyArea"><div id="joyKnob"></div></div>
                <div style="text-align:center; font-size:11px; color:#aaa; margin-top:5px; margin-bottom:-10px;">KENDİ ETRAFINDA DÖNÜŞ (YAW)</div>
                <div id="joyAreaR"><div id="joyKnobR"></div></div>
                <div class="vec-label">HAREKET VEKTÖRÜ</div>
                <canvas id="vectorCanvas" width="160" height="160"></canvas>
            </div>
            <button onclick="sendMode('idle')" style="background:#555;">&#9654; STAND POZİSYONUNA DÖN</button>
            <button class="btn-cal" onclick="sendMode('cal')">&#128295; MONTAJ MODU (TÜMÜ C:90 F:0 T:0)</button>
        </div>

        <div class="panel" style="border-top-color: #555;">
            <span class="label">DONANIM KALİBRASYONU</span>
            <select id="legSelect" onchange="loadLegCalib()">
                <option value="0">Sağ Ön</option><option value="1">Sağ Orta</option><option value="2">Sağ Arka</option>
                <option value="3">Sol Ön</option><option value="4">Sol Orta</option><option value="5">Sol Arka</option>
            </select>
            <div class="grid-3" style="font-size:12px; color:#aaa; margin-bottom:5px;">
                <div>EKLEM</div><div>MIN &mu;s</div><div>MAX &mu;s</div>
            </div>
            <div class="grid-3">
                <div>COXA</div> <div><input type="number" id="cMin"></div> <div><input type="number" id="cMax"></div>
                <div>FEMUR</div><div><input type="number" id="fMin"></div> <div><input type="number" id="fMax"></div>
                <div>TIBIA</div><div><input type="number" id="tMin"></div> <div><input type="number" id="tMax"></div>
            </div>
            <div class="grid-3" style="margin-top:5px;">
                <div>OFSET(&deg;)</div><div><input type="number" id="cOff"></div> <div></div>
                <div>OFSET(&deg;)</div><div><input type="number" id="fOff"></div> <div></div>
                <div>OFSET(&deg;)</div><div><input type="number" id="tOff"></div> <div></div>
            </div>
            <button onclick="saveCalib()" style="background:#444; color:#fff;">KAYDET</button>

            <div style="border-top: 1px solid #333; margin: 15px 0;"></div>
            <span class="label">MANUEL TEST</span>
            <input type="range" min="0" max="180" value="90" oninput="sendMan('0', this.value)">
            <input type="range" min="0" max="180" value="90" oninput="sendMan('1', this.value)">
            <input type="range" min="0" max="180" value="90" oninput="sendMan('2', this.value)">
        </div>

        <div class="panel" style="border-top-color:#9933ff;">
            <span class="label" style="color:#9933ff;">&#127968; AÇILIŞ KONUMU (HOME)</span>
            <div id="homeStatus" style="font-size:11px;color:#777;text-align:center;margin-bottom:8px;padding:6px;background:#111;border-radius:4px;">HOME: Henüz kaydedilmedi</div>
            <div style="font-size:11px;color:#666;margin-bottom:10px;line-height:1.5;">
                Robotu istenen başlangıç pozisyonuna getir, sliderları ayarla, her bacağı kaydet.
                Açılışta bu pozisyondan yavaşça ayağa kalkar.
            </div>
            <select id="homeLegSelect" onchange="loadHomeAngles()">
                <option value="0">Sağ Ön</option><option value="1">Sağ Orta</option><option value="2">Sağ Arka</option>
                <option value="3">Sol Ön</option><option value="4">Sol Orta</option><option value="5">Sol Arka</option>
            </select>
            <span class="label">COXA<span id="vhC" class="val-text">90</span></span>
            <input type="range" id="hcSlider" min="0" max="180" value="90" oninput="previewHomeJoint(0,this.value)">
            <span class="label">FEMUR<span id="vhF" class="val-text">131</span></span>
            <input type="range" id="hfSlider" min="0" max="180" value="131" oninput="previewHomeJoint(1,this.value)">
            <span class="label">TİBİA<span id="vhT" class="val-text">125</span></span>
            <input type="range" id="htSlider" min="0" max="180" value="125" oninput="previewHomeJoint(2,this.value)">
            <div style="display:grid;grid-template-columns:1fr 1fr;gap:8px;margin-top:8px;">
                <button onclick="saveHomeLeg()" style="background:#9933ff;color:#fff;margin:0;">&#128190; KAYDET</button>
                <button onclick="playHomeRise()" style="background:#444;margin:0;">&#127775; KALKIŞ OYNAT</button>
            </div>
        </div>

        <div class="panel" style="border-top-color:#ff6600;">
            <span class="label" style="color:#ff6600;">&#128266; SES S&#304;STEM&#304;</span>
            <div style="display:flex;align-items:center;justify-content:center;gap:12px;margin:12px 0;">
                <button onclick="audioCmd('prev')" style="width:52px;padding:12px;font-size:20px;margin:0;">&#9664;&#9664;</button>
                <div style="text-align:center;">
                    <div style="font-size:11px;color:#777;letter-spacing:1px;">PARCA</div>
                    <div id="trackDisp" style="font-size:36px;font-weight:bold;color:#ff6600;line-height:1;">1</div>
                </div>
                <button onclick="audioCmd('next')" style="width:52px;padding:12px;font-size:20px;margin:0;">&#9654;&#9654;</button>
            </div>
            <div style="display:grid;grid-template-columns:1fr 1fr;gap:8px;margin-bottom:14px;">
                <button onclick="audioCmd('play')" style="background:#ff6600;font-size:20px;padding:16px;margin:0;">&#9654; OYNAT</button>
                <button onclick="audioCmd('stop')" style="font-size:20px;padding:16px;margin:0;">&#9209; DUR</button>
            </div>
            <span class="label">SES SEV&#304;YES&#304;<span id="vVol" class="val-text">15</span></span>
            <input type="range" id="volSlider" min="0" max="30" value="15" oninput="setVolume(this.value)">
        </div>
    </div>

    <script>
        let map; let apexMarker;
        let debounceParams, debounceIK;
        let gimbalState = false;

        window.onload = () => {
            map = L.map('map').setView([41.0256, 28.8895], 16);
            L.tileLayer('https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png', { attribution: '&copy; OSM' }).addTo(map);
            let redIcon = L.icon({ iconUrl: 'https://raw.githubusercontent.com/pointhi/leaflet-color-markers/master/img/marker-icon-red.png', iconSize: [25, 41], iconAnchor: [12, 41]});
            apexMarker = L.marker([41.0256, 28.8895], {icon: redIcon}).addTo(map);
            loadLegCalib();
            loadHomeAngles();
            sendGimbalFilter();
            drawVector();
            setInterval(fetchTelemetry, 1000);
            setInterval(drawVector, 80);
        };

        function fetchTelemetry() {
            fetch('/telemetry').then(r => r.json()).then(data => {
                let box = document.getElementById('logBox');
                let isAtBottom = box.scrollHeight - box.clientHeight <= box.scrollTop + 1;
                box.innerText = data.logs;
                if(isAtBottom) box.scrollTop = box.scrollHeight;

                document.getElementById('dispZ').innerText = data.z;
                document.getElementById('dispMode').innerText = data.mode;
                document.getElementById('hudLat').innerText = data.lat.toFixed(6);
                document.getElementById('hudLng').innerText = data.lng.toFixed(6);
                document.getElementById('hudSats').innerText = data.sats;
                document.getElementById('hudMpuP').innerText = data.mpuP.toFixed(1) + "°";
                document.getElementById('hudMpuR').innerText = data.mpuR.toFixed(1) + "°";
                document.getElementById('hudGP').innerText = data.gP.toFixed(1) + "°";
                document.getElementById('hudGR').innerText = data.gR.toFixed(1) + "°";
                let hs = document.getElementById('homeStatus');
                if (data.homeSet) { hs.innerText = "HOME: Kayıtlı ✓"; hs.style.color = "#9933ff"; }
                else              { hs.innerText = "HOME: Henüz kaydedilmedi"; hs.style.color = "#777"; }

                let gpsElem = document.getElementById('dispGPS');
                if (data.fix) {
                    gpsElem.innerText = "KILITLENDI"; gpsElem.style.color = "#0f0";
                    let newLatLng = new L.LatLng(data.lat, data.lng);
                    apexMarker.setLatLng(newLatLng); map.panTo(newLatLng);
                } else {
                    gpsElem.innerText = "ARANIYOR..."; gpsElem.style.color = "#ffaa00";
                }
            }).catch(e => console.log(e));
        }

        function toggleGimbal() {
            gimbalState = !gimbalState;
            let btn = document.getElementById('btnGimbal');
            if(gimbalState) {
                btn.className = "btn-gimbal";
                btn.innerText = "GİMBAL (OTO-DENGE): AKTİF";
            } else {
                btn.className = "btn-gimbal off";
                btn.innerText = "GİMBAL (OTO-DENGE): KAPALI";
            }
            fetch('/setGimbal?state=' + (gimbalState ? '1' : '0'));
        }

        function sendGimbalFilter() {
            let gf = document.getElementById('gfilter').value;
            document.getElementById('vGf').innerText = gf;
            fetch(`/setGimbalFilter?v=${gf}`);
        }

        function loadLegCalib() {
            let leg = document.getElementById('legSelect').value;
            fetch('/getCalib?leg=' + leg).then(res => res.json()).then(data => {
                document.getElementById('cMin').value = data.cMin; document.getElementById('cMax').value = data.cMax; document.getElementById('cOff').value = data.cOff;
                document.getElementById('fMin').value = data.fMin; document.getElementById('fMax').value = data.fMax; document.getElementById('fOff').value = data.fOff;
                document.getElementById('tMin').value = data.tMin; document.getElementById('tMax').value = data.tMax; document.getElementById('tOff').value = data.tOff;
            });
        }

        function saveCalib() {
            let leg = document.getElementById('legSelect').value;
            let q = `leg=${leg}&cMin=${document.getElementById('cMin').value}&cMax=${document.getElementById('cMax').value}&cOff=${document.getElementById('cOff').value}&fMin=${document.getElementById('fMin').value}&fMax=${document.getElementById('fMax').value}&fOff=${document.getElementById('fOff').value}&tMin=${document.getElementById('tMin').value}&tMax=${document.getElementById('tMax').value}&tOff=${document.getElementById('tOff').value}`;
            fetch('/saveCalib?' + q); alert("Kaydedildi!");
        }

        function sendMan(joint, val) {
            let leg = document.getElementById('legSelect').value; fetch(`/manual?leg=${leg}&j=${joint}&v=${val}`);
        }

        function sendParams() {
            let gz = document.getElementById('gz').value; let sl = document.getElementById('sl').value;
            let sh = document.getElementById('sh').value; let sp = document.getElementById('sp').value;
            document.getElementById('vGz').innerText = gz; document.getElementById('vSl').innerText = sl;
            document.getElementById('vSh').innerText = sh; document.getElementById('vSp').innerText = sp;
            clearTimeout(debounceParams); debounceParams = setTimeout(() => { fetch(`/params?z=${gz}&len=${sl}&lift=${sh}&spd=${sp}`); }, 50);
        }

        function sendIK() {
            let p = document.getElementById('pitch').value; let r = document.getElementById('roll').value; let y = document.getElementById('yaw').value;
            document.getElementById('vP').innerText = p + "°"; document.getElementById('vR').innerText = r + "°"; document.getElementById('vYw').innerText = y + "°";
            clearTimeout(debounceIK); debounceIK = setTimeout(() => { fetch(`/bodyIk?p=${p}&r=${r}&y=${y}`); }, 50);
        }

        function resetIK() { document.getElementById('pitch').value = 0; document.getElementById('roll').value = 0; document.getElementById('yaw').value = 0; sendIK(); }
        function sendMode(mode) { fetch(`/mode?m=${mode}`); }

        let valJoyX = 0, valJoyY = 0, valJoyR = 0;
        let joyTimer;

        function pushJoyData() {
            clearTimeout(joyTimer);
            joyTimer = setTimeout(() => { fetch(`/joy?x=${valJoyX.toFixed(2)}&y=${valJoyY.toFixed(2)}&r=${valJoyR.toFixed(2)}`); }, 20);
        }

        // ===== HAREKET VEKTÖRÜ GÖSTERGESİ =====
        function drawVector() {
            const canvas = document.getElementById('vectorCanvas');
            if (!canvas) return;
            const ctx = canvas.getContext('2d');
            const W = 160, H = 160, cx = 80, cy = 80, R = 72;

            ctx.clearRect(0, 0, W, H);

            // Arka plan daire
            ctx.beginPath();
            ctx.arc(cx, cy, R, 0, Math.PI * 2);
            ctx.strokeStyle = '#2a2a2a';
            ctx.lineWidth = 1;
            ctx.stroke();
            ctx.fillStyle = '#0d0d0d';
            ctx.fill();

            // Izgara çizgileri
            ctx.strokeStyle = '#1e1e1e';
            ctx.lineWidth = 1;
            ctx.beginPath(); ctx.moveTo(cx, cy - R); ctx.lineTo(cx, cy + R); ctx.stroke();
            ctx.beginPath(); ctx.moveTo(cx - R, cy); ctx.lineTo(cx + R, cy); ctx.stroke();

            // Yön etiketleri
            ctx.fillStyle = '#333';
            ctx.font = '9px monospace';
            ctx.textAlign = 'center';
            ctx.fillText('İLERİ', cx, cy - R + 11);
            ctx.fillText('GERİ', cx, cy + R - 3);

            // Dönüş (YAW) göstergesi
            if (Math.abs(valJoyR) > 0.05) {
                let rotMag = Math.abs(valJoyR);
                let rotColor = valJoyR > 0 ? '#ffaa00' : '#ff6600';
                let sweepAngle = rotMag * Math.PI * 0.9;
                let startAngle = -Math.PI / 2;
                let endAngle = valJoyR > 0 ? startAngle + sweepAngle : startAngle - sweepAngle;
                ctx.strokeStyle = rotColor;
                ctx.lineWidth = 4;
                ctx.beginPath();
                ctx.arc(cx, cy, R * 0.55, startAngle, endAngle, valJoyR < 0);
                ctx.stroke();
                // Ok ucu (dönüş yönü)
                let tipAngle = endAngle + (valJoyR > 0 ? 0.25 : -0.25);
                let tx = cx + R * 0.55 * Math.cos(tipAngle);
                let ty = cy + R * 0.55 * Math.sin(tipAngle);
                ctx.fillStyle = rotColor;
                ctx.beginPath();
                ctx.arc(tx, ty, 4, 0, Math.PI * 2);
                ctx.fill();
            }

            // Doğrusal hareket vektörü oku
            let vx = valJoyX, vy = valJoyY;
            let mag = Math.sqrt(vx * vx + vy * vy);
            if (mag > 0.04) {
                let len = mag * (R * 0.82);
                let ax = vx * len;
                let ay = -vy * len;   // ekran Y aşağı, robot Y yukarı/ileri

                // Ok gövdesi
                ctx.strokeStyle = '#00ffcc';
                ctx.lineWidth = 3;
                ctx.lineCap = 'round';
                ctx.beginPath();
                ctx.moveTo(cx, cy);
                ctx.lineTo(cx + ax, cy + ay);
                ctx.stroke();

                // Ok başı
                let angle = Math.atan2(ay, ax);
                let hLen = 13;
                let hAng = 0.42;
                ctx.fillStyle = '#00ffcc';
                ctx.beginPath();
                ctx.moveTo(cx + ax, cy + ay);
                ctx.lineTo(cx + ax - hLen * Math.cos(angle - hAng), cy + ay - hLen * Math.sin(angle - hAng));
                ctx.lineTo(cx + ax - hLen * Math.cos(angle + hAng), cy + ay - hLen * Math.sin(angle + hAng));
                ctx.closePath();
                ctx.fill();

                // Hız yüzdesi
                ctx.fillStyle = '#00ffcc';
                ctx.font = 'bold 10px monospace';
                ctx.textAlign = 'center';
                ctx.fillText((mag * 100).toFixed(0) + '%', cx, cy + R - 4);
            } else if (Math.abs(valJoyR) < 0.05) {
                // Hareketsiz nokta
                ctx.beginPath();
                ctx.arc(cx, cy, 4, 0, Math.PI * 2);
                ctx.fillStyle = '#333';
                ctx.fill();
            }
        }

        const joyArea = document.getElementById('joyArea'); const joyKnob = document.getElementById('joyKnob');
        let isDragging1 = false;
        function updateJoy1(e) {
            if (!isDragging1) return;
            let rect = joyArea.getBoundingClientRect();
            let clientX = e.touches ? e.touches[0].clientX : e.clientX; let clientY = e.touches ? e.touches[0].clientY : e.clientY;
            let x = clientX - rect.left - rect.width / 2; let y = clientY - rect.top - rect.height / 2;
            let radius = rect.width / 2 - 35; let distance = Math.sqrt(x*x + y*y);
            if (distance > radius) { x = (x / distance) * radius; y = (y / distance) * radius; }
            joyKnob.style.transition = 'none'; joyKnob.style.transform = `translate(${x}px, ${y}px)`;
            valJoyX = x / radius; valJoyY = -y / radius;
            pushJoyData();
        }
        function resetJoy1() { isDragging1 = false; joyKnob.style.transition = '0.2s ease-out'; joyKnob.style.transform = `translate(0px, 0px)`; valJoyX = 0; valJoyY = 0; pushJoyData(); }
        joyArea.addEventListener('mousedown', (e) => { isDragging1 = true; sendMode('bezier_joy'); updateJoy1(e); });
        document.addEventListener('mousemove', updateJoy1); document.addEventListener('mouseup', resetJoy1);
        joyArea.addEventListener('touchstart', (e) => { isDragging1 = true; sendMode('bezier_joy'); updateJoy1(e); });
        joyArea.addEventListener('touchmove', (e) => { e.preventDefault(); updateJoy1(e); }, {passive: false});
        joyArea.addEventListener('touchend', resetJoy1);

        const joyAreaR = document.getElementById('joyAreaR'); const joyKnobR = document.getElementById('joyKnobR');
        let isDragging2 = false;
        function updateJoy2(e) {
            if (!isDragging2) return;
            let rect = joyAreaR.getBoundingClientRect();
            let clientX = e.touches ? e.touches[0].clientX : e.clientX;
            let x = clientX - rect.left - rect.width / 2;
            let radius = rect.width / 2 - 30;
            if (x > radius) x = radius; if (x < -radius) x = -radius;
            joyKnobR.style.transition = 'none'; joyKnobR.style.transform = `translate(${x}px, 0px)`;
            valJoyR = x / radius; pushJoyData();
        }
        function resetJoy2() { isDragging2 = false; joyKnobR.style.transition = '0.2s ease-out'; joyKnobR.style.transform = `translate(0px, 0px)`; valJoyR = 0; pushJoyData(); }
        joyAreaR.addEventListener('mousedown', (e) => { isDragging2 = true; sendMode('bezier_joy'); updateJoy2(e); });
        document.addEventListener('mousemove', updateJoy2); document.addEventListener('mouseup', resetJoy2);
        joyAreaR.addEventListener('touchstart', (e) => { isDragging2 = true; sendMode('bezier_joy'); updateJoy2(e); });
        joyAreaR.addEventListener('touchmove', (e) => { e.preventDefault(); updateJoy2(e); }, {passive: false});
        joyAreaR.addEventListener('touchend', resetJoy2);

        // Sabit tutulan joystick pointer event üretmese de firmware deadman'i
        // beslenir. Bırakma fonksiyonları valJoy* değerlerini doğrudan sıfırlar.
        setInterval(pushJoyData, 100);

        // ===== SES SİSTEMİ =====
        // prev/next yalnızca parça numarasını günceller — otomatik oynatma YAPILMAZ.
        // Çalmayı başlatmak için ▶ OYNAT butonuna basılması gerekir.
        let currentTrack = 1;
        let volDebounce;
        function audioCmd(cmd) {
            if (cmd === 'prev') { currentTrack = Math.max(1, currentTrack - 1); document.getElementById('trackDisp').innerText = currentTrack; fetch('/audio?cmd=prev&t=' + currentTrack); return; }
            if (cmd === 'next') { currentTrack = Math.min(99, currentTrack + 1); document.getElementById('trackDisp').innerText = currentTrack; fetch('/audio?cmd=next&t=' + currentTrack); return; }
            fetch('/audio?cmd=' + cmd + '&t=' + currentTrack);
        }
        function setVolume(v) {
            document.getElementById('vVol').innerText = v;
            clearTimeout(volDebounce);
            volDebounce = setTimeout(() => fetch('/audio?cmd=vol&v=' + v), 120);
        }

        function loadHomeAngles() {
            let leg = document.getElementById('homeLegSelect').value;
            fetch('/getHome?leg=' + leg).then(r => r.json()).then(d => {
                let c=Math.round(d.c), f=Math.round(d.f), t=Math.round(d.t);
                document.getElementById('hcSlider').value = c; document.getElementById('vhC').innerText = c;
                document.getElementById('hfSlider').value = f; document.getElementById('vhF').innerText = f;
                document.getElementById('htSlider').value = t; document.getElementById('vhT').innerText = t;
            });
        }
        function previewHomeJoint(joint, val) {
            let leg = document.getElementById('homeLegSelect').value;
            document.getElementById(['vhC','vhF','vhT'][joint]).innerText = val;
            fetch('/homePreview?leg=' + leg + '&j=' + joint + '&v=' + val);
        }
        function saveHomeLeg() {
            let leg = document.getElementById('homeLegSelect').value;
            let c = document.getElementById('hcSlider').value;
            let f = document.getElementById('hfSlider').value;
            let t = document.getElementById('htSlider').value;
            fetch('/saveHome?leg=' + leg + '&c=' + c + '&f=' + f + '&t=' + t)
                .then(() => { alert('Bacak ' + (parseInt(leg)+1) + ' HOME kaydedildi!'); });
        }
        function playHomeRise() {
            if (!confirm('Robot govde destek cikintilari uzerinde ve cevresi bos mu? Kontrollu CAPTURE -> STAND baslatilsin mi?')) return;
            fetch('/goHome?confirm=start');
        }
    </script>
</body>
</html>
)rawliteral";

// ================= DONANIM ÇIKIŞI =================
void writeLegPwmTick(int leg, int joint, int tick) {
  // PCA9685 repeats its pulse autonomously. Keep its frequency and pulse
  // values unchanged, and avoid redundant I2C traffic for stationary joints.
  if (lastLegPwmTick[leg][joint] == tick) return;
  uint8_t channel = legPins[leg][joint];
  uint8_t result = leg < 3 ? pwmRight.setPWM(channel, 0, tick)
                          : pwmLeft.setPWM(channel, 0, tick);
  if (result == 0) lastLegPwmTick[leg][joint] = tick;
}

void send_to_PCA9685(int leg, int joint, float target_angle) {
  if (leg < 0 || leg >= 6 || joint < 0 || joint >= 3) return;
  uint8_t channel = legPins[leg][joint];
  if (!legPwmEnabled[leg][joint]) {
    writeLegPwmTick(leg, joint, 0);
    return;
  }
  // currentPhysicalAngle is always a mechanical/command angle. Calibration
  // offset is applied exactly once, only while converting that angle to PWM.
  const float minPhysical = max(MIN_DEGREE, MIN_DEGREE - offset[leg][joint]);
  const float maxPhysical = min(MAX_DEGREE, MAX_DEGREE - offset[leg][joint]);
  const float safe_angle = constrain(target_angle, minPhysical, maxPhysical);

  if (firstRun) {
    if (homeSet) {
      // Home pozisyonuna doğrudan git — softStart buradan başlar
      currentPhysicalAngle[leg][joint] = homeAngles[leg][joint];
      float ang = constrain(homeAngles[leg][joint] + offset[leg][joint], MIN_DEGREE, MAX_DEGREE);
      int us  = pwmMin[leg][joint] + (int)((ang / 180.0f) * (pwmMax[leg][joint] - pwmMin[leg][joint]));
      int tick = (int)(us / 4.8828f);
      writeLegPwmTick(leg, joint, tick);
    } else {
      if      (joint == 0) currentPhysicalAngle[leg][joint] = 90.0f;
      else if (joint == 1) currentPhysicalAngle[leg][joint] = (leg < 3) ? 131.0f : 49.0f;
      else                 currentPhysicalAngle[leg][joint] = (leg < 3) ? 125.0f : 55.0f;
    }
    return;
  }

  float diff = safe_angle - currentPhysicalAngle[leg][joint];
  if (abs(diff) > maxDiffThisFrame) maxDiffThisFrame = abs(diff);

  float absDiff = fabs(diff);
  float alpha = safeTransitionActive ? 0.10f : 0.18f;
  alpha += min(absDiff, 20.0f) * 0.008f;
  if (safeTransitionActive) alpha = constrain(alpha, 0.06f, 0.22f);
  else                    alpha = constrain(alpha, 0.12f, 0.30f);
  alpha = 1.0f - powf(1.0f - alpha, controlFrameDtS / 0.020f);

  float maxStep;
  if      (currentMode == HOME_RISE) maxStep = STAND_MAX_SPEED_DEG_S * controlFrameDtS;
  else if (bootSoftStart)          maxStep = 0.5f + 2.0f * bootMotionBlend;
  else if (safeTransitionActive)   maxStep = SAFE_TRANSITION_MAX_SPEED_DEG_S * controlFrameDtS;
  else                             maxStep = LEG_TRACK_MAX_SPEED_DEG_S * controlFrameDtS;

  if (currentMode == HOME_RISE && homeRisePhase == 1) {
    // Quintic minimum-jerk interpolation: zero speed/acceleration at endpoints.
    // Do not stack an exponential low-pass filter onto this trajectory.
    const float t = constrain(bootMotionBlend, 0.0f, 1.0f);
    const float blend = t*t*t*(10.0f + t*(-15.0f + 6.0f*t));
    const float start = constrain(homeAngles[leg][joint], minPhysical, maxPhysical);
    const float desired = start + (safe_angle - start) * blend;
    currentPhysicalAngle[leg][joint] += constrain(desired - currentPhysicalAngle[leg][joint], -maxStep, maxStep);
  }
  else if (absDiff < 0.04f) currentPhysicalAngle[leg][joint] = safe_angle;
  else {
    float step = constrain(diff * alpha, -maxStep, maxStep);
    currentPhysicalAngle[leg][joint] += step;
  }

  int min_us = pwmMin[leg][joint];
  int max_us = pwmMax[leg][joint];
  const float electricalAngle = constrain(currentPhysicalAngle[leg][joint] + offset[leg][joint], MIN_DEGREE, MAX_DEGREE);
  int us = min_us + (int)((electricalAngle / 180.0) * (max_us - min_us));
  int tick = (int)(us / 4.8828f);

  writeLegPwmTick(leg, joint, tick);
}

void writeLegAngleImmediate(int leg, int joint, float targetAngle) {
  if (leg < 0 || leg >= 6 || joint < 0 || joint >= 3) return;
  if (!legPwmEnabled[leg][joint]) return;

  const float physical = constrain(targetAngle, MIN_DEGREE, MAX_DEGREE);
  const float electrical = constrain(physical + offset[leg][joint], MIN_DEGREE, MAX_DEGREE);
  const int us = pwmMin[leg][joint] +
                 (int)((electrical / 180.0f) * (pwmMax[leg][joint] - pwmMin[leg][joint]));
  const int tick = (int)(us / 4.8828f);
  currentPhysicalAngle[leg][joint] = physical;
  const uint8_t channel = legPins[leg][joint];
  writeLegPwmTick(leg, joint, tick);
}

bool beginStandupSequence(String &reason) {
  if (spineState != SPINE_DISARMED) {
    reason = "spine must be DISARMED";
    return false;
  }
  if (!homeSet) {
    reason = "HOME/CAPTURE pose is not stored";
    return false;
  }
  if (!configValid) {
    reason = "servo calibration/HOME validation failed";
    return false;
  }
  if (!pcaRightOnline || !pcaLeftOnline) {
    reason = "both PCA9685 controllers must be online";
    return false;
  }

  joyX = joyY = joyR = 0.0f;
  s_joyX = s_joyY = s_joyR = 0.0f;
  bodyPitch = bodyRoll = bodyYaw = 0.0f;
  s_pitch = s_roll = s_yaw = 0.0f;
  gimbalActive = false;
  linkFailsafeActive = false;
  motionCommandSeen = false;

  // The chassis is resting on its mechanical support protrusions. HOME is the
  // repeatable CAPTURE pose selected by the operator, not an inferred angle.
  enableLegPwm(-1, -1);
  for (int leg = 0; leg < 6; leg++) {
    for (int joint = 0; joint < 3; joint++) {
      writeLegAngleImmediate(leg, joint, homeAngles[leg][joint]);
    }
  }

  firstRun = false;
  manualTargetsInitialized = false;
  homeRisePhase = 0;
  homeRisePhaseMs = millis();
  bootSoftStart = false;
  bootMotionBlend = 1.0f;
  safeTransitionActive = true;
  safeTransitionStartMs = millis();
  currentMode = HOME_RISE;
  spineState = SPINE_CAPTURE;
  addLog("[SAFETY] CAPTURE pozu etkin. Kontrollu kalkis operator komutuyla basladi.\n");
  reason = "capture started";
  return true;
}

// ================= 3D EULER MATRİSİ =================
void calculate_body_ik_and_move(int leg, float targetX, float targetY, float targetZ) {
  // Canonical robot frame: +X ileri, +Y sol, +Z yukarı.
  // Pozitif roll: sol taraf yukarı; pozitif pitch: burun aşağı;
  // pozitif yaw: yukarıdan bakınca sola/CCW dönüş (sağ-el kuralı).
  const float roll  = s_roll  * PI / 180.0f;
  const float pitch = s_pitch * PI / 180.0f;
  const float yaw   = s_yaw   * PI / 180.0f;

  // Nominal dünya/robot ayak noktası: hip offset + gait target.
  const float footX = bodyOffsetX[leg] + targetX;
  const float footY = bodyOffsetY[leg] + targetY;
  const float footZ = targetZ;

  // Body orientation compensation = R^T * p.
  // Standart ZYX gövde yönelimi R = Rz(yaw) Ry(pitch) Rx(roll) ise
  // inverse dönüş R^T = Rx(-roll) Ry(-pitch) Rz(-yaw) olur.
  // Vektöre uygulama sırası: önce Rz(-yaw), sonra Ry(-pitch), sonra Rx(-roll).
  const float cy = cosf(yaw),   sy = sinf(yaw);
  const float cp = cosf(pitch), sp = sinf(pitch);
  const float cr = cosf(roll),  sr = sinf(roll);

  // 1) Rz(-yaw)
  const float x1 =  cy * footX + sy * footY;
  const float y1 = -sy * footX + cy * footY;
  const float z1 =  footZ;

  // 2) Ry(-pitch)
  const float x2 =  cp * x1 - sp * z1;
  const float y2 =  y1;
  const float z2 =  sp * x1 + cp * z1;

  // 3) Rx(-roll)
  const float x3 = x2;
  const float y3 = cr * y2 + sr * z2;
  const float z3 = -sr * y2 + cr * z2;

  // Hip merkezine göre yerel bacak hedefi.
  const float ikX = x3 - bodyOffsetX[leg];
  const float dyBody = y3 - bodyOffsetY[leg];

  // Yerel IK'da outward her iki tarafta da pozitif olsun:
  // sağ bacakların dışı body -Y, sol bacakların dışı body +Y.
  const float ikYOut = (leg < 3) ? -dyBody : dyBody;
  const float ikZ = z3;

  float d = sqrtf(ikX * ikX + ikYOut * ikYOut);
  float r = d - l;
  if (r < 5.0f) r = 5.0f;

  float c = sqrtf(ikZ * ikZ + r * r);
  const float cMax = (a + b) * 0.999f;
  const float cMin = fabsf(a - b) * 1.001f;
  c = constrain(c, cMin, cMax);

  const float thetaCoxaGeom = atan2f(ikYOut, ikX) * 180.0f / PI;
  const float cosFemur = constrain((a*a + c*c - b*b) / (2.0f*a*c), -1.0f, 1.0f);
  const float cosTibia = constrain((a*a + b*b - c*c) / (2.0f*a*b), -1.0f, 1.0f);
  const float thetaFemurGeom = atan2f(r, -ikZ) * 180.0f / PI + acosf(cosFemur) * 180.0f / PI;
  const float thetaTibiaGeom = 180.0f - acosf(cosTibia) * 180.0f / PI;

  if (!isfinite(thetaCoxaGeom) || !isfinite(thetaFemurGeom) || !isfinite(thetaTibiaGeom)) return;

  const float servoCoxa  = mapIkToServoDeg(leg, 0, thetaCoxaGeom);
  const float servoFemur = mapIkToServoDeg(leg, 1, thetaFemurGeom);
  const float servoTibia = mapIkToServoDeg(leg, 2, thetaTibiaGeom);

  // Aşırı/ulaşılamaz komutta sessizce yanlış poza gitmek yerine o frame'i atla.
  if (servoCoxa < -5.0f || servoCoxa > 185.0f ||
      servoFemur < -5.0f || servoFemur > 185.0f ||
      servoTibia < -5.0f || servoTibia > 185.0f) return;

  send_to_PCA9685(leg, 0, servoCoxa);
  send_to_PCA9685(leg, 1, servoFemur);
  send_to_PCA9685(leg, 2, servoTibia);
}

void loadMemory() {
  preferences.begin("apex_calib2", true);
  for(int i=0; i<6; i++) {
    String pfx = String(i);
    pwmMin[i][0] = preferences.getInt((pfx+"cMin").c_str(), 500); pwmMax[i][0] = preferences.getInt((pfx+"cMax").c_str(), 2500); offset[i][0] = preferences.getInt((pfx+"cOff").c_str(), 0);
    pwmMin[i][1] = preferences.getInt((pfx+"fMin").c_str(), 500); pwmMax[i][1] = preferences.getInt((pfx+"fMax").c_str(), 2500); offset[i][1] = preferences.getInt((pfx+"fOff").c_str(), 0);
    pwmMin[i][2] = preferences.getInt((pfx+"tMin").c_str(), 500); pwmMax[i][2] = preferences.getInt((pfx+"tMax").c_str(), 2500); offset[i][2] = preferences.getInt((pfx+"tOff").c_str(), 0);
  }
  homeSet = preferences.getBool("homeSet", false);
  for (int i = 0; i < 6; i++) {
    String pfx = String(i);
    homeAngles[i][0] = preferences.getFloat((pfx+"hC").c_str(), 90.0f);
    homeAngles[i][1] = preferences.getFloat((pfx+"hF").c_str(), (i<3)?131.0f:49.0f);
    homeAngles[i][2] = preferences.getFloat((pfx+"hT").c_str(), (i<3)?125.0f:55.0f);
  }
  preferences.end();
}

bool validateLegConfiguration() {
  bool valid = true;
  for (int leg = 0; leg < 6; leg++) {
    for (int joint = 0; joint < 3; joint++) {
      if (pwmMin[leg][joint] < 400 || pwmMax[leg][joint] > 2600 ||
          pwmMin[leg][joint] >= pwmMax[leg][joint] ||
          offset[leg][joint] < -90 || offset[leg][joint] > 90 ||
          !isfinite(homeAngles[leg][joint]) ||
          homeAngles[leg][joint] < MIN_DEGREE || homeAngles[leg][joint] > MAX_DEGREE) {
        valid = false;
      }
    }
  }
  if (!valid) {
    homeSet = false;
    addLog("[SAFETY] NVS servo/HOME verisi gecersiz; kalkis kilitlendi.\n");
  }
  return valid;
}


// ================= ROBOT KOL PWM ÇIKIŞI =================
int armJointIndexFromId(String id) {
  id.trim(); id.toLowerCase();
  if (id == "0") return ARM_BASE;
  if (id == "1") return ARM_JOINT1;
  if (id == "2") return ARM_JOINT2;
  if (id == "3") return ARM_GRIPPER;
  if (id == "b" || id == "base") return ARM_BASE;
  if (id == "j1" || id == "joint1" || id == "shoulder") return ARM_JOINT1;
  if (id == "j2" || id == "joint2" || id == "elbow") return ARM_JOINT2;
  if (id == "g" || id == "gripper" || id == "grip") return ARM_GRIPPER;
  return -1;
}

float armClampAngle(uint8_t joint, float angleDeg) {
  if (joint >= ARM_JOINT_COUNT) return 90.0f;
  return constrain(angleDeg, armMinDeg[joint], armMaxDeg[joint]);
}

int armAngleToTick(uint8_t joint, float angleDeg) {
  float safeAngle = armClampAngle(joint, angleDeg);
  int us = armPwmMinUs[joint] + (int)((safeAngle / 180.0f) * (armPwmMaxUs[joint] - armPwmMinUs[joint]));
  us = constrain(us, 100, 3000);
  return (int)(us / 4.8828125f);  // 50Hz: 20ms / 4096 = 4.8828125us/tick
}

void armWriteJointNow(uint8_t joint, float angleDeg) {
  if (joint >= ARM_JOINT_COUNT) return;
  float safeAngle = armClampAngle(joint, angleDeg);
  int tick = armAngleToTick(joint, safeAngle);
  pwmRight.setPWM(armPins[joint], 0, tick);
  armCurrentAngle[joint] = safeAngle;
}

void armSetTarget(uint8_t joint, float angleDeg, bool immediate=false) {
  if (joint >= ARM_JOINT_COUNT) return;
  float safeAngle = armClampAngle(joint, angleDeg);
  armTargetAngle[joint] = safeAngle;
  armOutputEnabled = true;
  if (immediate) armWriteJointNow(joint, safeAngle);
}

void armSetPoseTargets(float base, float joint1, float joint2, float gripper, bool immediate=false) {
  armSetTarget(ARM_BASE,    base,    immediate);
  armSetTarget(ARM_JOINT1,  joint1,  immediate);
  armSetTarget(ARM_JOINT2,  joint2,  immediate);
  armSetTarget(ARM_GRIPPER, gripper, immediate);
}

void armUpdate() {
  if (!armOutputEnabled) return;
  unsigned long now = millis();
  if (now - lastArmUpdateMs < 20) return;
  lastArmUpdateMs = now;

  for (uint8_t j = 0; j < ARM_JOINT_COUNT; j++) {
    float diff = armTargetAngle[j] - armCurrentAngle[j];
    if (fabsf(diff) < 0.05f) {
      armWriteJointNow(j, armTargetAngle[j]);
    } else {
      float step = constrain(diff, -ARM_MAX_STEP_DEG, ARM_MAX_STEP_DEG);
      armWriteJointNow(j, armCurrentAngle[j] + step);
    }
  }
}

void armDisablePwm() {
  for (uint8_t j = 0; j < ARM_JOINT_COUNT; j++) pwmRight.setPWM(armPins[j], 0, 0);
  armOutputEnabled = false;
}

void loadArmMemory() {
  preferences.begin("apex_arm", true);
  for (uint8_t j = 0; j < ARM_JOINT_COUNT; j++) {
    String k = String(j);
    armMinDeg[j]    = preferences.getFloat((String("amin") + k).c_str(), armMinDeg[j]);
    armMaxDeg[j]    = preferences.getFloat((String("amax") + k).c_str(), armMaxDeg[j]);
    armPwmMinUs[j]  = preferences.getInt((String("umin") + k).c_str(), armPwmMinUs[j]);
    armPwmMaxUs[j]  = preferences.getInt((String("umax") + k).c_str(), armPwmMaxUs[j]);
    if (armMinDeg[j] > armMaxDeg[j]) { float tmp = armMinDeg[j]; armMinDeg[j] = armMaxDeg[j]; armMaxDeg[j] = tmp; }
    armMinDeg[j] = constrain(armMinDeg[j], 5.0f, 175.0f);
    armMaxDeg[j] = constrain(armMaxDeg[j], 5.0f, 175.0f);
  }
  armGripperOpenDeg  = 90.0f;
  armGripperCloseDeg = preferences.getFloat("gclose", armMaxDeg[ARM_GRIPPER]);

  // Eski firmware'den kalmış preferences varsa yeni mekanik gerçek zorla uygulanır:
  // gripper 90° açık, 90° altına inmez; kapalı açı her zaman MAX'tir.
  armMinDeg[ARM_GRIPPER] = 90.0f;
  armMaxDeg[ARM_GRIPPER] = constrain(armMaxDeg[ARM_GRIPPER], 90.0f, 175.0f);
  armGripperOpenDeg = 90.0f;
  armGripperCloseDeg = armMaxDeg[ARM_GRIPPER];
  preferences.end();

  // Açılışta fiziksel PWM basma; sadece hedef/current değerlerini güvenli aralığa oturt.
  for (uint8_t j = 0; j < ARM_JOINT_COUNT; j++) {
    armCurrentAngle[j] = armClampAngle(j, armCalibrationPose[j]);
    armTargetAngle[j]  = armCurrentAngle[j];
  }
  addLog("[ARM] PWM hazir. J1:5 katli/175 one acik, gripper:90 acik/MAX kapali. /armCalibrate ile 90 dereceye al.\n");
}

void saveArmMemory() {
  preferences.begin("apex_arm", false);
  for (uint8_t j = 0; j < ARM_JOINT_COUNT; j++) {
    String k = String(j);
    preferences.putFloat((String("amin") + k).c_str(), armMinDeg[j]);
    preferences.putFloat((String("amax") + k).c_str(), armMaxDeg[j]);
    preferences.putInt((String("umin") + k).c_str(), armPwmMinUs[j]);
    preferences.putInt((String("umax") + k).c_str(), armPwmMaxUs[j]);
  }
  preferences.putFloat("gopen", armGripperOpenDeg);
  preferences.putFloat("gclose", armGripperCloseDeg);
  preferences.end();
}

void armApplyConfigArgs(bool persist=false) {
  // Python arayüzünden veya direkt curl'den gelen gripper limitlerini alır.
  // Rack-and-pinion gripper: 90° açık sabit; kapatma 90° -> 180° yönünde.
  // gmin/gopen gelse bile güvenlik için 90'a sabitlenir; gmax veya gclose kapalı/MAX açısını belirler.
  armMinDeg[ARM_GRIPPER] = 90.0f;
  if (server.hasArg("gmax") || server.hasArg("grMax") || server.hasArg("gripperMax")) {
    float v = server.hasArg("gmax") ? server.arg("gmax").toFloat() : (server.hasArg("grMax") ? server.arg("grMax").toFloat() : server.arg("gripperMax").toFloat());
    armMaxDeg[ARM_GRIPPER] = constrain(v, 90.0f, 175.0f);
  }
  if (server.hasArg("gclose")) {
    armMaxDeg[ARM_GRIPPER] = constrain(server.arg("gclose").toFloat(), 90.0f, 175.0f);
  }
  armGripperOpenDeg = 90.0f;
  armGripperCloseDeg = armMaxDeg[ARM_GRIPPER];

  armTargetAngle[ARM_GRIPPER]  = armClampAngle(ARM_GRIPPER, armTargetAngle[ARM_GRIPPER]);
  armCurrentAngle[ARM_GRIPPER] = armClampAngle(ARM_GRIPPER, armCurrentAngle[ARM_GRIPPER]);
  if (persist) saveArmMemory();
}

String armStatusJson() {
  String json = "{";
  json += "\"enabled\":" + String(armOutputEnabled ? "true" : "false") + ",";
  json += "\"pca\":\"0x40\",";
  json += "\"channels\":{\"base\":12,\"joint1\":13,\"joint2\":14,\"gripper\":15},";
  json += "\"current\":{";
  for (uint8_t j = 0; j < ARM_JOINT_COUNT; j++) {
    if (j) json += ",";
    json += "\"" + String(armNames[j]) + "\":" + String(armCurrentAngle[j], 1);
  }
  json += "},\"target\":{";
  for (uint8_t j = 0; j < ARM_JOINT_COUNT; j++) {
    if (j) json += ",";
    json += "\"" + String(armNames[j]) + "\":" + String(armTargetAngle[j], 1);
  }
  json += "},\"limits\":{";
  for (uint8_t j = 0; j < ARM_JOINT_COUNT; j++) {
    if (j) json += ",";
    json += "\"" + String(armNames[j]) + "\":{";
    json += "\"min\":" + String(armMinDeg[j], 1) + ",\"max\":" + String(armMaxDeg[j], 1) + "}";
  }
  json += "},\"gripper\":{";
  json += "\"open\":" + String(armGripperOpenDeg, 1) + ",\"close\":" + String(armGripperCloseDeg, 1);
  json += "}}";
  return json;
}

// ================= HTTP SERVER HANDLERS =================
void setup() {
  Serial.begin(115200);
  setStatusLed(0, 0, 5);
  Serial.println("[APEX] " APEX_FIRMWARE_VERSION " / " APEX_BUILD_PROFILE);

  gpsSerial.setRxBufferSize(1024);
  gpsSerial.begin(9600, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);

  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(400000);
  initBatteryMonitor();

  dfSerial.begin(9600, SERIAL_8N1, DF_RX_PIN, DF_TX_PIN);
  delay(200);
  if (dfPlayer.begin(dfSerial)) {
    dfReady = true;
    dfPlayer.volume(dfVolume);
    addLog("[AUDIO] DFPlayer HAZIR. Vol=" + String(dfVolume) + "\n");
    bootDiagLog += "[AUDIO] DFPlayer HAZIR\n";
  } else {
    addLog("[-] DFPlayer BULUNAMADI (Pin TX:4 RX:5)\n");
    bootDiagLog += "[-] DFPlayer BULUNAMADI\n";
  }

  // Probe first. Some third-party sensor initializers contain unbounded waits
  // when their board is unpowered; safe boot must remain operational with the
  // rest of the spine deliberately powered down.
  scanI2C();
  initMPU6050();
  calibrateMPU6050Gyro();
  initTOF400C();

  pwmRight.begin(); pwmRight.setPWMFreq(50);
  pwmLeft.begin();  pwmLeft.setPWMFreq(50);
  // PCA power-up state is not trusted. Explicitly force every leg and arm
  // channel OFF before loading any stored pose or accepting network traffic.
  disableLegPwm(-1, -1);
  armDisablePwm();
  loadMemory();
  configValid = validateLegConfiguration();
  loadArmMemory();
  // Software angle state is seeded without writing PWM. The physical servos
  // remain unknown until the operator starts the repeatable CAPTURE phase.
  for (int leg = 0; leg < 6; leg++) {
    for (int joint = 0; joint < 3; joint++) {
      currentPhysicalAngle[leg][joint] = constrain(homeAngles[leg][joint], MIN_DEGREE, MAX_DEGREE);
    }
  }
  firstRun = false;
  initManualTargetsFromCurrent();

  currentMode      = IDLE;
  homeRisePhase    = 0;
  homeRisePhaseMs  = 0;
  bootSoftStart    = false;
  bootMotionBlend  = 1.0f;
  spineState       = SPINE_DISARMED;
  legOutputsArmed  = false;
  addLog("[SAFETY] SAFE BOOT: tum servo PWM cikislari kapali; operator komutu bekleniyor.\n");

  // APEX-HUB üzerinde sabit IP — DHCP'den bağımsız, MAC rezervasyonuna gerek yok.
  IPAddress staticIP(192, 168, 7, 50);
  IPAddress gateway(192, 168, 7, 1);
  IPAddress subnet(255, 255, 255, 0);
  WiFi.config(staticIP, gateway, subnet);
  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);
  WiFi.setAutoReconnect(true);
  WiFi.begin(ssid, password);
  unsigned long t = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - t < 10000) { delay(500); }

  if(WiFi.status() == WL_CONNECTED) {
      addLog("[WIFI] UPLINK ESTABLISHED. IP: " + WiFi.localIP().toString() + "\n");
      Serial.println("[WIFI] connected ip=" + WiFi.localIP().toString());
  } else {
      addLog("[-] WIFI UPLINK YOK; yerel kontrol sunucusu acik kalacak.\n");
      Serial.println("[WIFI] offline status=" + String((int)WiFi.status()));
  }

  server.on("/", []() { server.send(200, "text/html", INDEX_HTML); });

  server.on("/telemetry", []() {
    String json = "{";
    String safeLogs = sysLogs;
    safeLogs.replace("\\", "\\\\"); safeLogs.replace("\"", "\\\""); safeLogs.replace("\r", ""); safeLogs.replace("\n", "\\n");
    json += "\"logs\": \"" + safeLogs + "\",";
    String safeBootDiag = bootDiagLog;
    safeBootDiag.replace("\\", "\\\\"); safeBootDiag.replace("\"", "\\\""); safeBootDiag.replace("\r", ""); safeBootDiag.replace("\n", "\\n");
    json += "\"bootDiag\": \"" + safeBootDiag + "\",";
    json += "\"firmwareVersion\": \"" APEX_FIRMWARE_VERSION "\",";
    json += "\"buildProfile\": \"" APEX_BUILD_PROFILE "\",";
    json += "\"spineState\": \"" + String(spineStateName()) + "\",";
    json += "\"outputsArmed\": " + String(legOutputsArmed ? "true" : "false") + ",";
    json += "\"pcaRightOnline\": " + String(pcaRightOnline ? "true" : "false") + ",";
    json += "\"pcaLeftOnline\": " + String(pcaLeftOnline ? "true" : "false") + ",";
    json += "\"mpuOnline\": " + String(mpuOnline ? "true" : "false") + ",";
    json += "\"configValid\": " + String(configValid ? "true" : "false") + ",";
    json += "\"rgbPin\": " + String(PIN_RGB_LED) + ",";
    json += "\"fix\": " + String(gpsHasFix ? "true" : "false") + ",";
    json += "\"lat\": " + String(currentLat, 6) + ",";
    json += "\"lng\": " + String(currentLng, 6) + ",";
    json += "\"sats\": " + String(gps.satellites.value()) + ",";
    json += "\"mpuP\": " + String(mpuPitch) + ",";
    json += "\"mpuR\": " + String(mpuRoll) + ",";
    json += "\"mpuY\": " + String(mpuYaw, 2) + ",";
    json += "\"z\": " + String((int)gaitZ) + ",";
    json += "\"gaitParams\":{\"z\":" + String(gaitZ, 1) + ",\"len\":" + String(stepLen, 1) + ",\"lift\":" + String(stepLift, 1) + ",\"spd\":" + String(baseSpeed, 1) + "},";
    json += "\"vx\": " + String(s_joyX, 3) + ",";
    json += "\"vy\": " + String(s_joyY, 3) + ",";
    json += "\"gP\": " + String(s_gimbalPitch, 2) + ",";
    json += "\"gR\": " + String(s_gimbalRoll,  2) + ",";
    json += "\"gaitPhase\": " + String(currentPhase, 4) + ",";
    json += "\"homeSet\": " + String(homeSet ? "true" : "false") + ",";
    json += "\"cmdAgeMs\": " + String(motionCommandSeen ? (long)(millis() - lastMotionCommandMs) : -1L) + ",";
    json += "\"failsafe\": " + String(linkFailsafeActive ? "true" : "false") + ",";
    json += "\"failsafeCount\": " + String(motionFailsafeCount) + ",";
    json += "\"tofOnline\": " + String(tofReady ? "true" : "false") + ",";
    json += "\"tofMm\": " + String(tofDistanceMm) + ",";
    json += "\"batR\": " + String(batteryRightV, 3) + ",";
    json += "\"batL\": " + String(batteryLeftV, 3) + ",";
    json += "\"batRCell\": " + String(batteryRightV / 3.0f, 3) + ",";
    json += "\"batLCell\": " + String(batteryLeftV / 3.0f, 3) + ",";
    json += "\"batRAdcMv\": " + String(batteryRightAdcMv) + ",";
    json += "\"batLAdcMv\": " + String(batteryLeftAdcMv) + ",";
    json += "\"batRCal\": " + String(BAT_RIGHT_CAL_FACTOR, 5) + ",";
    json += "\"batLCal\": " + String(BAT_LEFT_CAL_FACTOR, 5) + ",";

    String mStr = "IDLE";
    if(currentMode == CAL) mStr = "CAL(KILITLI)";
    else if(currentMode == BEZIER_JOY) mStr = "YURUYUS";
    else if(currentMode == MANUAL) mStr = "MANUEL TEST";
    else if(currentMode == HOME_RISE) mStr = "KALKIS";
    json += "\"mode\": \"" + mStr + "\"";
    json += "}";
    server.send(200, "application/json", json);
  });

  // Batarya kalibrasyonu: multimetrede görülen gerçek pack voltajını gir.
  // Örnek: /batteryCal?side=R&actual=12.60
  // Yeni katsayı ADC mV ve divider oranından hesaplanır, NVS'ye kaydedilir.
  server.on("/batteryCal", []() {
    String side = server.arg("side");
    side.toUpperCase();
    float actual = server.arg("actual").toFloat();
    if (actual < 1.0f || actual > 20.0f) {
      server.send(400, "application/json", "{\"ok\":false,\"error\":\"actual out of range\"}");
      return;
    }

    float baseV = 0.0f;
    float newFactor = 1.0f;
    preferences.begin("apex_power", false);
    if (side == "R") {
      baseV = (batteryRightAdcMv / 1000.0f) * BAT_RIGHT_DIVIDER_RATIO;
      if (baseV < 0.1f) { preferences.end(); server.send(409, "application/json", "{\"ok\":false,\"error\":\"right adc not ready\"}"); return; }
      newFactor = actual / baseV;
      BAT_RIGHT_CAL_FACTOR = newFactor;
      preferences.putFloat("batRCal", BAT_RIGHT_CAL_FACTOR);
    } else if (side == "L") {
      baseV = (batteryLeftAdcMv / 1000.0f) * BAT_LEFT_DIVIDER_RATIO;
      if (baseV < 0.1f) { preferences.end(); server.send(409, "application/json", "{\"ok\":false,\"error\":\"left adc not ready\"}"); return; }
      newFactor = actual / baseV;
      BAT_LEFT_CAL_FACTOR = newFactor;
      preferences.putFloat("batLCal", BAT_LEFT_CAL_FACTOR);
    } else {
      preferences.end();
      server.send(400, "application/json", "{\"ok\":false,\"error\":\"side must be R or L\"}");
      return;
    }
    preferences.end();
    String out = "{\"ok\":true,\"side\":\"" + side + "\",\"actual\":" + String(actual,3) + ",\"factor\":" + String(newFactor,5) + "}";
    server.send(200, "application/json", out);
  });

  server.on("/setGimbal", []() {
    const bool requested = (server.arg("state") == "1");
    if (requested && !mpuOnline) {
      server.send(409, "application/json", "{\"ok\":false,\"error\":\"MPU offline\"}");
      return;
    }
    if (requested && spineState != SPINE_STAND && spineState != SPINE_WALK) {
      server.send(409, "application/json", "{\"ok\":false,\"error\":\"gimbal requires STAND/WALK\"}");
      return;
    }
    gimbalActive = requested;
    if (gimbalActive) {
      gimbalRefPitch = s_mpuPitch;
      gimbalRefRoll  = s_mpuRoll;
      s_gimbalPitch  = 0.0f;
      s_gimbalRoll   = 0.0f;
      addLog("[GIMBAL] REFERANS ALINDI. P=" + String(gimbalRefPitch,1) + " R=" + String(gimbalRefRoll,1) + "\n");
    } else {
      addLog("[SYS] GIMBAL KAPATILDI. MANUEL IK DEVREDE.");
    }
    server.send(200, "text/plain", "OK");
  });

  server.on("/setGimbalFilter", []() {
    if (server.hasArg("v")) gimbalFilterLevel = constrain(server.arg("v").toFloat(), 0.0f, 100.0f);
    server.send(200, "text/plain", "OK");
  });

  server.on("/getCalib", []() {
    int leg = server.arg("leg").toInt();
    if (leg < 0 || leg > 5) { server.send(400, "text/plain", "bad leg"); return; }
    String json = "{";
    json += "\"cMin\":" + String(pwmMin[leg][0]) + ",\"cMax\":" + String(pwmMax[leg][0]) + ",\"cOff\":" + String(offset[leg][0]) + ",";
    json += "\"fMin\":" + String(pwmMin[leg][1]) + ",\"fMax\":" + String(pwmMax[leg][1]) + ",\"fOff\":" + String(offset[leg][1]) + ",";
    json += "\"tMin\":" + String(pwmMin[leg][2]) + ",\"tMax\":" + String(pwmMax[leg][2]) + ",\"tOff\":" + String(offset[leg][2]);
    json += "}";
    server.send(200, "application/json", json);
  });

  server.on("/saveCalib", []() {
    int leg = server.arg("leg").toInt();
    if (leg < 0 || leg > 5) { server.send(400, "text/plain", "bad leg"); return; }
    int newMin[3] = {pwmMin[leg][0], pwmMin[leg][1], pwmMin[leg][2]};
    int newMax[3] = {pwmMax[leg][0], pwmMax[leg][1], pwmMax[leg][2]};
    int newOff[3] = {offset[leg][0], offset[leg][1], offset[leg][2]};
    if(server.hasArg("cMin")) newMin[0] = server.arg("cMin").toInt();
    if(server.hasArg("cMax")) newMax[0] = server.arg("cMax").toInt();
    if(server.hasArg("cOff")) newOff[0] = server.arg("cOff").toInt();
    if(server.hasArg("fMin")) newMin[1] = server.arg("fMin").toInt();
    if(server.hasArg("fMax")) newMax[1] = server.arg("fMax").toInt();
    if(server.hasArg("fOff")) newOff[1] = server.arg("fOff").toInt();
    if(server.hasArg("tMin")) newMin[2] = server.arg("tMin").toInt();
    if(server.hasArg("tMax")) newMax[2] = server.arg("tMax").toInt();
    if(server.hasArg("tOff")) newOff[2] = server.arg("tOff").toInt();
    for (int j = 0; j < 3; j++) {
      if (newMin[j] < 400 || newMax[j] > 2600 || newMin[j] >= newMax[j] || newOff[j] < -90 || newOff[j] > 90) {
        server.send(422, "application/json", "{\"ok\":false,\"error\":\"invalid pwm calibration\"}");
        return;
      }
    }
    preferences.begin("apex_calib2", false);
    String pfx = String(leg);
    const char* names[3] = {"c", "f", "t"};
    for (int j = 0; j < 3; j++) {
      pwmMin[leg][j] = newMin[j]; pwmMax[leg][j] = newMax[j]; offset[leg][j] = newOff[j];
      preferences.putInt((pfx + names[j] + "Min").c_str(), newMin[j]);
      preferences.putInt((pfx + names[j] + "Max").c_str(), newMax[j]);
      preferences.putInt((pfx + names[j] + "Off").c_str(), newOff[j]);
    }
    preferences.end();
    configValid = validateLegConfiguration();
    server.send(200, "text/plain", "OK");
  });

  server.on("/manual", []() {
    int leg = server.arg("leg").toInt();
    int j = server.arg("j").toInt();
    float v = server.arg("v").toFloat();
    if (leg < 0 || leg > 5 || j < 0 || j > 2) { server.send(400, "text/plain", "bad leg/joint"); return; }
    setManualTarget(leg, j, v);
    server.send(200, "application/json", "{\"ok\":true,\"angle\":" + String(manualTargetAngle[leg][j],1) + "}");
  });

  // Açık isimli yeni endpoint: eski /manual ile aynı altyapıyı kullanır.
  server.on("/legServo", []() {
    int leg = server.arg("leg").toInt();
    int j = server.arg("j").toInt();
    float angle = server.arg("angle").toFloat();
    if (leg < 0 || leg > 5 || j < 0 || j > 2) { server.send(400, "text/plain", "bad leg/joint"); return; }
    setManualTarget(leg, j, angle);
    server.send(200, "application/json", "{\"ok\":true,\"leg\":" + String(leg) + ",\"joint\":" + String(j) + ",\"angle\":" + String(manualTargetAngle[leg][j],1) + "}");
  });

  server.on("/legMountPose", []() {
    int leg = server.hasArg("leg") ? server.arg("leg").toInt() : -1;
    if (leg < -1 || leg > 5) { server.send(400, "text/plain", "bad leg"); return; }
    setLegMountTargets(leg);
    server.send(200, "application/json", "{\"ok\":true,\"pose\":\"mount\"}");
  });

  server.on("/legHomePose", []() {
    int leg = server.hasArg("leg") ? server.arg("leg").toInt() : -1;
    if (leg < -1 || leg > 5) { server.send(400, "text/plain", "bad leg"); return; }
    setLegHomeTargets(leg);
    server.send(200, "application/json", "{\"ok\":true,\"pose\":\"home\"}");
  });

  server.on("/legOff", []() {
    int leg = server.hasArg("leg") ? server.arg("leg").toInt() : -1;
    int j   = server.hasArg("j")   ? server.arg("j").toInt()   : -1;
    if (leg < -1 || leg > 5 || j < -1 || j > 2) { server.send(400, "text/plain", "bad leg/joint"); return; }
    disableLegPwm(leg, j);
    if (leg < 0 && j < 0) {
      joyX = joyY = joyR = 0.0f;
      s_joyX = s_joyY = s_joyR = 0.0f;
      gimbalActive = false;
      currentMode = IDLE;
      addLog("[SAFETY] Tum bacak PWM cikislari operator tarafindan kapatildi.\n");
    }
    server.send(200, "application/json", "{\"ok\":true,\"pwm\":\"off\"}");
  });

  server.on("/legOn", []() {
    int leg = server.hasArg("leg") ? server.arg("leg").toInt() : -1;
    int j   = server.hasArg("j")   ? server.arg("j").toInt()   : -1;
    if (leg < -1 || leg > 5 || j < -1 || j > 2) { server.send(400, "text/plain", "bad leg/joint"); return; }
    if (leg < 0) {
      server.send(409, "application/json", "{\"ok\":false,\"error\":\"all-leg arm is only allowed through /goHome?confirm=start\"}");
      return;
    }
    if (server.arg("confirm") != "lab" || currentMode != MANUAL) {
      server.send(409, "application/json", "{\"ok\":false,\"error\":\"select MANUAL target, then use confirm=lab\"}");
      return;
    }
    if ((leg < 3 && !pcaRightOnline) || (leg >= 3 && !pcaLeftOnline)) {
      server.send(409, "application/json", "{\"ok\":false,\"error\":\"required PCA9685 offline\"}");
      return;
    }
    enableLegPwm(leg, j);
    server.send(200, "application/json", "{\"ok\":true,\"pwm\":\"on\"}");
  });

  server.on("/legStatus", []() {
    String json = "{\"legs\":[";
    for (int leg = 0; leg < 6; leg++) {
      if (leg) json += ",";
      json += "{\"leg\":" + String(leg);
      json += ",\"pca\":\"" + String((leg < 3) ? "0x40" : "0x41") + "\"";
      json += ",\"channels\":[" + String(legPins[leg][0]) + "," + String(legPins[leg][1]) + "," + String(legPins[leg][2]) + "]";
      json += ",\"dir\":[" + String(jointServoDir[leg][0]) + "," + String(jointServoDir[leg][1]) + "," + String(jointServoDir[leg][2]) + "]";
      json += ",\"enabled\":[" + String(legPwmEnabled[leg][0] ? "true" : "false") + "," + String(legPwmEnabled[leg][1] ? "true" : "false") + "," + String(legPwmEnabled[leg][2] ? "true" : "false") + "]";
      json += ",\"angle\":[" + String(currentPhysicalAngle[leg][0],1) + "," + String(currentPhysicalAngle[leg][1],1) + "," + String(currentPhysicalAngle[leg][2],1) + "]";
      json += ",\"target\":[" + String(manualTargetAngle[leg][0],1) + "," + String(manualTargetAngle[leg][1],1) + "," + String(manualTargetAngle[leg][2],1) + "]";
      json += ",\"home\":[" + String(homeAngles[leg][0],1) + "," + String(homeAngles[leg][1],1) + "," + String(homeAngles[leg][2],1) + "]}";
    }
    json += "]}";
    server.send(200, "application/json", json);
  });

  server.on("/params", []() {
    if (server.hasArg("z")) gaitZ = constrain(server.arg("z").toFloat(), -300.0f, 0.0f);
    if (server.hasArg("len")) stepLen = constrain(server.arg("len").toFloat(), 0.0f, 250.0f);
    if (server.hasArg("lift")) stepLift = constrain(server.arg("lift").toFloat(), 0.0f, 180.0f);
    if (server.hasArg("spd")) baseSpeed = constrain(server.arg("spd").toFloat(), 250.0f, 5000.0f);
    server.send(200, "text/plain", "OK");
  });

  server.on("/bodyIk", []() {
    if (!legOutputsArmed || (spineState != SPINE_STAND && spineState != SPINE_WALK)) {
      server.send(409, "application/json", "{\"ok\":false,\"error\":\"body IK requires STAND/WALK\"}");
      return;
    }
    if (server.hasArg("p")) bodyPitch = constrain(server.arg("p").toFloat(), -30.0f, 30.0f);
    if (server.hasArg("r")) bodyRoll = constrain(server.arg("r").toFloat(), -30.0f, 30.0f);
    if (server.hasArg("y")) bodyYaw = constrain(server.arg("y").toFloat(), -45.0f, 45.0f);
    server.send(200, "text/plain", "OK");
  });

  server.on("/joy", []() {
    if (!server.hasArg("x") || !server.hasArg("y") || !server.hasArg("r")) {
      server.send(400, "application/json", "{\"ok\":false,\"error\":\"x/y/r required\"}");
      return;
    }
    float x = server.arg("x").toFloat();
    float y = server.arg("y").toFloat();
    float r = server.arg("r").toFloat();
    if (!isfinite(x) || !isfinite(y) || !isfinite(r)) {
      server.send(400, "application/json", "{\"ok\":false,\"error\":\"non-finite joystick\"}");
      return;
    }
    const float requestedMagnitude = sqrtf(x*x + y*y + r*r);
    if (requestedMagnitude > 0.02f && (!legOutputsArmed || spineState != SPINE_WALK)) {
      server.send(409, "application/json", "{\"ok\":false,\"error\":\"non-zero joystick requires WALK\"}");
      return;
    }
    joyX = constrain(x, -1.0f, 1.0f);
    joyY = constrain(y, -1.0f, 1.0f);
    joyR = constrain(r, -1.0f, 1.0f);
    markMotionCommand();
    server.send(200, "text/plain", "OK");
  });

  server.on("/mode", []() {
    String m = server.arg("m"); Mode newMode = currentMode;
    if (m == "cal") newMode = CAL;
    else if (m == "idle") newMode = IDLE;
    else if (m == "bezier_joy") newMode = BEZIER_JOY;
    else if (m == "home_rise") {
      server.send(409, "application/json", "{\"ok\":false,\"error\":\"use /goHome?confirm=start\"}");
      return;
    }
    else { server.send(400, "application/json", "{\"ok\":false,\"error\":\"unknown mode\"}"); return; }

    if (newMode == BEZIER_JOY) {
      if (!legOutputsArmed || (spineState != SPINE_STAND && spineState != SPINE_WALK)) {
        server.send(409, "application/json", "{\"ok\":false,\"error\":\"walking requires completed STAND\"}");
        return;
      }
      markMotionCommand();
      linkFailsafeActive = false;
      spineState = SPINE_WALK;
      if (currentMode != BEZIER_JOY) {
        currentPhase = 0.0f;
        gaitCommandWasActive = false;
        s_joyX = s_joyY = s_joyR = 0.0f;
      }
    } else if (newMode == CAL && legOutputsArmed) {
      server.send(409, "application/json", "{\"ok\":false,\"error\":\"disarm all leg PWM before calibration mode\"}");
      return;
    } else if (newMode == IDLE) {
      const bool recoveringFailsafe = linkFailsafeActive || spineState == SPINE_FAILSAFE;
      joyX = joyY = joyR = 0.0f;
      linkFailsafeActive = false;
      spineState = legOutputsArmed ? SPINE_STAND : SPINE_DISARMED;
      if (recoveringFailsafe) {
        safeTransitionActive = true;
        safeTransitionStartMs = millis();
      }
    }

    if (currentMode != newMode) {
      if (currentMode == CAL || newMode == CAL) {
        safeTransitionActive = true;
        safeTransitionStartMs = millis();
      }
      currentMode = newMode;
    }
    server.send(200, "text/plain", "OK");
  });

  server.on("/audio", []() {
    String cmd = server.arg("cmd");
    if (!dfReady) { server.send(503, "text/plain", "DFPlayer yok"); return; }

    if (cmd == "play") {
      // Sadece OYNAT butonu çalmayı başlatır
      dfTrack = constrain(server.arg("t").toInt(), 1, 99);
      dfPlayer.play(dfTrack);
      addLog("[AUDIO] Parca " + String(dfTrack) + " oynatiliyor.\n");
    } else if (cmd == "next" || cmd == "prev") {
      // Parça numarası güncellenir; çalma BAŞLATILMAZ
      dfTrack = constrain(server.arg("t").toInt(), 1, 99);
      addLog("[AUDIO] Parca secildi: " + String(dfTrack) + "\n");
    } else if (cmd == "stop") {
      dfPlayer.stop();
      addLog("[AUDIO] Durduruldu.\n");
    } else if (cmd == "vol") {
      dfVolume = constrain(server.arg("v").toInt(), 0, 30);
      dfPlayer.volume(dfVolume);
      addLog("[AUDIO] Ses: " + String(dfVolume) + "\n");
    }
    server.send(200, "text/plain", "OK");
  });

  server.on("/getHome", []() {
    int leg = server.arg("leg").toInt();
    if (leg < 0 || leg > 5) { server.send(400, "text/plain", "bad leg"); return; }
    String json = "{\"c\":" + String(homeAngles[leg][0],1)
                + ",\"f\":" + String(homeAngles[leg][1],1)
                + ",\"t\":" + String(homeAngles[leg][2],1) + "}";
    server.send(200, "application/json", json);
  });

  server.on("/saveHome", []() {
    int leg = server.arg("leg").toInt();
    if (leg < 0 || leg > 5 || !server.hasArg("c") || !server.hasArg("f") || !server.hasArg("t")) {
      server.send(400, "text/plain", "bad/missing home args"); return;
    }
    float c = server.arg("c").toFloat(), f = server.arg("f").toFloat(), t = server.arg("t").toFloat();
    if (!isfinite(c) || !isfinite(f) || !isfinite(t) || c < MIN_DEGREE || c > MAX_DEGREE || f < MIN_DEGREE || f > MAX_DEGREE || t < MIN_DEGREE || t > MAX_DEGREE) {
      server.send(422, "application/json", "{\"ok\":false,\"error\":\"home angle out of range\"}"); return;
    }
    homeAngles[leg][0] = c; homeAngles[leg][1] = f; homeAngles[leg][2] = t;
    preferences.begin("apex_calib2", false);
    String pfx = String(leg);
    preferences.putFloat((pfx+"hC").c_str(), homeAngles[leg][0]);
    preferences.putFloat((pfx+"hF").c_str(), homeAngles[leg][1]);
    preferences.putFloat((pfx+"hT").c_str(), homeAngles[leg][2]);
    preferences.putBool("homeSet", true);
    preferences.end();
    homeSet = true;
    configValid = validateLegConfiguration();
    addLog("[HOME] Bacak " + String(leg) + " kaydedildi.\n");
    server.send(200, "text/plain", "OK");
  });

  server.on("/homePreview", []() {
    int leg = server.arg("leg").toInt();
    int j   = server.arg("j").toInt();
    float v = server.arg("v").toFloat();
    if (leg < 0 || leg > 5 || j < 0 || j > 2) { server.send(400, "text/plain", "bad leg/joint"); return; }
    setManualTarget(leg, j, v);
    server.send(200, "text/plain", "OK");
  });

  server.on("/goHome", []() {
    if (server.arg("confirm") != "start") {
      server.send(400, "application/json", "{\"ok\":false,\"error\":\"confirm=start required\"}");
      return;
    }
    String reason;
    if (!beginStandupSequence(reason)) {
      String escaped = reason;
      escaped.replace("\\", "\\\\");
      escaped.replace("\"", "\\\"");
      server.send(409, "application/json", "{\"ok\":false,\"error\":\"" + escaped + "\"}");
      return;
    }
    server.send(202, "application/json", "{\"ok\":true,\"state\":\"CAPTURE\"}");
  });


  server.on("/armStatus", []() {
    server.send(200, "application/json", armStatusJson());
  });

  server.on("/armPose", []() {
    armApplyConfigArgs(false);
    bool immediate = (server.arg("immediate") == "1");
    if (server.hasArg("base"))    armSetTarget(ARM_BASE,    server.arg("base").toFloat(),    immediate);
    if (server.hasArg("joint1"))  armSetTarget(ARM_JOINT1,  server.arg("joint1").toFloat(),  immediate);
    if (server.hasArg("joint2"))  armSetTarget(ARM_JOINT2,  server.arg("joint2").toFloat(),  immediate);
    if (server.hasArg("gripper")) armSetTarget(ARM_GRIPPER, server.arg("gripper").toFloat(), immediate);
    addLog("[ARM] armPose: B=" + String(armTargetAngle[ARM_BASE],1) +
           " J1=" + String(armTargetAngle[ARM_JOINT1],1) +
           " J2=" + String(armTargetAngle[ARM_JOINT2],1) +
           " G="  + String(armTargetAngle[ARM_GRIPPER],1) + "\n");
    server.send(200, "application/json", armStatusJson());
  });

  server.on("/armServo", []() {
    armApplyConfigArgs(false);
    String id = server.hasArg("id") ? server.arg("id") : server.arg("joint");
    int joint = armJointIndexFromId(id);
    if (joint < 0) { server.send(400, "text/plain", "Bilinmeyen kol servosu. id=base|joint1|joint2|gripper"); return; }
    if (!server.hasArg("angle")) { server.send(400, "text/plain", "angle eksik"); return; }
    bool immediate = (server.arg("immediate") == "1");
    armSetTarget((uint8_t)joint, server.arg("angle").toFloat(), immediate);
    addLog("[ARM] armServo " + String(armNames[joint]) + " -> " + String(armTargetAngle[joint],1) + "\n");
    server.send(200, "application/json", armStatusJson());
  });

  server.on("/armCalibrate", []() {
    bool immediate = (server.arg("immediate") == "1");
    armSetPoseTargets(90.0f, 90.0f, 90.0f, 90.0f, immediate);
    addLog("[ARM] 90 derece kalibrasyon pozu aktif. Hornlari bu pozda tak.\n");
    server.send(200, "application/json", armStatusJson());
  });

  server.on("/armHome", []() {
    bool immediate = (server.arg("immediate") == "1");
    armSetPoseTargets(armHomePose[0], armHomePose[1], armHomePose[2], armHomePose[3], immediate);
    addLog("[ARM] Bekleme pozu aktif.\n");
    server.send(200, "application/json", armStatusJson());
  });

  server.on("/armOpen", []() {
    armSetTarget(ARM_GRIPPER, armGripperOpenDeg, server.arg("immediate") == "1");
    server.send(200, "application/json", armStatusJson());
  });

  server.on("/armClose", []() {
    armSetTarget(ARM_GRIPPER, armGripperCloseDeg, server.arg("immediate") == "1");
    server.send(200, "application/json", armStatusJson());
  });

  server.on("/armConfig", []() {
    bool persist = (server.arg("save") == "1");
    // Ana eklemler sabit [5,175]. Sadece gripper aralığı ve open/close kullanıcı ayarlı.
    armApplyConfigArgs(persist);
    if (persist) addLog("[ARM] Gripper kaydedildi: 90 acik, MAX/kapali=" + String(armMaxDeg[ARM_GRIPPER],1) + "\n");
    server.send(200, "application/json", armStatusJson());
  });

  server.on("/armOff", []() {
    armDisablePwm();
    addLog("[ARM] Kol PWM kapatildi.\n");
    server.send(200, "application/json", armStatusJson());
  });



  // ARM UYUMLULUK ALIAS'LARI — RPi/UI tarafı slash'li yol kullanırsa da 404 vermesin.
  // Aynı handler mantığı, aynı PCA 0x40 CH12-15. Bacak kanallarına dokunmaz.
  server.on("/arm/status", []() {
    server.send(200, "application/json", armStatusJson());
  });

  server.on("/arm/pose", []() {
    armApplyConfigArgs(false);
    bool immediate = (server.arg("immediate") == "1");
    if (server.hasArg("base"))    armSetTarget(ARM_BASE,    server.arg("base").toFloat(),    immediate);
    if (server.hasArg("joint1"))  armSetTarget(ARM_JOINT1,  server.arg("joint1").toFloat(),  immediate);
    if (server.hasArg("joint2"))  armSetTarget(ARM_JOINT2,  server.arg("joint2").toFloat(),  immediate);
    if (server.hasArg("gripper")) armSetTarget(ARM_GRIPPER, server.arg("gripper").toFloat(), immediate);
    addLog("[ARM] /arm/pose alias: B=" + String(armTargetAngle[ARM_BASE],1) +
           " J1=" + String(armTargetAngle[ARM_JOINT1],1) +
           " J2=" + String(armTargetAngle[ARM_JOINT2],1) +
           " G="  + String(armTargetAngle[ARM_GRIPPER],1) + "\n");
    server.send(200, "application/json", armStatusJson());
  });

  server.on("/arm/servo", []() {
    armApplyConfigArgs(false);
    String id = server.hasArg("id") ? server.arg("id") : server.arg("joint");
    int joint = armJointIndexFromId(id);
    if (joint < 0) { server.send(400, "text/plain", "Bilinmeyen kol servosu. id=base|joint1|joint2|gripper"); return; }
    if (!server.hasArg("angle")) { server.send(400, "text/plain", "angle eksik"); return; }
    bool immediate = (server.arg("immediate") == "1");
    armSetTarget((uint8_t)joint, server.arg("angle").toFloat(), immediate);
    addLog("[ARM] /arm/servo alias " + String(armNames[joint]) + " -> " + String(armTargetAngle[joint],1) + "\n");
    server.send(200, "application/json", armStatusJson());
  });

  server.on("/arm/calibrate", []() {
    bool immediate = (server.arg("immediate") == "1");
    armSetPoseTargets(90.0f, 90.0f, 90.0f, 90.0f, immediate);
    addLog("[ARM] /arm/calibrate alias: 90 derece kalibrasyon pozu aktif.\n");
    server.send(200, "application/json", armStatusJson());
  });

  server.on("/arm/home", []() {
    bool immediate = (server.arg("immediate") == "1");
    armSetPoseTargets(armHomePose[0], armHomePose[1], armHomePose[2], armHomePose[3], immediate);
    addLog("[ARM] /arm/home alias: bekleme pozu aktif.\n");
    server.send(200, "application/json", armStatusJson());
  });

  server.on("/arm/open", []() {
    armSetTarget(ARM_GRIPPER, armGripperOpenDeg, server.arg("immediate") == "1");
    server.send(200, "application/json", armStatusJson());
  });

  server.on("/arm/close", []() {
    armSetTarget(ARM_GRIPPER, armGripperCloseDeg, server.arg("immediate") == "1");
    server.send(200, "application/json", armStatusJson());
  });

  server.on("/arm/config", []() {
    bool persist = (server.arg("save") == "1");
    armApplyConfigArgs(persist);
    if (persist) addLog("[ARM] /arm/config alias: gripper araligi kaydedildi.\n");
    server.send(200, "application/json", armStatusJson());
  });

  server.on("/arm/off", []() {
    armDisablePwm();
    addLog("[ARM] /arm/off alias: kol PWM kapatildi.\n");
    server.send(200, "application/json", armStatusJson());
  });

  server.begin();
  // SAFE BOOT invariant: server başlasa da motor rampası başlamaz. Yalnızca
  // açık operator onaylı /goHome?confirm=start CAPTURE sekansını başlatabilir.
  bootStartMs     = 0;
  bootSoftStart   = false;
  bootMotionBlend = 1.0f;
  updateStatusLed();
  Serial.println("[SAFETY] state=" + String(spineStateName()) +
                 " outputsArmed=" + String(legOutputsArmed ? "true" : "false") +
                 " pcaR=" + String(pcaRightOnline ? "online" : "offline") +
                 " pcaL=" + String(pcaLeftOnline ? "online" : "offline") +
                 " config=" + String(configValid ? "valid" : "invalid"));
}

void loop() {
  server.handleClient();
  updateStatusLed();
  armUpdate();
  recoverLateI2CDevices();
  updateTOF400C();
  updateBatteryMonitor();

  if (currentMode == BEZIER_JOY &&
      (!motionCommandSeen || millis() - lastMotionCommandMs > MOTION_COMMAND_TIMEOUT_MS)) {
    triggerMotionFailsafe();
  }

  while (gpsSerial.available() > 0) {
    if (gps.encode(gpsSerial.read())) {
      if (gps.location.isValid()) {
        currentLat = gps.location.lat();
        currentLng = gps.location.lng();
        if (!gpsHasFix) { gpsHasFix = true; addLog("[GPS] UYDU KILIDI SAGLANDI!\n"); }
      }
    }
  }

  // GPS TANI: kablolama doğru mu, modülden veri geliyor mu — fix şartı olmadan.
  // İçeride (gök görmeyen bir yerde) fix asla gelmez, bu normal; ama charsProcessed()
  // artıyorsa kablolama ve baud rate doğru demektir, fix sadece açık havada gelir.
  static unsigned long lastGpsDiagMs = 0;
  static unsigned long lastGpsChars  = 0;
  if (millis() - lastGpsDiagMs > 3000) {
    lastGpsDiagMs = millis();
    unsigned long chars = gps.charsProcessed();
    if (chars == lastGpsChars) {
      addLog("[GPS] VERI YOK! Kablolama kontrol et (RX=16 TX=17, 9600 baud).\n");
    } else if (!gpsHasFix) {
      addLog("[GPS] Veri aliniyor (" + String(chars) + " byte), fix bekleniyor. Uydu: " + String(gps.satellites.value()) + "\n");
    }
    lastGpsChars = chars;
  }

  unsigned long currentMillis = millis();

  if (bootSoftStart) {
    bootMotionBlend = constrain((currentMillis - bootStartMs) / (float)STAND_RAMP_MS, 0.0f, 1.0f);
    if (bootMotionBlend >= 1.0f) bootSoftStart = false;
  } else {
    bootMotionBlend = 1.0f;
  }

  // MPU varsa referansı bir kez yakala; gimbal çıktısı yine operator açana kadar sıfırdır.
  if (mpuOnline && !bootSoftStart && !gimbalRefCaptured) {
    gimbalRefPitch   = s_mpuPitch;
    gimbalRefRoll    = s_mpuRoll;
    gimbalRefCaptured = true;
    addLog("[GIMBAL] Referans otomatik alindi: P=" + String(gimbalRefPitch,1) + " R=" + String(gimbalRefRoll,1) + "\n");
  }

  if (currentMillis - lastUpdate >= 20) {
    controlFrameDtS = min(currentMillis - lastUpdate, 40UL) / 1000.0f;
    lastUpdate = currentMillis;
    maxDiffThisFrame = 0.0;

    if (safeTransitionActive && currentMode != HOME_RISE && currentMillis - safeTransitionStartMs > 2000) {
      safeTransitionActive = false;
    }

    readMPU6050();

    // GELİŞMİŞ GİMBAL: 4 aşamalı filtre
    // gimbalFilterLevel: 0=hızlı/keskin  100=yavaş/yumuşak
    float t_f = gimbalFilterLevel / 100.0f;
    float baseAlpha    = 0.35f - t_f * 0.25f;    // 0.35→0.10
    float gimbalDeadband = 0.3f + t_f * 1.7f;    // 0.3°→2.0°

    if (!gimbalActive) {
      s_mpuPitch    = mpuPitch;
      s_mpuRoll     = mpuRoll;
      s_gimbalPitch = 0.0f;
      s_gimbalRoll  = 0.0f;
    } else {
      // Aşama 1: IMU low-pass filtresi
      s_mpuPitch += (mpuPitch - s_mpuPitch) * baseAlpha;
      s_mpuRoll  += (mpuRoll  - s_mpuRoll)  * baseAlpha;

      // Aşama 2: Referansa göre geri-besleme hatası.
      // Canonical frame: +X ileri, +Y sol, +Z yukarı.
      // +pitch = burun aşağı, +roll = sol taraf yukarı.
      // Fiziksel testte roll yönü doğrulandı. IMU X ekseni robot önü olduğu için
      // burun yukarı ölçümü negatif pitch üretir; mevcut R^T IK ile istenen
      // ön-Z azalt / arka-Z artır tepkisi pitch için ölçüm-referans işaretidir.
      float pErr = s_mpuPitch - gimbalRefPitch;
      float rErr = gimbalRefRoll - s_mpuRoll;

      // Aşama 3: Deadband — küçük titremelere tepki yok
      float pCorr = (fabsf(pErr) > gimbalDeadband) ? pErr - copysignf(gimbalDeadband, pErr) : 0.0f;
      float rCorr = (fabsf(rErr) > gimbalDeadband) ? rErr - copysignf(gimbalDeadband, rErr) : 0.0f;

      // Aşama 4: Sabit alpha (adaptif kazanç kaldırıldı — büyük hatalarda patlama yapıyordu)
      // ±15° sınırı: aşırı düzeltmeyi ve IK limitine çarpmayı engeller
      s_gimbalPitch = constrain(s_gimbalPitch + (pCorr - s_gimbalPitch) * baseAlpha, -15.0f, 15.0f);
      s_gimbalRoll  = constrain(s_gimbalRoll  + (rCorr - s_gimbalRoll)  * baseAlpha, -15.0f, 15.0f);
    }

    float targetPitch = bodyPitch + s_gimbalPitch;
    float targetRoll  = bodyRoll  + s_gimbalRoll;

    s_gaitZ = approach(s_gaitZ, gaitZ, 2.0);
    s_yaw   = approach(s_yaw,   bodyYaw, 1.0);
    s_len   = approach(s_len, stepLen, 1.5);
    s_lift  = approach(s_lift, stepLift, 1.5);

    s_pitch = approach(s_pitch, targetPitch, 1.5);
    s_roll  = approach(s_roll,  targetRoll,  1.5);

    if (currentMode == CAL) {
      for(int i=0; i<6; i++) {
        send_to_PCA9685(i, 0, 90.0);
        send_to_PCA9685(i, 1, (i < 3) ? 5.0 : 175.0);
        send_to_PCA9685(i, 2, (i < 3) ? 5.0 : 175.0);
      }
    }
    else if (currentMode == MANUAL) {
      if (!manualTargetsInitialized) initManualTargetsFromCurrent();
      for (int i = 0; i < 6; i++)
        for (int j = 0; j < 3; j++)
          send_to_PCA9685(i, j, manualTargetAngle[i][j]);
    }
    else if (currentMode == IDLE) {
      s_joyX = approach(s_joyX, 0.0, 0.12);
      s_joyY = approach(s_joyY, 0.0, 0.12);
      s_joyR = approach(s_joyR, 0.0, 0.12);

      for(int i=0; i<6; i++) {
        calculate_body_ik_and_move(i, gaitX + legNeutralFwdOffset[i], (i < 3) ? -gaitY : gaitY, s_gaitZ);
      }
    }
    else if (currentMode == BEZIER_JOY) {
      float joyMag = sqrt(joyX * joyX + joyY * joyY + joyR * joyR);

      // s_joy değerleri HER ZAMAN gerçek joystick hedefine yaklaşır (sıfıra dönüş dahil).
      // Önceden bu satırlar "joyMag > 0.05" şartına bağlıydı; joystick bırakılınca
      // ham değer 0 olup şart false olduğundan s_joyX/Y/R son haliyle DONUP KALIYORDU
      // → robot bırakınca da yürümeye devam ediyordu. Düzeltme: yumuşatma koşulsuz.
      s_joyX = approach(s_joyX, joyX, 0.12);
      s_joyY = approach(s_joyY, joyY, 0.12);
      s_joyR = approach(s_joyR, joyR, 0.12);

      float s_joyMag = sqrt(s_joyX * s_joyX + s_joyY * s_joyY + s_joyR * s_joyR);

      // Her yeni hareket çift-destek sınırından başlar. Komut bırakıldığında
      // yumuşatılmış genlik sıfıra yaklaşana kadar fazı sürdürmek, havadaki
      // ayağın aynı fazda dikey düşmesini engeller.
      if (joyMag > 0.05f && !gaitCommandWasActive && s_joyMag < 0.03f) currentPhase = 0.0f;
      if (joyMag > 0.05f || s_joyMag > 0.02f) {
        float speedMultiplier = max(s_joyMag, 0.1f);
        float cycleDuration = baseSpeed / speedMultiplier;

        currentPhase += (controlFrameDtS * 1000.0f / cycleDuration);
        if (currentPhase >= 1.0) currentPhase -= 1.0;
      }
      gaitCommandWasActive = joyMag > 0.05f || s_joyMag > 0.02f;

      float vX =  s_joyY * s_len;
      float vY = -s_joyX * s_len;

      // Araba diferansiyeli: sağ/sol taraf farklı BOYUNA adım uzunluğu, yanal bileşen YOK
      // joyR > 0: sağ taraf iç (kısa), sol taraf dış (uzun) → sağa viraj
      // joyR < 0: sağ taraf dış (uzun), sol taraf iç (kısa) → sola viraj
      float diff = s_joyR * s_len;

      for(int i=0; i<6; i++) {
        float t_leg = fmod(currentPhase + legPhase[i], 1.0);

        float sideSign = (i < 3) ? 1.0f : -1.0f;  // +1 sağ taraf, -1 sol taraf
        float leg_vX = vX - sideSign * diff;
        float leg_vY = vY;  // yanal: sadece joyX'ten gelir, dönüş katkısı yok

        float legBaseX = gaitX + legNeutralFwdOffset[i];
        float legX, legY, legZ;
        float baseLegY = (i < 3) ? -gaitY : gaitY;

        if (t_leg < 0.5) {
          float bt = t_leg * 2.0;
          float smooth = bt*bt*bt*(10.0f + bt*(-15.0f + 6.0f*bt));
          float swing_curve = -1.0f + 2.0f * smooth;
          legX = legBaseX + (leg_vX / 2.0) * swing_curve;
          legY = baseLegY + (leg_vY / 2.0) * swing_curve;
          // Small joystick input must not command full-height suspended legs.
          // At zero input the lift settles to zero with the smoothed command.
          float liftBlend = constrain(s_joyMag / 0.35f, 0.0f, 1.0f);
          float liftShape = powf(sinf(bt * PI), 4.0f);
          legZ = s_gaitZ + s_lift * liftBlend * liftShape;
        } else {
          float st = (t_leg - 0.5) * 2.0;
          float smooth = st*st*st*(10.0f + st*(-15.0f + 6.0f*st));
          float stance_curve = 1.0f - 2.0f * smooth;
          legX = legBaseX + (leg_vX / 2.0) * stance_curve;
          legY = baseLegY + (leg_vY / 2.0) * stance_curve;
          legZ = s_gaitZ;
        }
        calculate_body_ik_and_move(i, legX, legY, legZ);
      }
    }
    else if (currentMode == HOME_RISE) {
      s_joyX = approach(s_joyX, 0.0, 0.12);
      s_joyY = approach(s_joyY, 0.0, 0.12);
      s_joyR = approach(s_joyR, 0.0, 0.12);

      if (homeRisePhase == 0) {
        // Faz 0 – CAPTURE: mekanik gövde destekleri üzerindeyken kaydedilmiş,
        // bilinen 18-eklem pozu kısa süre sabit tutulur. Burada IK yoktur.
        for (int leg = 0; leg < 6; leg++)
          for (int joint = 0; joint < 3; joint++)
            send_to_PCA9685(leg, joint, homeAngles[leg][joint]);

        if (currentMillis - homeRisePhaseMs >= CAPTURE_HOLD_MS) {
          homeRisePhase = 1;
          homeRisePhaseMs = currentMillis;
          bootStartMs = currentMillis;
          bootSoftStart = true;
          bootMotionBlend = 0.0f;
          safeTransitionActive = true;
          safeTransitionStartMs = currentMillis;
          spineState = SPINE_RISING;
          addLog("[SAFETY] CAPTURE tamam; yumusak STAND rampasi basladi.\n");
        }
      }
      else if (homeRisePhase == 1) {
        // Faz 1 – RISING: altı bacağı birlikte, sınırlı adım/süzme ile nominal
        // STAND IK hedefine götür. Eski otomatik tripod kaldırmaları kaldırıldı.
        for (int i = 0; i < 6; i++)
          calculate_body_ik_and_move(i, gaitX + legNeutralFwdOffset[i],
                                     (i < 3) ? -gaitY : gaitY, s_gaitZ);

        if (!bootSoftStart && maxDiffThisFrame < 1.0f) {
          homeRisePhase   = 0;
          homeRisePhaseMs = 0;
          currentMode     = IDLE;
          spineState      = SPINE_STAND;
          safeTransitionActive = false;
          addLog("[SAFETY] Kontrollu kalkis tamamlandi; STAND aktif.\n");
        }
      }
    }

    if (safeTransitionActive && maxDiffThisFrame < 1.0) {
      safeTransitionActive = false;
    }
    firstRun = false;
  }
}
