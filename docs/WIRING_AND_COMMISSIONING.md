# META‑1 v54 — Wiring & Commissioning

## 1. Canonical coordinate frame

- +X: robot front
- +Y: robot left
- +Z: up

Leg order in firmware:

| Index | Leg | Side | PCA |
|---|---|---|---|
| 0 | RF | Right Front | 0x40 |
| 1 | RM | Right Middle | 0x40 |
| 2 | RR | Right Rear | 0x40 |
| 3 | LF | Left Front | 0x41 |
| 4 | LM | Left Middle | 0x41 |
| 5 | LR | Left Rear | 0x41 |

Tripod groups:

- Group A: RF + RR + LM
- Group B: RM + LF + LR

## 2. I2C bus

| Device | Address | SDA | SCL |
|---|---:|---:|---:|
| PCA9685 Right | 0x40 | GPIO8 | GPIO9 |
| PCA9685 Left | 0x41 | GPIO8 | GPIO9 |
| MPU6050 | 0x68 | GPIO8 | GPIO9 |
| TOF400C / VL53L1X | 0x29 | GPIO8 | GPIO9 |

All I2C grounds must be common with ESP32-S3 ground.

PCA9685 channel map:

| PCA | Leg | Coxa | Femur | Tibia |
|---|---|---:|---:|---:|
| 0x40 right | RF | 0 | 1 | 2 |
| 0x40 right | RM | 4 | 5 | 6 |
| 0x40 right | RR | 8 | 10 | 9 |
| 0x41 left | LF | 0 | 1 | 2 |
| 0x41 left | LM | 4 | 5 | 6 |
| 0x41 left | LR | 8 | 9 | 10 |

Mechanical order when viewed from above, robot front pointing toward `+X`:

```text
                 FRONT / +X
          LF (3)             RF (0)
          LM (4)             RM (1)
          LR (5)             RR (2)
                 REAR / -X
```

Arm channels are on PCA 0x40: base 12, joint1 13, joint2 14, gripper 15.

PCA logic VCC and servo V+ are different rails. Servo V+ is the separate 6 V
high-current rail; never feed servo power through the ESP or Pi. Grounds of
the servo supplies, PCA boards, ESP, and sensors must meet at common ground.

## 2.1 USB spine transport

The current Pi connection is an FT232 adapter (`0403:6001`) exposed with the
stable device name below. Do not hard-code `ttyUSB0`; the number may change
when the LiDAR is connected.

`/dev/serial/by-id/usb-FTDI_FT232R_USB_UART_A5069RR4-if00-port0`

- Baud: 115200, 8N1
- `@IMU1`: 50 Hz IMU measurement packet
- `@TEL1`: 10 Hz health/sensor packet (firmware v57.11 and newer)
- `@CMD1`: acknowledged, allow-listed peripheral command (firmware v57.12 and newer)
- ESP Wi-Fi is disabled. Telemetry plus laser/DFPlayer commands use this cable.
- The bridge deliberately rejects motion/PWM writes; it cannot arm or move a servo.
- Local diagnostics: `http://127.0.0.1:8091/diagnostics`

For a generic ESP32-S3 UART0/FT232 connection, cross TX and RX: FT232 TX to
ESP RX0 (normally GPIO44), FT232 RX to ESP TX0 (normally GPIO43), and connect
ground. If ESP has its own supply, do not also connect the FT232 5 V pin unless
the exact board power topology explicitly supports it.

## 2.2 UART peripherals

| Peripheral | Peripheral pin | ESP32-S3 pin | Baud | Note |
|---|---|---:|---:|---|
| GPS | TX | GPIO16 / RX2 | 9600 | TX and RX cross |
| GPS | RX | GPIO17 / TX2 | 9600 | Optional if GPS is transmit-only |
| DFPlayer Mini | RX | GPIO4 / TX1 | 9600 | Put 1 kΩ in series near DFPlayer RX |
| DFPlayer Mini | TX | GPIO5 / RX1 | 9600 | 3.3 V UART input |

GPS module supply voltage must be taken from its exact board marking or
datasheet; bare GNSS modules and regulator-equipped breakout boards differ.
DFPlayer uses 5 V and common ground. Connect a speaker only between SPK1 and
SPK2, never between a speaker output and system ground.

## 2.3 External 5 V laser

GPIO14 is reserved by firmware v57.13 for the laser PWM control input. PCA9685 channel 14 is a
different device and does not conflict with ESP GPIO14. Proposed topology:

- Laser VCC -> regulated 5 V
- Laser GND -> system common GND
- Laser S -> GPIO14 only after the module's S-input voltage/current and
  active-high/active-low behavior are verified

GPIO8 must never be used for the laser because it is the shared I2C SDA line.
A
3.3 V GPIO must not directly power the laser diode. If S is not a documented
3.3 V-compatible logic input, use a logic-level MOSFET/transistor stage with a
gate/base resistor and a 10 kΩ default-off resistor. Firmware defaults the
output OFF at boot; the Command Center controls 0–100% PWM and provides an
explicit OFF control.

## 2.4 GPIO18 fan / status-light PWM

GPIO18 drives only the gate/input of an external driver; it does not power a
fan or LED directly. Firmware v57.24 starts this output at 0% and slews every
manual or automatic target gradually, so a 0→100% request does not create the
previous full-power inrush on the supply. A directly connected LED must have a
series resistor and must never draw more current than an ESP GPIO can supply.

## 3. Battery ADC

| Rail | ESP32-S3 input |
|---|---|
| Right 3S battery divider output | GPIO1 / ADC1 |
| Left 3S battery divider output | GPIO2 / ADC1 |

Firmware defaults:

```cpp
#define BAT_RIGHT_ADC_PIN 1
#define BAT_LEFT_ADC_PIN  2
#define BAT_RIGHT_DIVIDER_RATIO 4.20f
#define BAT_LEFT_DIVIDER_RATIO  4.20f
#define BAT_RIGHT_CAL_FACTOR 1.000f
#define BAT_LEFT_CAL_FACTOR  1.000f
```

The default ratio assumes 12.6 V at battery side produces 3.0 V at ADC side. Replace the ratio with the actual resistor ratio:

`divider_ratio = (Rtop + Rbottom) / Rbottom`

Commissioning:

1. Do not connect servos during first ADC test.
2. Measure the battery with a reliable multimeter.
3. Read `batR`, `batL`, `batRAdcMv`, `batLAdcMv` from `/telemetry`.
4. Correct the ratio first.
5. Use `BAT_*_CAL_FACTOR` only for the remaining small systematic error.
6. Add a 100 nF ceramic capacitor from ADC pin to GND close to the ESP32 if the reading is noisy.

## 4. Leg mount pose

The new Servo Lab mount pose is intentionally separate from normal STAND/HOME.

| Side | Coxa | Femur | Tibia |
|---|---:|---:|---:|
| Right RF/RM/RR | 90° | 5° | 5° |
| Left LF/LM/LR | 90° | 175° | 175° |

The software safety range is 5–175°.

## 5. Safe first test sequence

1. Power ESP32 and PCA logic first, servo power disconnected.
2. Open Command Center.
3. Check I2C diagnostics: 0x40, 0x41, 0x68, 0x29.
4. Verify battery ADC readings against multimeter.
5. Connect one servo only.
6. In LEG SERVO LAB select exact leg + joint.
7. Test 90° first for coxa; use 5°/175° mount presets for F/T according to side.
8. Verify physical direction with small commands, not full sweep.
9. Confirm RR channel order physically: firmware preserves RR `{8,10,9}`.
10. Confirm tibia direction physically before attaching full load.
11. Use PWM OFF before mechanically changing horns.
12. Only after all 18 channels are verified, use HOME/STAND.

## 6. Two-monitor workspace

```bash
./apex meta-1 start
```

Default layout:

- Primary Windows screen: Command Center, port 8080
- First non-primary screen: Kinematics Lab, port 8090

The PowerShell launcher uses Windows screen bounds and opens two separate Edge/Chrome windows. If no supported browser path is found, it falls back to the Windows default browser without guaranteed positioning.
