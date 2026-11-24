#!/usr/bin/env bash
set -u

echo "APEX_PI_POWER_AUDIT_V1"
date --iso-8601=seconds
tr -d '\0' </proc/device-tree/model 2>/dev/null || true
echo

if command -v vcgencmd >/dev/null 2>&1; then
  vcgencmd get_throttled || true
  vcgencmd measure_temp || true
  vcgencmd pmic_read_adc EXT5V_V 2>/dev/null || true
  vcgencmd pmic_read_adc EXT5V_I 2>/dev/null || true
else
  echo "vcgencmd=missing"
fi

echo "-- kernel power messages --"
dmesg --color=never 2>/dev/null | grep -Ei 'under.?voltage|voltage normal|throttl|power supply' | tail -n 30 || true

echo "-- boot current policy --"
if command -v rpi-eeprom-config >/dev/null 2>&1; then
  rpi-eeprom-config 2>/dev/null | grep -E '^(PSU_MAX_CURRENT|USB_MAX_CURRENT_ENABLE)=' || true
fi
grep -E '^(usb_max_current_enable|PSU_MAX_CURRENT)=' /boot/firmware/config.txt /boot/config.txt 2>/dev/null || true

echo "-- USB topology --"
lsusb -t 2>/dev/null || true

echo "-- power-supply sysfs --"
for supply in /sys/class/power_supply/*; do
  [ -e "$supply" ] || continue
  echo "[$supply]"
  for item in type online status voltage_now current_now power_now usb_type; do
    [ -r "$supply/$item" ] && printf '%s=' "$item" && cat "$supply/$item"
  done
done
