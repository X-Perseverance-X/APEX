#!/usr/bin/env python3
"""Small status UI rendered directly to the Pi's RGB565 SPI framebuffer.

This avoids the Xorg shadow framebuffer on the installed ILI9486 screen.
Sensor acquisition runs in a separate process; this only reads HTTP telemetry.
"""

from __future__ import annotations

import argparse
import json
import math
import mmap
import signal
import time
from pathlib import Path
from urllib.error import URLError
from urllib.request import urlopen

import numpy as np
from PIL import Image, ImageDraw, ImageFont


WIDTH, HEIGHT = 480, 320
BG = (8, 18, 29)
PANEL = (19, 37, 54)
TEXT = (231, 243, 247)
MUTED = (136, 182, 198)
GREEN = (101, 225, 174)
ORANGE = (255, 174, 107)
GRID = (49, 78, 96)


def font(size: int, bold: bool = False) -> ImageFont.ImageFont:
    name = "DejaVuSans-Bold.ttf" if bold else "DejaVuSans.ttf"
    path = Path("/usr/share/fonts/truetype/dejavu") / name
    try:
        return ImageFont.truetype(str(path), size)
    except OSError:
        return ImageFont.load_default()


FONT_10 = font(10)
FONT_11 = font(11)
FONT_12 = font(12)
FONT_12B = font(12, True)
FONT_14B = font(14, True)


def fmt(value: object, digits: int = 1) -> str:
    return f"{value:.{digits}f}" if isinstance(value, (int, float)) and math.isfinite(value) else "--"


def draw_scan(draw: ImageDraw.ImageDraw, state: dict) -> None:
    left, top, right, bottom = 5, 52, 237, 238
    draw.rounded_rectangle((left, top, right, bottom), radius=5, fill=PANEL, outline=GRID)
    draw.text((13, 59), "LIDAR TOP VIEW", font=FONT_11, fill=MUTED)
    cx, cy, radius = 121, 143, 74
    draw.ellipse((cx-radius, cy-radius, cx+radius, cy+radius), outline=GRID)
    draw.line((cx-radius, cy, cx+radius, cy), fill=GRID)
    draw.line((cx, cy-radius, cx, cy+radius), fill=GRID)
    lidar = state.get("lidar", {})
    points = lidar.get("scan") or []
    maximum = max((float(point[1]) for point in points), default=3.0)
    scale = radius / max(3.0, maximum)
    for angle, distance in points[::2]:
        x = cx + math.cos(angle) * distance * scale
        y = cy - math.sin(angle) * distance * scale
        draw.point((round(x), round(y)), fill=GREEN)
    draw.ellipse((cx-3, cy-3, cx+3, cy+3), fill=ORANGE)
    draw.text((13, 218), f"{lidar.get('points', 0)} pt  {fmt(lidar.get('scan_hz'))} Hz",
              font=FONT_11, fill=TEXT)


def draw_map(draw: ImageDraw.ImageDraw, state: dict) -> None:
    left, top, right, bottom = 243, 52, 475, 238
    draw.rounded_rectangle((left, top, right, bottom), radius=5, fill=PANEL, outline=GRID)
    draw.text((251, 59), "2D OCCUPANCY", font=FONT_11, fill=MUTED)
    cx, cy, scale = 359, 143, 17
    draw.line((251, cy, 467, cy), fill=GRID)
    draw.line((cx, 72, cx, 209), fill=GRID)
    slam = state.get("slam", {})
    cells = slam.get("map_cells") or []
    resolution = slam.get("resolution_m", .05)
    for gx, gy, probability in cells[::max(1, len(cells) // 2000)]:
        x, y = round(cx + gx * resolution * scale), round(cy - gy * resolution * scale)
        if 251 <= x <= 467 and 72 <= y <= 209:
            draw.rectangle((x, y, x+1, y+1), fill=ORANGE if probability > .58 else MUTED)
    path = [(cx+x*scale, cy-y*scale) for x, y in slam.get("trajectory", [])]
    if len(path) > 1:
        draw.line(path, fill=(255, 224, 121), width=2)
    x, y = cx + slam.get("x_m", 0)*scale, cy - slam.get("y_m", 0)*scale
    draw.ellipse((x-3, y-3, x+3, y+3), fill=(255, 224, 121))
    draw.text((251, 218), f"x {slam.get('x_m', 0):+.2f}  y {slam.get('y_m', 0):+.2f} m",
              font=FONT_11, fill=TEXT)


def render(state: dict | None, connected: bool) -> Image.Image:
    image = Image.new("RGB", (WIDTH, HEIGHT), BG)
    draw = ImageDraw.Draw(image)
    draw.text((8, 5), "SLAM LAB V0", font=FONT_14B, fill=GREEN)
    ip = (state or {}).get("system", {}).get("ip") or "--"
    draw.text((362, 6), f"PI {ip}", font=FONT_10, fill=MUTED)
    sensors = (state or {}).get("sensors", {})
    health = (f"LIDAR {sensors.get('lidar', '--')}   "
              f"IMU {sensors.get('imu', '--')}") if connected else "Dashboard server disconnected"
    draw.text((8, 29), health, font=FONT_12, fill=GREEN if connected and sensors.get("lidar") == "STREAMING" else ORANGE)
    state = state or {}
    draw_scan(draw, state)
    draw_map(draw, state)
    imu = state.get("imu", {})
    draw.text((8, 246),
              f"ROLL {fmt(imu.get('roll_deg'))}   PITCH {fmt(imu.get('pitch_deg'))}   "
              f"YAW REL {fmt(imu.get('yaw_relative_deg'))}",
              font=FONT_12, fill=TEXT)
    status = state.get("slam", {}).get("mapping_status", "PAUSED: NO DATA")
    if len(status) > 41:
        status = status[:40] + "..."
    draw.text((8, 272), f"MAPPING {status}", font=FONT_12B,
              fill=GREEN if status == "ACTIVE" else ORANGE)
    draw.text((8, 299), "ORIENTATION PREVIEW / NOT FULL 3D SLAM", font=FONT_10, fill=MUTED)
    return image


def rgb565(image: Image.Image) -> bytes:
    rgb = np.asarray(image, dtype=np.uint16)
    packed = ((rgb[:, :, 0] >> 3) << 11) | ((rgb[:, :, 1] >> 2) << 5) | (rgb[:, :, 2] >> 3)
    return packed.astype("<u2", copy=False).tobytes()


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--framebuffer", default="/dev/fb1")
    parser.add_argument("--url", default="http://127.0.0.1:8765/api/local_state")
    parser.add_argument("--preview", type=Path, help="Save a PNG and exit without touching framebuffer")
    args = parser.parse_args()
    state = None
    try:
        with urlopen(args.url, timeout=0.5) as response:
            state = json.load(response)
    except (URLError, TimeoutError, ValueError, OSError):
        pass
    if args.preview:
        render(state, state is not None).save(args.preview)
        return

    running = True

    def stop(_signum: int, _frame: object) -> None:
        nonlocal running
        running = False

    signal.signal(signal.SIGTERM, stop)
    signal.signal(signal.SIGINT, stop)
    with open(args.framebuffer, "r+b", buffering=0) as device:
        with mmap.mmap(device.fileno(), WIDTH * HEIGHT * 2, access=mmap.ACCESS_WRITE) as frame:
            original = frame[:]
            try:
                while running:
                    start = time.monotonic()
                    connected = False
                    try:
                        with urlopen(args.url, timeout=0.5) as response:
                            state = json.load(response)
                        connected = True
                    except (URLError, TimeoutError, ValueError, OSError):
                        pass
                    frame.seek(0)
                    frame.write(rgb565(render(state, connected)))
                    time.sleep(max(0.0, 0.2 - (time.monotonic() - start)))
            finally:
                frame.seek(0)
                frame.write(original)


if __name__ == "__main__":
    main()
