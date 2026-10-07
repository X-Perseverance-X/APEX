#!/usr/bin/env python3
"""480x320 X11 dashboard. Polling happens outside the Tk render thread."""

from __future__ import annotations

import json
import math
import threading
import time
import tkinter as tk
from urllib.error import URLError
from urllib.request import urlopen


BG = "#08121d"
PANEL = "#132536"
TEXT = "#e7f3f7"
MUTED = "#88b6c6"
GREEN = "#65e1ae"
ORANGE = "#ffae6b"


class LocalStatus:
    def __init__(self, url: str) -> None:
        self.url = url
        self.latest: dict | None = None
        self.connected = False
        self.lock = threading.Lock()
        self.stop = threading.Event()

        self.window = tk.Tk()
        self.window.title("SLAM LAB LOCAL")
        self.window.geometry("480x320+0+0")
        # The installed pcmanfm desktop sometimes stacks above managed kiosk
        # windows on this SPI framebuffer. An override-redirect window is
        # mapped directly at the screen origin and kept in front.
        self.window.overrideredirect(True)
        self.window.attributes("-topmost", True)
        self.window.configure(bg=BG)
        self.window.bind("<Escape>", lambda _event: self.close())
        self.window.after(1000, self.window.lift)

        self.header = tk.Label(self.window, text="SLAM LAB V0", bg=BG, fg=GREEN,
                               font=("DejaVu Sans", 10, "bold"), anchor="w")
        self.header.place(x=8, y=2, width=220, height=22)
        self.ip = tk.Label(self.window, text="PI --", bg=BG, fg=MUTED,
                           font=("DejaVu Sans", 9), anchor="e")
        self.ip.place(x=245, y=2, width=228, height=22)
        self.health = tk.Label(self.window, text="Connecting...", bg=BG, fg=ORANGE,
                               font=("DejaVu Sans", 9), anchor="w")
        self.health.place(x=8, y=25, width=460, height=20)

        self.scan = tk.Canvas(self.window, width=228, height=185, bg=PANEL,
                              highlightthickness=1, highlightbackground="#39566a")
        self.scan.place(x=5, y=48)
        self.grid = tk.Canvas(self.window, width=228, height=185, bg=PANEL,
                              highlightthickness=1, highlightbackground="#39566a")
        self.grid.place(x=245, y=48)
        self.attitude = tk.Label(self.window, text="ROLL --   PITCH --   YAW REL --",
                                 bg=BG, fg=TEXT, font=("DejaVu Sans", 9), anchor="w")
        self.attitude.place(x=8, y=238, width=464, height=24)
        self.mapping = tk.Label(self.window, text="MAPPING PAUSED", bg=BG, fg=ORANGE,
                                font=("DejaVu Sans", 9, "bold"), anchor="w")
        self.mapping.place(x=8, y=261, width=464, height=24)
        self.foot = tk.Label(self.window, text="ORIENTATION PREVIEW / NOT FULL 3D SLAM",
                             bg=BG, fg=MUTED, font=("DejaVu Sans", 7), anchor="w")
        self.foot.place(x=8, y=289, width=464, height=20)

        threading.Thread(target=self._fetch_loop, name="local-ui-http", daemon=True).start()
        self.window.after(200, self._render)

    def _fetch_loop(self) -> None:
        while not self.stop.is_set():
            try:
                with urlopen(self.url, timeout=0.5) as response:
                    state = json.load(response)
                with self.lock:
                    self.latest = state
                    self.connected = True
            except (URLError, TimeoutError, ValueError, OSError):
                with self.lock:
                    self.connected = False
            self.stop.wait(0.2)  # 5 Hz, independent of sensor rates

    @staticmethod
    def _fmt(value: float | None, digits: int = 1) -> str:
        return "--" if value is None else f"{value:.{digits}f}"

    def _draw_scan(self, state: dict) -> None:
        c = self.scan
        c.delete("all")
        c.create_text(7, 8, text="LIDAR TOP VIEW", fill=MUTED, anchor="nw",
                      font=("DejaVu Sans", 8, "bold"))
        cx, cy, radius = 114, 102, 77
        c.create_oval(cx-radius, cy-radius, cx+radius, cy+radius, outline="#34566b")
        c.create_line(cx-radius, cy, cx+radius, cy, fill="#29475b")
        c.create_line(cx, cy-radius, cx, cy+radius, fill="#29475b")
        points = state["lidar"]["scan"]
        max_range = max((point[1] for point in points), default=3.0)
        scale = radius / max(3.0, max_range)
        for angle, distance in points[::2]:
            x = cx + math.cos(angle) * distance * scale
            y = cy - math.sin(angle) * distance * scale
            c.create_rectangle(x, y, x+1, y+1, outline=GREEN, fill=GREEN)
        c.create_oval(cx-3, cy-3, cx+3, cy+3, fill=ORANGE, outline=ORANGE)
        c.create_text(7, 176, text=f"{state['lidar']['points']} pt  {state['lidar']['scan_hz']:.1f} Hz",
                      fill=TEXT, anchor="sw", font=("DejaVu Sans", 8))

    def _draw_map(self, state: dict) -> None:
        c = self.grid
        c.delete("all")
        c.create_text(7, 8, text="2D OCCUPANCY", fill=MUTED, anchor="nw",
                      font=("DejaVu Sans", 8, "bold"))
        cx, cy, scale = 114, 102, 17
        c.create_line(0, cy, 228, cy, fill="#29475b")
        c.create_line(cx, 23, cx, 175, fill="#29475b")
        slam = state["slam"]
        resolution = slam["resolution_m"]
        cells = slam["map_cells"]
        for cell_x, cell_y, probability in cells[::max(1, len(cells)//2000)]:
            x, y = cx + cell_x*resolution*scale, cy - cell_y*resolution*scale
            if 0 <= x < 228 and 23 <= y < 175:
                color = ORANGE if probability > .58 else "#436b79"
                c.create_rectangle(x, y, x+2, y+2, fill=color, outline=color)
        trajectory = slam["trajectory"]
        if len(trajectory) > 1:
            flat = []
            for x_m, y_m in trajectory:
                flat.extend((cx+x_m*scale, cy-y_m*scale))
            c.create_line(*flat, fill="#ffe079", width=2)
        x = cx + slam["x_m"]*scale
        y = cy - slam["y_m"]*scale
        c.create_oval(x-3, y-3, x+3, y+3, fill="#ffe079", outline="#ffe079")
        c.create_text(7, 176,
                      text=f"x {slam['x_m']:+.2f}  y {slam['y_m']:+.2f} m",
                      fill=TEXT, anchor="sw", font=("DejaVu Sans", 8))

    def _render(self) -> None:
        with self.lock:
            state, connected = self.latest, self.connected
        if state:
            self.ip.configure(text=f"IP {state['system']['ip'] or '--'}")
            sensor = state["sensors"]
            self.health.configure(
                text=f"LIDAR {sensor['lidar']}   IMU {sensor['imu']}   "
                     f"CPU {self._fmt(state['system']['cpu_load'], 2)}",
                fg=GREEN if sensor["lidar"] == "STREAMING" else ORANGE,
            )
            imu = state["imu"]
            self.attitude.configure(text=(
                f"ROLL {self._fmt(imu['roll_deg'])}°   "
                f"PITCH {self._fmt(imu['pitch_deg'])}°   "
                f"YAW REL {self._fmt(imu['yaw_relative_deg'])}°"
            ))
            status = state["slam"]["mapping_status"]
            self.mapping.configure(text=f"MAPPING {status}",
                                   fg=GREEN if status == "ACTIVE" else ORANGE)
            self._draw_scan(state)
            self._draw_map(state)
        if not connected:
            self.health.configure(text="Dashboard server disconnected", fg=ORANGE)
        if not self.stop.is_set():
            self.window.after(200, self._render)

    def close(self) -> None:
        self.stop.set()
        self.window.destroy()

    def run(self) -> None:
        self.window.mainloop()


if __name__ == "__main__":
    LocalStatus("http://127.0.0.1:8765/api/state").run()
