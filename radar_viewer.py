#!/usr/bin/env python3
"""
ESP32 Radar Scene Modeler
-------------------------
Connects to ESP32 WebSocket server, receives LiDAR + temperature data,
and renders a real-time polar radar chart for scene modeling.

Usage:
    pip install websocket-client matplotlib numpy
    python radar_viewer.py [--ip 192.168.1.100] [--port 81]

Data format (JSON from ESP32):
    {
        "ts": <int64_t microseconds>,
        "temp": <float celsius>,
        "vectors": [
            {"a": <angle_deg>, "d": <distance_mm>},
            ...
        ]
    }
"""

import argparse
import json
import queue
import sys
import threading
import time

import numpy as np
import matplotlib
import matplotlib.pyplot as plt

matplotlib.use("TkAgg")

try:
    import websocket
except ImportError:
    print("Missing websocket-client. Install: pip install websocket-client")
    sys.exit(1)


DANGER_RANGE_MM = 1000.0
SAFE_RANGE_MM   = 6000.0

COLOR_DANGER  = "#ff4444"
COLOR_SAFE    = "#ffaa00"
COLOR_OK      = "#44ff44"
BG_COLOR      = "#0a0a14"
GRID_COLOR    = "#222244"
TEXT_COLOR    = "#cccccc"


class RadarViewer:
    def __init__(self, ip: str, port: int):
        self.ip = ip
        self.port = port
        self.url = f"ws://{ip}:{port}/"

        self.data_queue = queue.Queue(maxsize=16)
        self.running = True
        self.ws = None

        self.latest_vectors = []
        self.latest_temp = float("nan")
        self.latest_ts = 0

        self._init_plot()

    # ── matplotlib init ──────────────────────────────────────────────

    def _init_plot(self):
        self.fig = plt.figure(figsize=(8, 8), facecolor=BG_COLOR)
        self.ax = self.fig.add_subplot(111, projection="polar",
                                        facecolor=BG_COLOR)

        self.ax.set_theta_zero_location("N")
        self.ax.set_theta_direction(-1)

        self.ax.set_ylim(0, SAFE_RANGE_MM * 1.15)
        self.ax.set_yticks([1000, 2000, 3000, 4000, 5000, 6000])
        self.ax.set_yticklabels(
            ["1m", "2m", "3m", "4m", "5m", "6m"],
            color=TEXT_COLOR, fontsize=7,
        )

        self.ax.set_xticks(np.radians(np.arange(0, 360, 30)))
        self.ax.set_xticklabels(
            ["0°", "30°", "60°", "90°", "120°", "150°",
             "180°", "210°", "240°", "270°", "300°", "330°"],
            color=TEXT_COLOR, fontsize=7,
        )

        self.ax.grid(color=GRID_COLOR, alpha=0.6, linewidth=0.5)
        self.ax.spines["polar"].set_color(GRID_COLOR)
        self.ax.spines["polar"].set_linewidth(1.0)

        # ── zone fills ──
        theta = np.linspace(0, 2 * np.pi, 200)

        self.ax.fill_between(theta, 0, DANGER_RANGE_MM,
                             color=COLOR_DANGER, alpha=0.06)
        self.ax.fill_between(theta, DANGER_RANGE_MM, SAFE_RANGE_MM,
                             color=COLOR_OK, alpha=0.03)

        self.ax.plot(theta, np.full_like(theta, DANGER_RANGE_MM),
                     color=COLOR_DANGER, alpha=0.35,
                     linewidth=1.2, linestyle="--")

        # ── scatter (colored by distance, near=red  far=green) ──
        self.scatter = self.ax.scatter(
            [], [], s=24, c=[], cmap="RdYlGn_r",
            vmin=0, vmax=SAFE_RANGE_MM,
            edgecolors="none", zorder=5,
        )

        # ── title ──
        self.ax.set_title(
            f"ESP32 Radar  |  ws://{self.ip}:{self.port}",
            color=TEXT_COLOR, fontsize=11, pad=18,
        )

        # ── temperature overlay ──
        self.temp_text = self.ax.text(
            0.02, 0.02, "TEMP: --\u00b0C",
            transform=self.ax.transAxes, color="#ff8844",
            fontsize=14, fontweight="bold", fontfamily="monospace",
            bbox=dict(boxstyle="round,pad=0.3", facecolor="#111122",
                      edgecolor="#333355", alpha=0.9),
        )

        # ── stats ──
        self.stats_text = self.ax.text(
            0.02, 0.92, "",
            transform=self.ax.transAxes, color=TEXT_COLOR,
            fontsize=8, fontfamily="monospace",
        )

        self.fig.canvas.mpl_connect("close_event", self._on_close)

    # ── WebSocket thread ─────────────────────────────────────────────

    def _ws_on_open(self, ws):
        print(f"[WS] Connected to {self.url}")

    def _ws_on_message(self, ws, message):
        try:
            self.data_queue.put_nowait(message)
        except queue.Full:
            pass

    def _ws_on_error(self, ws, error):
        print(f"[WS] Error: {error}")

    def _ws_on_close(self, ws, close_status_code, close_msg):
        print(f"[WS] Disconnected: {close_status_code} {close_msg}")
        self.running = False

    def _ws_thread(self):
        while self.running:
            try:
                self.ws = websocket.WebSocketApp(
                    self.url,
                    on_open=self._ws_on_open,
                    on_message=self._ws_on_message,
                    on_error=self._ws_on_error,
                    on_close=self._ws_on_close,
                )
                self.ws.run_forever(reconnect=3)
            except Exception as e:
                print(f"[WS] Connection error: {e}")
                time.sleep(3)

    # ── data parsing ─────────────────────────────────────────────────

    def _parse_data(self, raw: str):
        try:
            obj = json.loads(raw)
        except json.JSONDecodeError:
            return

        self.latest_ts = obj.get("ts", 0)
        self.latest_temp = obj.get("temp", float("nan"))

        vectors = obj.get("vectors", [])
        self.latest_vectors = [(v["a"], v["d"]) for v in vectors]

    # ── plot update ──────────────────────────────────────────────────

    def _update_plot(self):
        if not self.latest_vectors:
            return

        a_rad = []
        d_mm = []

        for a_deg, d in self.latest_vectors:
            if np.isnan(d) or d <= 0:
                continue
            a_rad.append(np.radians(a_deg))
            d_mm.append(d)

        if not a_rad:
            self.scatter.set_offsets(np.empty((0, 2)))
            return

        a_rad = np.array(a_rad)
        d_mm = np.array(d_mm)

        mask = d_mm <= SAFE_RANGE_MM
        a_rad = a_rad[mask]
        d_mm = d_mm[mask]

        if len(a_rad) == 0:
            self.scatter.set_offsets(np.empty((0, 2)))
            return

        pts = np.column_stack((a_rad, d_mm))
        self.scatter.set_offsets(pts)
        self.scatter.set_array(d_mm)

        temp_str = (f"{self.latest_temp:.1f}\u00b0C"
                    if not np.isnan(self.latest_temp) else "--\u00b0C")
        self.temp_text.set_text(f"TEMP: {temp_str}")

        danger = int(np.sum(d_mm <= DANGER_RANGE_MM))
        safe = int(np.sum((d_mm > DANGER_RANGE_MM) & (d_mm <= SAFE_RANGE_MM)))
        self.stats_text.set_text(
            f"Points : {len(a_rad)} / {len(self.latest_vectors)}\n"
            f"Danger : {danger}  (<1m)\n"
            f"Safe   : {safe}  (1-6m)",
        )

    # ── main loop ────────────────────────────────────────────────────

    def _on_close(self, event):
        self.running = False

    def run(self):
        ws_thread = threading.Thread(target=self._ws_thread, daemon=True)
        ws_thread.start()

        print(f"Connecting to {self.url} ...")
        plt.ion()
        plt.show()

        while self.running:
            try:
                raw = self.data_queue.get(timeout=0.1)
                self._parse_data(raw)
                self._update_plot()
                self.fig.canvas.draw_idle()
                self.fig.canvas.flush_events()
            except queue.Empty:
                self.fig.canvas.flush_events()
                continue

        plt.ioff()
        if self.ws:
            self.ws.close()
        print("Exited.")


def main():
    parser = argparse.ArgumentParser(
        description="ESP32 Radar Scene Modeler")
    parser.add_argument("--ip", default="192.168.1.100",
                        help="ESP32 IP address (default: 192.168.1.100)")
    parser.add_argument("--port", type=int, default=81,
                        help="WebSocket port (default: 81)")
    args = parser.parse_args()

    viewer = RadarViewer(args.ip, args.port)
    viewer.run()


if __name__ == "__main__":
    main()
