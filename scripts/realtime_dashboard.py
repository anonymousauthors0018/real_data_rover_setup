#!/usr/bin/env python3
"""
Real-time Sync Viewer — MJPEG HTTP stream
Subscribes to /synced/pair, /camera/color, /camera/depth, renders the sync panel,
and serves it as MJPEG at http://<jetson-ip>:8080

Usage:
  python3 gui_view_realtime.py
  Open browser to http://<jetson-ip>:8080
"""

import os
import sys
import json
import threading
import time
# from http.server import BaseHTTPRequestHandler, HTTPServer
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from collections import deque

import cv2
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.backends.backend_agg import FigureCanvasAgg

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge

# =========================
# CONFIG
# =========================
MANIFEST_PATH = "/workspace/radar_info.json"

RGB_TOPIC = "/camera/color/image_raw"
DEPTH_TOPIC = "/camera/depth/image_rect_raw"
SYNCED_TOPIC = "/synced/pair"

HTTP_PORT = 8080
DISPLAY_HEIGHT = 480

# Radar params
NR, RX, TX, CHIRPS = 256, 4, 2, 16
TOTAL_CHIRPS = CHIRPS * TX
FS, SLOPE, C, FC = 5209e3, 70e12, 3e8, 77e9
RPB = C * FS / (2 * SLOPE * NR)
DC_SKIP = 5

ND = 128
PRF = FS / NR
DOP_AXIS = (np.arange(ND) - ND / 2) * PRF / ND * C / (2 * FC)
ANGLE_BINS = 64
RANGE_MAX_M = NR // 2 * RPB

R_TARGET = 1.0
R_MARGIN = 0.3
CENTER_BIN = int(round(R_TARGET / RPB))
MARGIN_BINS = int(np.ceil(R_MARGIN / RPB))
BIN_RANGE = slice(max(DC_SKIP, CENTER_BIN - MARGIN_BINS),
                  min(NR, CENTER_BIN + MARGIN_BINS))

DOPPLER_HISTORY = 50

# =========================
# SHARED FRAME BUFFER
# =========================
class FrameBuffer:
    def __init__(self):
        self.lock = threading.Lock()
        self.jpeg = None

    def set(self, bgr):
        ok, buf = cv2.imencode('.jpg', bgr, [cv2.IMWRITE_JPEG_QUALITY, 80])
        if ok:
            with self.lock:
                self.jpeg = buf.tobytes()

    def get(self):
        with self.lock:
            return self.jpeg

FRAME_BUFFER = FrameBuffer()

# =========================
# RADAR DECODE + VIS
# =========================
def decode_radar_frame(raw_path, manifest):
    chirps = int(manifest["chirps"])
    tx = int(manifest["tx"])
    rx = int(manifest["rx"])
    samples = int(manifest["samples"])
    raw = np.fromfile(raw_path, dtype=np.int16)
    if raw.size == 0:
        return None
    expected = chirps * tx * rx * (samples // 2) * 2 * 2
    if raw.size < expected:
        raw = np.concatenate([raw, np.zeros(expected - raw.size, dtype=np.int16)])
    try:
        adc = raw[:expected].reshape(1, chirps, tx, rx, samples // 2, 2, 2)
        adc = np.transpose(adc, (0, 1, 2, 3, 4, 6, 5))
        adc = adc.reshape(1, chirps, tx, rx, samples, 2)
        cplx = (1j * adc[..., 0] + adc[..., 1]).astype(np.complex64)
        return cplx[0].reshape(chirps * tx, rx, samples).transpose(2, 1, 0)
    except ValueError:
        return None

def compute_ra_map(frame):
    range_fft = np.fft.fft(frame, axis=0)
    range_fft -= np.mean(range_fft, axis=2, keepdims=True)
    virtual = np.concatenate([range_fft[:, :, 0::2], range_fft[:, :, 1::2]], axis=1)
    az = np.fft.fftshift(np.fft.fft(virtual, n=ANGLE_BINS, axis=1), axes=1)
    return np.mean(np.abs(az), axis=2)

def compute_range_profile(frame):
    rp = np.mean(np.abs(np.fft.fft(frame, axis=0)), axis=(1, 2))
    rp[:DC_SKIP] = 0
    return rp

def compute_micro_doppler(frame):
    iq = frame[:, 0, 0::2]
    range_fft = np.fft.fft(iq, axis=0)
    sig = np.sum(range_fft[BIN_RANGE, :], axis=0)
    return np.abs(np.fft.fftshift(np.fft.fft(sig, n=ND)))

def fig_to_bgr(fig):
    canvas = FigureCanvasAgg(fig)
    canvas.draw()
    arr = np.asarray(canvas.buffer_rgba())
    bgr = cv2.cvtColor(arr[:, :, :3], cv2.COLOR_RGB2BGR)
    plt.close(fig)
    return bgr

def render_radar_panel(frame, doppler_history, elapsed_sec,
                       offset_rgb_radar, offset_rgb_depth, seq, panel_height=480):
    rp = compute_range_profile(frame)
    rp_half = rp[DC_SKIP:NR // 2]

    peak_bin = int(np.argmax(rp[DC_SKIP:NR // 2]) + DC_SKIP)
    peak_range = peak_bin * RPB

    H = panel_height
    W = int(H * 1.3)
    TITLE_H = 35
    MARGIN_L = 55
    MARGIN_R = 140
    MARGIN_B = 35
    MARGIN_T = TITLE_H + 10

    plot_w = W - MARGIN_L - MARGIN_R
    plot_h = H - MARGIN_T - MARGIN_B

    canvas = np.full((H, W, 3), 20, dtype=np.uint8)

    cv2.rectangle(canvas, (0, 0), (W, TITLE_H), (40, 40, 55), -1)
    cv2.putText(canvas, "RANGE FFT", (12, 24),
                cv2.FONT_HERSHEY_DUPLEX, 0.65, (220, 220, 240), 1, cv2.LINE_AA)
    cv2.putText(canvas, f"seq {seq}", (W - MARGIN_R - 90, 24),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (140, 220, 255), 1, cv2.LINE_AA)

    # Plot area background
    cv2.rectangle(canvas, (MARGIN_L, MARGIN_T),
                  (MARGIN_L + plot_w, MARGIN_T + plot_h),
                  (30, 30, 40), -1)
    cv2.rectangle(canvas, (MARGIN_L - 1, MARGIN_T - 1),
                  (MARGIN_L + plot_w, MARGIN_T + plot_h),
                  (100, 100, 120), 1)

    # Normalize profile for plotting
    rp_db = 20 * np.log10(rp_half + 1e-9)
    rp_db = rp_db - np.min(rp_db)
    if np.max(rp_db) > 0:
        rp_db = rp_db / np.max(rp_db)

    n_bins = len(rp_half)
    pts = []
    for i, val in enumerate(rp_db):
        x = MARGIN_L + int(i * (plot_w - 1) / max(1, n_bins - 1))
        y = MARGIN_T + plot_h - int(val * (plot_h - 1))
        pts.append((x, y))

    # Grid lines for range
    y_ticks = [0, 1, 2, 3, 4, 5]
    for r in y_ticks:
        if r > RANGE_MAX_M:
            continue
        x = MARGIN_L + int((r / RANGE_MAX_M) * plot_w)
        cv2.line(canvas, (x, MARGIN_T + plot_h), (x, MARGIN_T + plot_h + 5),
                 (180, 180, 200), 1)
        cv2.line(canvas, (x, MARGIN_T), (x, MARGIN_T + plot_h),
                 (60, 60, 80), 1, cv2.LINE_AA)
        cv2.putText(canvas, f"{r}", (x - 8, MARGIN_T + plot_h + 20),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.4, (200, 200, 220), 1, cv2.LINE_AA)

    # Horizontal amplitude guides
    for frac in [0.25, 0.5, 0.75]:
        y = MARGIN_T + plot_h - int(frac * plot_h)
        cv2.line(canvas, (MARGIN_L, y), (MARGIN_L + plot_w, y),
                 (60, 60, 80), 1, cv2.LINE_AA)

    # Draw the FFT curve
    for i in range(1, len(pts)):
        cv2.line(canvas, pts[i - 1], pts[i], (0, 255, 255), 2, cv2.LINE_AA)

    # Peak marker
    peak_x = MARGIN_L + int(((peak_bin - DC_SKIP) / max(1, n_bins - 1)) * (plot_w - 1))
    cv2.line(canvas, (peak_x, MARGIN_T), (peak_x, MARGIN_T + plot_h),
             (0, 0, 255), 1, cv2.LINE_AA)

    cv2.putText(canvas, "Range (m)",
                (MARGIN_L + plot_w // 2 - 30, H - 6),
                cv2.FONT_HERSHEY_SIMPLEX, 0.42, (180, 180, 200), 1, cv2.LINE_AA)

    cv2.putText(canvas, "Amplitude",
                (8, MARGIN_T + plot_h // 2),
                cv2.FONT_HERSHEY_SIMPLEX, 0.42, (180, 180, 200), 1, cv2.LINE_AA)

    sx = MARGIN_L + plot_w + 15
    sy = MARGIN_T + 5

    cv2.rectangle(canvas, (sx - 8, sy - 5),
                  (W - 8, MARGIN_T + plot_h),
                  (35, 35, 50), -1)
    cv2.rectangle(canvas, (sx - 8, sy - 5),
                  (W - 8, MARGIN_T + plot_h),
                  (80, 80, 100), 1)

    def put(text, y_off, color=(220, 220, 240), size=0.42, bold=False):
        thick = 2 if bold else 1
        cv2.putText(canvas, text, (sx, sy + y_off),
                    cv2.FONT_HERSHEY_SIMPLEX, size, color, thick, cv2.LINE_AA)

    put("TIME", 15, (140, 140, 170), 0.38)
    put(f"{elapsed_sec:.1f} s", 35, (220, 220, 240), 0.52, bold=True)

    put("PEAK RANGE", 65, (140, 140, 170), 0.38)
    put(f"{peak_range:.2f} m", 85, (80, 255, 120), 0.52, bold=True)

    put("RGB-RADAR", 115, (140, 140, 170), 0.38)
    color = (80, 255, 120) if offset_rgb_radar < 20 else (80, 220, 255) if offset_rgb_radar < 50 else (80, 120, 255)
    put(f"{offset_rgb_radar:.1f} ms", 135, color, 0.48, bold=True)

    put("RGB-DEPTH", 165, (140, 140, 170), 0.38)
    color = (80, 255, 120) if offset_rgb_depth < 20 else (80, 220, 255) if offset_rgb_depth < 50 else (80, 120, 255)
    put(f"{offset_rgb_depth:.1f} ms", 185, color, 0.48, bold=True)

    cv2.circle(canvas, (sx + 8, sy + 220), 5, (80, 255, 120), -1)
    put("LIVE", 225, (80, 255, 120), 0.42, bold=True)

    return canvas

def process_depth_image(msg, bridge):
    d = bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
    d = np.nan_to_num(d, nan=0.0, posinf=0.0, neginf=0.0)
    n = cv2.normalize(d, None, 0, 255, cv2.NORM_MINMAX, dtype=cv2.CV_8U)
    return cv2.applyColorMap(n, cv2.COLORMAP_TURBO)

def resize_to_height(img, h):
    ih, iw = img.shape[:2]
    return cv2.resize(img, (int(h * (iw / ih)), h))

# =========================
# ROS2 NODE
# =========================
class SyncViewerNode(Node):
    def __init__(self, manifest):
        super().__init__('sync_viewer')
        self.manifest = manifest
        self.bridge = CvBridge()
        self.doppler_history = deque(maxlen=DOPPLER_HISTORY)
        self.first_radar_ns = None

        # Buffers keyed by timestamp_ns
        self.rgb_buffer = {}   # ns -> Image msg
        self.depth_buffer = {} # ns -> Image msg
        self.buffer_limit = 60 # keep last ~6 seconds

        self.create_subscription(Image, RGB_TOPIC, self.rgb_cb, 10)
        self.create_subscription(Image, DEPTH_TOPIC, self.depth_cb, 10)
        self.create_subscription(String, SYNCED_TOPIC, self.sync_cb, 10)

        self.get_logger().info(f"Subscribed to {RGB_TOPIC}, {DEPTH_TOPIC}, {SYNCED_TOPIC}")

    def _stamp_ns(self, msg):
        return int(msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec)

    def _prune(self, buf):
        if len(buf) > self.buffer_limit:
            for k in sorted(buf.keys())[:-self.buffer_limit]:
                del buf[k]

    def rgb_cb(self, msg):
        self.rgb_buffer[self._stamp_ns(msg)] = msg
        self._prune(self.rgb_buffer)

    def depth_cb(self, msg):
        self.depth_buffer[self._stamp_ns(msg)] = msg
        self._prune(self.depth_buffer)

    def _find_closest(self, buf, target_ns, tol=int(50e6)):
        if target_ns in buf:
            return buf[target_ns]
        if not buf:
            return None
        best_key = min(buf.keys(), key=lambda k: abs(k - target_ns))
        if abs(best_key - target_ns) <= tol:
            return buf[best_key]
        return None

    def sync_cb(self, msg):
        try:
            pair = json.loads(msg.data)
        except Exception as e:
            self.get_logger().warn(f"Bad sync msg: {e}")
            return

        rgb_ns = pair["rgb_stamp_ns"]
        depth_ns = pair["depth_stamp_ns"]
        radar_ns = pair["radar_stamp_ns"]
        bin_path = pair["radar_raw_bin"]

        if self.first_radar_ns is None:
            self.first_radar_ns = radar_ns

        rgb_msg = self._find_closest(self.rgb_buffer, rgb_ns)
        depth_msg = self._find_closest(self.depth_buffer, depth_ns)

        if rgb_msg is None or depth_msg is None:
            return
        if not os.path.exists(bin_path):
            return

        elapsed = (radar_ns - self.first_radar_ns) / 1e9
        off_rgb_radar = abs(rgb_ns - radar_ns) / 1e6
        off_rgb_depth = abs(rgb_ns - depth_ns) / 1e6

        try:
            rgb = self.bridge.imgmsg_to_cv2(rgb_msg, desired_encoding="bgr8")
            rgb_disp = resize_to_height(rgb, DISPLAY_HEIGHT)

            depth_disp = resize_to_height(process_depth_image(depth_msg, self.bridge), DISPLAY_HEIGHT)


            radar_frame = decode_radar_frame(bin_path, self.manifest)
            if radar_frame is not None:
                radar_panel = render_radar_panel(
                    radar_frame, self.doppler_history, elapsed,
                    off_rgb_radar, off_rgb_depth, pair.get('radar_seq', 0),
                    panel_height=DISPLAY_HEIGHT)
            else:
                radar_panel = np.zeros((DISPLAY_HEIGHT, DISPLAY_HEIGHT, 3), dtype=np.uint8)

            cv2.putText(rgb_disp, f"RGB t={elapsed:.1f}s", (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            cv2.putText(depth_disp, "Depth", (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

            canvas = np.hstack((rgb_disp, depth_disp, radar_panel))
            FRAME_BUFFER.set(canvas)
        except Exception as e:
            self.get_logger().warn(f"Render error: {e}")

# =========================
# HTTP MJPEG SERVER
# =========================
class MJPEGHandler(BaseHTTPRequestHandler):
    def log_message(self, format, *args):
        pass  # suppress default logging

    def do_GET(self):
        if self.path == '/':
            self.send_response(200)
            self.send_header('Content-Type', 'text/html')
            self.end_headers()
            
            # Modern, Clean Dashboard UI
            html = b"""
            <!DOCTYPE html>
            <html lang="en">
            <head>
                <meta charset="UTF-8">
                <meta name="viewport" content="width=device-width, initial-scale=1.0">
                <title>Radar - L515 Data Collection Sync</title>
                <style>
                    :root {
                        --primary: #2563eb;
                        --bg-light: #f8fafc;
                        --card-bg: #ffffff;
                        --text-main: #1e293b;
                        --text-dim: #64748b;
                        --accent: #10b981;
                    }
                    body {
                        background-color: var(--bg-light);
                        color: var(--text-main);
                        font-family: 'Segoe UI', system-ui, -apple-system, sans-serif;
                        margin: 0;
                        padding: 0;
                        display: flex;
                        flex-direction: column;
                        align-items: center;
                    }
                    header {
                        width: 100%;
                        background: var(--card-bg);
                        padding: 1.5rem 0;
                        box-shadow: 0 1px 3px rgba(0,0,0,0.1);
                        margin-bottom: 2rem;
                        display: flex;
                        justify-content: center;
                    }
                    .header-content {
                        width: 90%;
                        max-width: 1200px;
                        display: flex;
                        justify-content: space-between;
                        align-items: center;
                    }
                    h1 { 
                        margin: 0; 
                        font-size: 1.25rem; 
                        font-weight: 700;
                        letter-spacing: -0.025em;
                        color: var(--primary);
                    }
                    .status-badge {
                        display: flex;
                        align-items: center;
                        background: #ecfdf5;
                        color: var(--accent);
                        padding: 0.4rem 0.8rem;
                        border-radius: 99px;
                        font-size: 0.85rem;
                        font-weight: 600;
                    }
                    .dot {
                        height: 8px;
                        width: 8px;
                        background-color: var(--accent);
                        border-radius: 50%;
                        display: inline-block;
                        margin-right: 8px;
                        animation: pulse 2s infinite;
                    }
                    @keyframes pulse {
                        0% { transform: scale(0.95); opacity: 0.7; }
                        50% { transform: scale(1.1); opacity: 1; }
                        100% { transform: scale(0.95); opacity: 0.7; }
                    }
                    .container {
                        width: 95%;
                        max-width: 1400px;
                        display: flex;
                        flex-direction: column;
                        align-items: center;
                    }
                    .viewer-card {
                        background: var(--card-bg);
                        padding: 1rem;
                        border-radius: 12px;
                        box-shadow: 0 10px 25px -5px rgba(0, 0, 0, 0.05);
                        border: 1px solid #e2e8f0;
                        overflow: hidden;
                    }
                    img {
                        max-width: 100%;
                        display: block;
                        border-radius: 4px;
                    }
                    .footer-info {
                        margin-top: 1.5rem;
                        font-size: 0.85rem;
                        color: var(--text-dim);
                        text-align: center;
                    }
                </style>
            </head>
            <body>
                <header>
                    <div class="header-content">
                        <h1>Jetson Radar - L515<span style="font-weight:300; color: #94a3b8;">| Realtime Sync Stream</span></h1>
                        <div class="status-badge">
                            <span class="dot"></span> LIVE SYSTEM
                        </div>
                    </div>
                </header>
                <div class="container">
                    <div class="viewer-card">
                        <img src="/stream.mjpg" alt="Real-time Stream" />
                    </div>
                    <div class="footer-info">
                        <!-- <p>Subscribed to: <code>/synced/pair</code> &bull; Resolution: 480p (Scaled) &bull; Framerate: ~10 Hz</p> -->
                    </div>
                </div>
            </body>
            </html>
            """
            self.wfile.write(html)
        elif self.path == '/stream.mjpg':
            self.send_response(200)
            self.send_header('Content-Type', 'multipart/x-mixed-replace; boundary=--jpgboundary')
            self.end_headers()
            try:
                while True:
                    jpeg = FRAME_BUFFER.get()
                    if jpeg is None:
                        time.sleep(0.05)
                        continue
                    self.wfile.write(b'--jpgboundary\r\n')
                    self.send_header('Content-Type', 'image/jpeg')
                    self.send_header('Content-Length', str(len(jpeg)))
                    self.end_headers()
                    self.wfile.write(jpeg)
                    self.wfile.write(b'\r\n')
                    time.sleep(0.1)
            except (BrokenPipeError, ConnectionResetError):
                pass
        else:
            self.send_response(404)
            self.end_headers()

class ThreadedHTTPServer(ThreadingHTTPServer):
    """Handle requests in a separate thread."""
    daemon_threads = True

def start_http_server(port):
    server = ThreadedHTTPServer(('0.0.0.0', port), MJPEGHandler)
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    print(f"\n  MJPEG server: http://0.0.0.0:{port}")
    print(f"  Open http://<jetson-ip>:{port} in a browser\n")
    return server

# =========================
# MAIN
# =========================
def main():
    with open(MANIFEST_PATH, "r") as f:
        manifest = json.load(f)

    start_http_server(HTTP_PORT)

    rclpy.init()
    node = SyncViewerNode(manifest)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()