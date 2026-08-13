#!/usr/bin/env python3
"""
SwarmDashboard Synthetic Camera Streamer
Generates a synthetic HUD frame per drone showing real telemetry
and streams it as H.264 RTP UDP to QGC VideoReceiver.
Drone 1 -> port 5600
Drone 2 -> port 5601  
Drone 3 -> port 5602
"""

import sys
import time
import threading
import math

from gz.transport13 import Node
from gz.msgs10.pose_v_pb2 import Pose_V

import gi
gi.require_version('Gst', '1.0')
from gi.repository import Gst, GLib

try:
    import numpy as np
    from PIL import Image, ImageDraw, ImageFont
    HAS_PIL = True
except ImportError:
    HAS_PIL = False
    print("Installing Pillow and numpy...")
    import subprocess
    subprocess.run([sys.executable, "-m", "pip", "install", "Pillow", "numpy", "--break-system-packages"])
    import numpy as np
    from PIL import Image, ImageDraw, ImageFont

Gst.init(None)

WIDTH  = 640
HEIGHT = 480
FPS    = 15

DRONES = [
    {"id": 1, "model": "x500_0", "port": 5600, "color": (0, 200, 100)},
    {"id": 2, "model": "x500_1", "port": 5601, "color": (0, 150, 255)},
    {"id": 3, "model": "x500_2", "port": 5602, "color": (255, 180, 0)},
]

HOST = "172.27.80.1"

# Shared telemetry state
telemetry = {d["id"]: {"x": 0.0, "y": 0.0, "z": 0.0, "frame": 0} for d in DRONES}

pipelines = {}
appsrcs   = {}

def create_pipeline(drone_id, port):
    pipeline_str = (
        f"appsrc name=src_{drone_id} is-live=true block=false format=time "
        f"caps=video/x-raw,format=RGB,width={WIDTH},height={HEIGHT},framerate={FPS}/1 ! "
        f"videoconvert ! "
        f"x264enc tune=zerolatency bitrate=800 speed-preset=ultrafast key-int-max=15 ! "
        f"rtph264pay config-interval=1 pt=96 ! "
        f"udpsink host={HOST} port={port} sync=false"
    )
    pipeline = Gst.parse_launch(pipeline_str)
    appsrc   = pipeline.get_by_name(f"src_{drone_id}")
    pipeline.set_state(Gst.State.PLAYING)
    print(f"[Drone {drone_id}] Pipeline -> udp://{HOST}:{port}")
    return pipeline, appsrc

def draw_frame(drone_id, color):
    t   = telemetry[drone_id]
    img = Image.new("RGB", (WIDTH, HEIGHT), (10, 10, 20))
    d   = ImageDraw.Draw(img)

    # Sky gradient
    for y in range(HEIGHT // 2):
        ratio = y / (HEIGHT // 2)
        r = int(10 + ratio * 20)
        g = int(10 + ratio * 30)
        b = int(20 + ratio * 60)
        d.line([(0, y), (WIDTH, y)], fill=(r, g, b))

    # Ground
    for y in range(HEIGHT // 2, HEIGHT):
        ratio = (y - HEIGHT // 2) / (HEIGHT // 2)
        r = int(30 + ratio * 20)
        g = int(20 + ratio * 10)
        b = int(10)
        d.line([(0, y), (WIDTH, y)], fill=(r, g, b))

    # Horizon line
    d.line([(0, HEIGHT // 2), (WIDTH, HEIGHT // 2)], fill=(100, 100, 100), width=1)

    # Grid lines on ground
    for i in range(0, WIDTH, 80):
        d.line([(i, HEIGHT // 2), (WIDTH // 2, HEIGHT)], fill=(50, 50, 50), width=1)
    for j in range(HEIGHT // 2, HEIGHT, 40):
        d.line([(0, j), (WIDTH, j)], fill=(40, 40, 40), width=1)

    # Crosshair
    cx, cy = WIDTH // 2, HEIGHT // 2
    d.line([(cx - 30, cy), (cx - 8, cy)], fill=color, width=2)
    d.line([(cx + 8,  cy), (cx + 30, cy)], fill=color, width=2)
    d.line([(cx, cy - 30), (cx, cy - 8)], fill=color, width=2)
    d.line([(cx, cy + 8),  (cx, cy + 30)], fill=color, width=2)
    d.ellipse([(cx-4, cy-4), (cx+4, cy+4)], outline=color, width=1)

    # Top bar
    d.rectangle([(0, 0), (WIDTH, 32)], fill=(0, 0, 0, 180))
    d.text((8, 8),  f"DRONE {drone_id}", fill=color)
    d.text((WIDTH // 2 - 40, 8), f"SWARM DASHBOARD", fill=(200, 200, 200))
    d.text((WIDTH - 100, 8), f"FPS {FPS}", fill=(150, 150, 150))

    # Bottom HUD bar
    d.rectangle([(0, HEIGHT - 60), (WIDTH, HEIGHT)], fill=(0, 0, 0))

    # Altitude
    alt = abs(t["z"])
    d.text((10, HEIGHT - 55), "ALT", fill=(150, 150, 150))
    d.text((10, HEIGHT - 36), f"{alt:.1f} m", fill=color)

    # Position
    d.text((WIDTH // 2 - 60, HEIGHT - 55), "POSITION", fill=(150, 150, 150))
    d.text((WIDTH // 2 - 70, HEIGHT - 36), f"X:{t['x']:.1f} Y:{t['y']:.1f}", fill=(200, 200, 200))

    # Frame counter
    d.text((WIDTH - 80, HEIGHT - 55), "FRAME", fill=(150, 150, 150))
    d.text((WIDTH - 80, HEIGHT - 36), f"{t['frame']}", fill=(150, 150, 150))

    # Animated pulse on drone ID
    pulse = int(127 + 127 * math.sin(time.time() * 3))
    d.ellipse([(8, HEIGHT - 90), (18, HEIGHT - 80)], fill=(0, pulse, 0))

    # Border
    d.rectangle([(0, 0), (WIDTH-1, HEIGHT-1)], outline=color, width=2)

    return np.array(img).tobytes()

def stream_drone(drone_id, color):
    appsrc = appsrcs[drone_id]
    frame_duration_ns = int(1e9 / FPS)
    pts = 0
    while True:
        try:
            raw = draw_frame(drone_id, color)
            buf = Gst.Buffer.new_wrapped(raw)
            buf.pts      = pts
            buf.duration = frame_duration_ns
            appsrc.emit("push-buffer", buf)
            pts += frame_duration_ns
            telemetry[drone_id]["frame"] += 1
            if telemetry[drone_id]["frame"] % (FPS * 5) == 1:
                print(f"[Drone {drone_id}] Streaming frame {telemetry[drone_id]['frame']}")
            time.sleep(1.0 / FPS)
        except Exception as e:
            print(f"[Drone {drone_id}] Stream error: {e}")
            time.sleep(0.1)

def setup_telemetry(node):
    """Subscribe to Gazebo pose topic to get real drone positions."""
    def cb(msg):
        for pose in msg.pose:
            for d in DRONES:
                if d["model"] in pose.name:
                    tid = d["id"]
                    telemetry[tid]["x"] = pose.position.x
                    telemetry[tid]["y"] = pose.position.y
                    telemetry[tid]["z"] = pose.position.z
    node.subscribe(Pose_V, "/world/default/pose/info", cb)
    print("Telemetry subscribed: /world/default/pose/info")

def main():
    print("SwarmDashboard Synthetic Camera Streamer")
    print("=" * 45)
    print(f"Resolution: {WIDTH}x{HEIGHT} @ {FPS} fps")
    print(f"Codec: H.264 (x264enc zerolatency)")
    print()

    # Create GStreamer pipelines
    for d in DRONES:
        pipeline, appsrc = create_pipeline(d["id"], d["port"])
        pipelines[d["id"]] = pipeline
        appsrcs[d["id"]]   = appsrc

    # Subscribe to real pose data
    node = Node()
    setup_telemetry(node)

    # Start streaming threads
    for d in DRONES:
        t = threading.Thread(
            target=stream_drone,
            args=(d["id"], d["color"]),
            daemon=True
        )
        t.start()
        print(f"[Drone {d['id']}] Streaming thread started")

    print()
    print("QGC Video Settings:")
    print("  Settings > Video > Source: UDP h.264 Video Stream")
    print("  UDP URL: 0.0.0.0:5600  (for Drone 1)")
    print("  Change port to 5601 or 5602 for other drones")
    print()
    print("Streaming... Press Ctrl+C to stop.")

    loop = GLib.MainLoop()
    try:
        loop.run()
    except KeyboardInterrupt:
        print("\nStopping all pipelines...")
        for p in pipelines.values():
            p.set_state(Gst.State.NULL)
        sys.exit(0)

if __name__ == "__main__":
    main()
