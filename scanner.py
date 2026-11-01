import cv2
import subprocess
import numpy as np
import zxingcpp
from pymavlink import mavutil

master = mavutil.mavlink_connection("/dev/serial0", baud=57600)
#master.wait_heartbeat()

#cap = subprocess.Popen(
#    ["rpicam-vid", "--codec", "mjpeg", "--width", "1280", "--height", "960",
#     "--framerate", "30", "--timeout", "0", "--autofocus-mode", "continuous",
#     "--lens-position", "0", "--output", "-"],
#    stdout=subprocess.PIPE, stderr=subprocess.DEVNULL
#)

RTSP_URL = "rtsp://root:12345@192.168.1.10/stream=0"

FFPLAY_CMD = [
    "ffplay",
    "-fflags",         "nobuffer",
    "-flags",          "low_delay",
    "-probesize",      "32",
    "-analyzeduration", "0",
    "-framedrop",
    "-rtsp_transport", "tcp",
    "-window_title",   "WiFiLink Stream",
    "-loglevel",       "warning",
    RTSP_URL,
]



bytes_buffer = b""
seen = set()

print("Scanning for QR codes... Press Ctrl+C to stop.")

try:
    while True:
        cap = subprocess.run(FFPLAY_CMD)
        bytes_buffer += cap.stdout.read(4096)
        a = bytes_buffer.find(b"\xff\xd8")
        b = bytes_buffer.find(b"\xff\xd9")

        if a != -1 and b != -1 and b > a:
            jpg = bytes_buffer[a:b+2]
            bytes_buffer = bytes_buffer[b+2:]

            frame = cv2.imdecode(np.frombuffer(jpg, dtype=np.uint8), cv2.IMREAD_COLOR)

            if frame is not None:
                results = zxingcpp.read_barcodes(frame)
                for result in results:
                    if result.text and result.text not in seen:
                        seen.add(result.text)
                        end_part = result.text.rstrip("/").rsplit("/", 1)[-1]
                        master.mav.statustext_send(
                            mavutil.mavlink.MAV_SEVERITY_INFO,
                            end_part.encode("utf-8")[:50]
                        )
                        print(f"[{result.format}] {end_part}")
except KeyboardInterrupt:
    print("\nStopped.")
finally:
    cap.terminate()