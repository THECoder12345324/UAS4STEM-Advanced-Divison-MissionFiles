import cv2
import subprocess
import numpy as np
import zxingcpp
from pymavlink import mavutil

master = mavutil.mavlink_connection("/dev/serial0", baud=57600)
RTSP_URL = "rtsp://root:12345@192.168.1.10/stream=0"

# FFMPEG configuration pushing an mjpeg stream to stdout
FFMPEG_CMD = [
    "ffmpeg",
    "-rtsp_transport", "tcp",
    "-fflags",         "nobuffer",
    "-flags",          "low_delay",
    "-probesize",      "32",
    "-analyzeduration", "0",
    "-i",              RTSP_URL,
    "-f",              "mjpeg",
    "-loglevel",       "warning",
    "-"
]

# Spawn the process once
cap = subprocess.Popen(FFMPEG_CMD, stdout=subprocess.PIPE, stderr=subprocess.DEVNULL)

bytes_buffer = b""
seen = set()

print("Scanning for QR codes... Press Ctrl+C in terminal or 'q' on the video window to stop.")

try:
    while True:
        chunk = cap.stdout.read(4096)
        if not chunk:
            print("Stream disconnected.")
            break
            
        bytes_buffer += chunk
        a = bytes_buffer.find(b"\xff\xd8")
        b = bytes_buffer.find(b"\xff\xd9")

        if a != -1 and b != -1 and b > a:
            jpg = bytes_buffer[a:b+2]
            bytes_buffer = bytes_buffer[b+2:]

            frame = cv2.imdecode(np.frombuffer(jpg, dtype=np.uint8), cv2.IMREAD_COLOR)

            if frame is not None:
                # --- NEW: Display the live preview window ---
                cv2.imshow("QR Scanner Preview", frame)
                
                # --- NEW: Break the loop if 'q' is pressed on the keyboard ---
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break

                # QR code processing logic
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
    print("\nStopped via Terminal.")
finally:
    # Clean up subprocess and close OpenCV GUI windows
    cap.terminate()
    cap.wait()
    cv2.destroyAllWindows()
    print("Resources released.")