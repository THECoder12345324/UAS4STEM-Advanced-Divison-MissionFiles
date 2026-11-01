import cv2
import subprocess
import numpy as np
import zxingcpp
import threading
import time
from pymavlink import mavutil

class PID:
    def __init__(self, kp, ki, kd, max):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.max = max
        self.integral =0
        self.last_error = 0
        self.last_time = time.time()

    def update(self, error):
        current_time = time.time()
        dt = current_time - self.last_time
        if dt <= 0:
            dt = 0.03
        self.integral += error * dt
        derivative = (error - self.last_error) / dt
        output = self.kp * error + self.ki * self.integral + self.kd * derivative
        output = max(min(output, self.max), -self.max)
        self.last_error = error
        self.last_time = current_time
        return output


CENTER_X = 1280//2
CENTER_Y = 720//2

pid_x = PID(.001, .0001, .00005, 1)
pid_y = PID(.001, .0001, .00005, 1)

def send_vel(vx, vy):
    master.mav.set_position_target_local_ned_send(
        0,
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_FRAME_BODY_NED,
        0b0000111111000111,
        0, 0, 0,
        vx, vy, 0,
        0, 0, 0,
        0, 0
    )

def get_coords(verbose=False):
    MSG_ID = mavutil.mavlink.MAVLINK_MSG_ID_GPS_RAW_INT
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
        0,
        MSG_ID,
        250000,
        0, 0, 0, 0, 0
    )
    master.mav.heartbeat_send(
        mavutil.mavlink.MAV_TYPE_GCS,
        mavutil.mavlink.MAV_AUTOPILOT_INVALID,
        0, 0, 0
    )
    msg = master.recv_match(type="GPS_RAW_INT", timeout=12, blocking=True)
    if (msg is None):
        print("No Data")
    else:
        lat = msg.lat / 1e7
        lon = mas.lon / 1e7
    return lat, lon
    if (verbose):
        print(f"Coordinates: {lat},{lon}")
        print(f"Satellites: {msg.satellites_visible}")
        print(f"Fix Type:  {msg.fix_type}")


WIDTH, HEIGHT = 1280, 720  
FRAME_SIZE = WIDTH * HEIGHT * 3

RTSP_URL = "rtsp://root:12345@192.168.1.10/stream=0"
master = mavutil.mavlink_connection("/dev/serial0", baud=115200)
master.wait_heartbeat()
print("Connected!")

mode = "GUIDED"
if mode in master.mode_mapping():
    mode_id = master.mode_mapping()[mode]
    master.set_mode(mode_id)

frame_delivery = {"latest": None}
stop_event = threading.Event()

FFMPEG_CMD = [
    "ffmpeg",
    "-rtsp_transport", "tcp",
    "-fflags",         "nobuffer",
    "-flags",          "low_delay",
    "-probesize",      "32",
    "-analyzeduration", "0",
    "-i",              RTSP_URL,
    "-an", 
    "-vf",             f"scale={WIDTH}:{HEIGHT}", 
    "-f",              "rawvideo",
    "-pix_fmt",        "bgr24",
    "-loglevel",       "warning",
    "-"
]

def stream_reader_thread(process, delivery_dict, stop_sig):
    while not stop_sig.is_set():
        frame_bytes = process.stdout.read(FRAME_SIZE)
        if len(frame_bytes) != FRAME_SIZE:
            continue
        frame = np.frombuffer(frame_bytes, dtype=np.uint8).reshape((HEIGHT, WIDTH, 3))
        delivery_dict["latest"] = frame

cap = subprocess.Popen(FFMPEG_CMD, stdout=subprocess.PIPE, stderr=subprocess.DEVNULL)

reader = threading.Thread(target=stream_reader_thread, args=(cap, frame_delivery, stop_event), daemon=True)
reader.start()

seen = set()
last_scan_time = 0.0
SCAN_INTERVAL = 0.15  # Slightly increased frequency (~6.5 scans per second)

print("Scanning.")

try:
    while True:
        if frame_delivery["latest"] is None:
            time.sleep(0.01)
            continue

        frame = frame_delivery["latest"].copy()

        cv2.imshow("QR Scanner Preview", frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

        current_time = time.time()
        if current_time - last_scan_time >= SCAN_INTERVAL:
            last_scan_time = current_time
            
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            
            clahe = cv2.createCLAHE(clipLimit=3.0, tileGridSize=(8, 8))
            enhanced_frame = clahe.apply(gray)

            results = zxingcpp.read_barcodes(enhanced_frame, formats=zxingcpp.BarcodeFormat.QRCode)
            
            for result in results:

                lat, lon = get_coords(verbose=True)
                print(f"Coordinates: {lat},{lon}")
                if result.text and result.text not in seen:
                    seen.add(result.text)
                    end_part = result.text.rstrip("/").rsplit("/", 1)[-1]
                    
                    master.mav.statustext_send(
                        mavutil.mavlink.MAV_SEVERITY_INFO,
                        end_part.encode("utf-8")[:50]
                    )
                    print(f"[{result.format}] {end_part}")

except KeyboardInterrupt:
    print("\nStopped via terminal.")
finally:
    stop_event.set()
    cap.terminate()
    cap.wait()
    cv2.destroyAllWindows()
    print("Done")