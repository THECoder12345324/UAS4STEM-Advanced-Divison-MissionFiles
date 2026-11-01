import cv2
import subprocess
import numpy as np
import zxingcpp
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
        lon = msg.lon / 1e7
    return lat, lon
    if (verbose):
        print(f"Coordinates: {lat},{lon}")
        print(f"Satellites: {msg.satellites_visible}")
        print(f"Fix Type:  {msg.fix_type}")


CENTER_X = 1280//2
CENTER_Y = 960//2

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

def send_global_gps_setpoint(vehicle, lat, lon, alt):
    lat_int = int(lat * 1e7)
    lon_int = int(lon * 1e7)
    
    type_mask = 0b110111111000 
    
    coordinate_frame = mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT

    print(f"Sending command to fly to: Lat={lat}, Lon={lon}, Alt={alt}m")
    
    vehicle.mav.set_position_target_global_int_send(
        0,                                      # time_boot_ms (ignored)
        vehicle.target_system,                  # Target system ID
        vehicle.target_component,               # Target component ID
        coordinate_frame,                       # Coordinate frame
        type_mask,                              # Bitmask ignoring unused fields
        lat_int,                                # Latitude (deg * 1E7)
        lon_int,                                # Longitude (deg * 1E7)
        alt,                                    # Altitude (meters relative to home)
        0, 0, 0,                                # X, Y, Z velocity (ignored)
        0, 0, 0,                                # X, Y, Z acceleration (ignored)
        0, 0                                    # Yaw, Yaw rate (ignored)
    )

def set_mode(mode):
    if mode in master.mode_mapping():
        mode_id = master.mode_mapping()[mode]
        master.set_mode(mode_id)


master = mavutil.mavlink_connection("/dev/ttyACM0", baud=115200)
print("Checking Heartbeat")
master.wait_heartbeat()
print("Connected!")


set_mode("GUIDED")
cap = subprocess.Popen(
    ["rpicam-vid", "--codec", "mjpeg", "--width", "1280", "--height", "960",
     "--framerate", "30", "--timeout", "0", "--autofocus-mode", "continuous",
     "--lens-position", "0", "--output", "-"],
    stdout=subprocess.PIPE, stderr=subprocess.DEVNULL
)

bytes_buffer = b""
seen = set()

print("Scanning for QR codes... Press Ctrl+C to stop.")

try:
    #kit = ServoKit(channels=16)
    #pan
    #kit.servo[0].angle = 90
    #tilt
    #kit.servo[1].angle = 0

    while True:

        msg = master.recv_match(blocking=False)

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
                    pos=result.position
                    qrx = (pos.top_left.x + pos.bottom_right.x)//2
                    qry = (pos.top_left.y + pos.bottom_right.y)//2

                    error_x = qrx - CENTER_X
                    error_y = CENTER_Y - qry

                    target_vy = pid_x.update(error_x)
                    target_vx = pid_y.update(error_y)

                    send_vel(target_vx, target_vy)
                    
                    lat, lon = get_coords()


                    if result.text and result.text not in seen:
                        seen.add(result.text)
                        end_part = result.text.rstrip("/").rsplit("/", 1)[-1]
                        master.mav.statustext_send(
                            mavutil.mavlink.MAV_SEVERITY_INFO,
                            end_part.encode("utf-8")[:50]
                        )
                        print(f"[{result.format}] New QR: {end_part} | Lat: {lat}, Lon: {lon}")
                        #set_mode("LAND")
                    else:
                        if lat and lon:
                            print(f"Centering... ErrX: {error_x}, ErrY: {error_y} | Current GPS: {lat}, {lon}", end="\r")

            else:
                send_vel(0,0)
except KeyboardInterrupt:
    print("\nStopped.")
finally:
    send_vel(0,0)
    cap.terminate()