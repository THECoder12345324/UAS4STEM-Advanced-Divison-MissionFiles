#--------IMPORT---------------
import cv2
import os
import subprocess
import numpy as np
import zxingcpp
import time
from pymavlink import mavutil
import keyboard

#-----------DEFINITION STUFF--------------
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
            dt = 0.033
        self.integral += error * dt
        derivative = (error - self.last_error) / dt
        output = self.kp * error + self.ki * self.integral + self.kd * derivative
        output = max(min(output, self.max), -self.max)
        self.last_error = error
        self.last_time = current_time
        return output

def calculate_coord_offset(offx, offy, unit):
    msg = master.recv_match(type="GPS_RAW_INT", timeout=2, blocking=True)
    if (msg is None):
        print("Couldn't calculate offset")
    else:
        lat = msg.lat / 1e7
        lon = msg.lon / 1e7
        alt = msg.alt / 1000
    msg = master.recv_match(type="ATTITUDE", timeout=2, blocking=True)
    if (msg is None):
        print("Coudn't get the yaw")
    else:
        yaw = msg.yaw
    
    #Calculating the coordinate offset
    r = np.sqrt(offx*offx + offy * offy)
    theta = np.tan(offx/offy)
    gamma = theta - yaw

    scale = alt * unit

    truey = r * np.cos(gamma) * scale
    truex = r * np.sin(gamma) * scale
    lat = truex * .000009 + lat
    lon = truey * .000009 + lon

    print(f"Coordinates of the point: {lat}, {lon}")
    return lat, lon



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
    if (verbose):
        print(f"Coordinates: {lat},{lon}")
        print(f"Satellites: {msg.satellites_visible}")
        print(f"Fix Type:  {msg.fix_type}")
    return lat, lon
    

def arm():
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0,
        1, 0, 0, 0, 0, 0, 0
    )
    master.motors_armed_wait()

def takeoff(alt):
    master.mav.command_long_send(
        master.target_system, 
        master.target_component,
        mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
        0,
        0, 0, 0, 0, 0, 0, 
        alt
    )
    ack_msg = master.recv_match(type='COMMAND_ACK', blocking=True)
    if (ack_msg):
        print(ack.msg.result)
    else:
        print("no ack received")
    
def circle(lat, lon, alt, r):
    lat = lat * 10**7
    lon = lon * 10**7
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_NAV_LOITER_UNLIM,
        0,
        r,
        0,
        0,
        0,
        lat,
        lon,
        alt
    )

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

def send_global_gps_setpoint(lat, lon, alt):
    lat_int = int(lat * 1e7)
    lon_int = int(lon * 1e7)
    
    type_mask = 0b110111111000 
    
    coordinate_frame = mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT

    print(f"Sending command to fly to: Lat={lat}, Lon={lon}, Alt={alt}m")
    
    master.mav.set_position_target_global_int_send(
        0,                                      # time_boot_ms (ignored)
        master.target_system,                   # Target system ID
        master.target_component,                # Target component ID
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

def set_yaw(heading, relative=0):
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_CONDITION_YAW,
        0,
        heading,
        0,
        1,
        relative,
        0, 0, 0
    )

def writetoFile(lines):
    with open("qrcoords.txt", "a") as file:
        file.writelines(lines)
        file.write("\n")

def save_img(name, img):
    cv2.imwrite(os.path.join(subfolder, name), img)
    print("Saved!")

def qr_scan(frame):
    global lines, seen
    #gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    #blurred = cv2.GaussianBlur(gray, (3, 3), 0)

    results = zxingcpp.read_barcodes(frame, formats=zxingcpp.BarcodeFormat.QRCode)
    for result in results:
        pos = result.position
        qrx = (pos.top_left.x + pos.bottom_right.x)//2
        qry = (pos.top_left.y + pos.bottom_right.y)//2

        error_x = qrx - CENTER_X
        error_y = CENTER_Y - qry

        precision_land(error_x, error_y)

        lat, lon = get_coords()
        qrlat = 0
        qrlon = 0
        qrlat, qrlon = calculate_coord_offset(error_x, error_y, UNIT_FACTOR)

        if result.text and result.text not in seen:
            save_img(f"frame_{time.time()}.jpg", frame)
            seen.add(result.text)
            end_part = result.text.rstrip("/").rsplit("/", 1)[-1]
            master.mav.statustext_send(
                mavutil.mavlink.MAV_SEVERITY_INFO,
                end_part.encode("utf-8")[:50]
            )
            lines[0] = (f"URL: {result.text} | QR: {end_part} | Drone Coords: {lat},{lon} | QR Coords: {qrlat}, {qrlon}")
            print(f"URL: {result.text} | QR: {end_part} | Drone Coords: {lat},{lon} | QR Coords: {qrlat}, {qrlon}")
            writetoFile(lines)


#PLND_ENABLED = 1
#PLND_TYPE = 1

def precision_land(error_x, error_y):
    target_num = 0       # Target identifier
    frame = 2
    size = 0.91
    distance = 0.0
    angle_x = error_x * (FOVH) / WIDTH
    angle_y = error_y * (FOVV) / HEIGHT
    master.mav.landing_target_send(
        time_usec=int(time.time() * 1000000),
        target_num=target_num,
        frame=frame,
        angle_x=angle_x,
        angle_y=angle_y,
        distance=distance,
        size=size,
        position_valid=position_valid
    )
    set_mode("LAND")

#-----------SETUP, PID, MAVLINK_CONNECTION-------
WIDTH = 1280
HEIGHT = 800
CENTER_X = WIDTH//2
CENTER_Y = HEIGHT//2
UNIT_FACTOR = 0.001
FOVH = 72 * 3.14159265/180
FOVV = 48 * 3.14159265/180
subfolder = "qrcodecaptures"
lines = ["NONE"]

if not os.path.exists(subfolder):
    os.makedirs(subfolder)


pid_x = PID(.001, .0001, .00005, 1)
pid_y = PID(.001, .0001, .00005, 1)

cap = subprocess.Popen(
    ["rpicam-vid", "--codec", "mjpeg", "--width", f"{WIDTH}", "--height", f"{HEIGHT}",
     "--framerate", "15", "--timeout", "0", 
     #"--autofocus-mode", "continuous",
     #"--exposure", "sport",
     "--lens-position", "0", 
     "--output", "-"],
    stdout=subprocess.PIPE, stderr=subprocess.DEVNULL
)

master = mavutil.mavlink_connection("/dev/ttyACM0", baud=115200)
print("Checking Heartbeat")
master.wait_heartbeat()
print("Connected!")

bytes_buffer = b""
seen = set()

print("Scanning for QR codes... Press Ctrl+C to stop.")
state = "SEARCHING"

try:
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
                if (keyboard.is_pressed('p')):
                    print('pressed')
                    save_img(f"07-12-2026_{time.time()}.jpg", frame)
                if (keyboard.is_pressed('c')):
                    get_coords()
                qr_scan(frame)

except KeyboardInterrupt:
    print("\nStopped.")
finally:
    cap.terminate()

