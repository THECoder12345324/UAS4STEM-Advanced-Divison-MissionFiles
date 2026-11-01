#region -----------IMPORT---------------
import cv2
import os
import subprocess
import numpy as np
import zxingcpp
import time
from pymavlink import mavutil
import keyboard
import threading
from http.server import BaseHTTPRequestHandler, HTTPServer
#endregion
#region -----------DEFINITION STUFF--------------
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

def ack(out=False):
    ack = master.recv_match(type="COMMAND_ACK", timeout=2, blocking=True)
    if (out):
        print("Ack: ", ack)
    return ack

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
    set_mode("GUIDED")
    master.mav.command_long_send(
        master.target_system, 
        master.target_component,
        mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
        0,
        0, 0, 0, 0, 0, 0, 
        alt
    )
    ack_msg = master.recv_match(type='COMMAND_ACK', timeout=2, blocking=True)
    if (ack_msg):
        print(ack_msg)
    else:
        print("no ack received")
    
def circle(lat, lon, alt, radius, velocity=1.0):
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_DO_SET_ROI,
        0,          # Confirmation
        0, 0, 0, 0, # Unused params
        lat,    # param5: Latitude
        lon,    # param6: Longitude
        alt     # param7: Altitude
    )

    ack = master.recv_match(type="COMMAND_ACK", blocking=True, timeout=2)
    print(ack)

    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        master.mavlink.MAV_CMD_NAV_LOITER_TURNS,
        2,
        0,
        radius,
        0,
        0,
        0,
        alt
    )
    ack = master.recv_match(type="COMMAND_ACK", blocking=True, timeout=2)
    print(ack)

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
    
    #type_mask = 0b110111111000 
    type_mask = (
        mavutil.mavlink.POSITION_TARGET_TYPEMASK_VX_IGNORE |
        mavutil.mavlink.POSITION_TARGET_TYPEMASK_VY_IGNORE |
        mavutil.mavlink.POSITION_TARGET_TYPEMASK_VZ_IGNORE |
        mavutil.mavlink.POSITION_TARGET_TYPEMASK_AX_IGNORE |
        mavutil.mavlink.POSITION_TARGET_TYPEMASK_AY_IGNORE |
        mavutil.mavlink.POSITION_TARGET_TYPEMASK_AZ_IGNORE |
        mavutil.mavlink.POSITION_TARGET_TYPEMASK_YAW_IGNORE |
        mavutil.mavlink.POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE
        )
    
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
    
    ack = master.recv_match(type="COMMAND_ACK", timeout=2, blocking=True)
    #print("ACK:", ack)

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
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    
    # 2. Increase contrast (CLAHE works best for varying light conditions)
    #clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8,8))
    #enhanced = clahe.apply(gray)
    
    # 3. Apply a slight sharpening filter to crisp up the QR edges
    #kernel = np.array([[0, -1, 0], [-1, 5, -1], [0, -1, 0]])
    #sharpened = cv2.filter2D(enhanced, -1, kernel)

    results = zxingcpp.read_barcodes(gray, formats=zxingcpp.BarcodeFormat.QRCode)
    if (keyboard.is_pressed("s")):
        save_img(f"processed_img_{time.time()}.jpg", frame)
    
    # if (len(results) == 0):
    #     found_square, err_x, err_y = find_square_target(frame)
    #     if found_square:
    #         print("QR text unreadable, but square frame detected. Descending to get closer...")
    #         precision_land(err_x, err_y, descend_only=True)

    for result in results:
        pos = result.position
        qrx = (pos.top_left.x + pos.bottom_right.x)//2
        qry = (pos.top_left.y + pos.bottom_right.y)//2

        error_x = qrx - CENTER_X
        error_y = CENTER_Y - qry

        #precision_land(error_x, error_y)

        lat, lon = get_coords()
        qrlat = 0
        qrlon = 0
        #qrlat, qrlon = calculate_coord_offset(error_x, error_y, UNIT_FACTOR)

        if result.text:
            end_part = result.text.rstrip("/").rsplit("/", 1)[-1]
            # master.mav.statustext_send(
            #     mavutil.mavlink.MAV_SEVERITY_INFO,
            #     end_part.encode("utf-8")[:50]
            # )
            if (result.text not in seen):
                save_img(f"frame_imx296_{time.time()}.jpg", frame)
                lines[0] = (f"{end_part}, {lat}, {lon}")
                writetoFile(lines)
            print(f"URL: {result.text} | QR: {end_part} | Drone Coords: {lat},{lon} | QR Coords: {qrlat}, {qrlon} | Time: {time.time()}")
            seen.add(result.text)


#PLND_ENABLED = 1
#PLND_TYPE = 1

def precision_land(error_x, error_y, descend_only=False):
    set_mode("GUIDED")
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
        angle_x=angle_x*2.5,
        angle_y=angle_y*2.5,
        size_x=.6,
        size_y=.6,
        distance=distance,
        position_valid=0
    )
    if (descend_only==False):
        print(f"Trying to center: {error_x}, {error_y}")
        set_mode("LAND")
    else:
        master.mav.set_position_target_local_ned_send(
            0,                                      # time_boot_ms
            master.target_system,                   # Target system
            master.target_component,                # Target component
            mavutil.mavlink.MAV_FRAME_BODY_NED,     # Relative to drone body
            0b0000111111000111,                     # Bitmask: control velocities only
            0, 0, 0,                                # X, Y, Z positions (ignored)
            0, 0, vz,                               # vx, vy, vz (vz is positive downward)
            0, 0, 0,                                # ax, ay, az (ignored)
            0, 0                                    # yaw, yaw_rate (ignored)
        )

def find_square_target(frame):
    # 1. Convert to grayscale and blur to reduce high-frequency camera noise
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    blurred = cv2.GaussianBlur(gray, (5, 5), 0)
    
    # 2. Use Canny edge detection
    edged = cv2.Canny(blurred, 50, 150)
    
    # 3. Find structural outlines (contours) in the image
    contours, _ = cv2.findContours(edged, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    
    for c in contours:
        # Approximate the contour geometry to determine number of sides
        peri = cv2.arcLength(c, True)
        approx = cv2.approxPolyDP(c, 0.04 * peri, True)
        
        # A square/rectangle will have exactly 4 vertices after approximation
        if len(approx) == 4:
            # Compute the bounding box dimensions
            (x, y, w, h) = cv2.boundingRect(approx)
            
            # Filter out tiny specks or background noise
            if w > 40 and h > 40:
                aspect_ratio = float(w) / h
                
                # A QR code target will maintain a fairly uniform aspect ratio (close to 1.0)
                if 0.75 <= aspect_ratio <= 1.25:
                    # Calculate center coordinates of the square
                    square_cx = x + (w // 2)
                    square_cy = y + (h // 2)
                    
                    # Compute pixel error relative to the camera's exact center
                    error_x = square_cx - CENTER_X
                    error_y = CENTER_Y - square_cy
                    return True, error_x, error_y
                    
    return False, 0, 0
#endregion
#region -----------HTTP WEB SERVER STREAMING LOGIC-------
latest_raw_mjpeg = b''
frame_lock = threading.Lock()

class MJPEGStreamHandler(BaseHTTPRequestHandler):
    def do_GET(self):
        if self.path == '/':
            self.send_response(200)
            self.send_header('Content-type', 'text/html')
            self.end_headers()
            html = "<html><head><title>Drone Stream</title></head><body><img src='/stream' width='100%'/></body></html>"
            self.wfile.write(html.encode('utf-8'))
        elif self.path == '/stream':
            self.send_response(200)
            self.send_header('Content-type', 'multipart/x-mixed-replace; boundary=frame')
            self.end_headers()
            try:
                while True:
                    with frame_lock:
                        frame = latest_raw_mjpeg
                    if frame:
                        self.wfile.write(b'--frame\r\n')
                        self.send_header('Content-Type', 'image/jpeg')
                        self.send_header('Content-Length', str(len(frame)))
                        self.end_headers()
                        self.wfile.write(frame)
                        self.wfile.write(b'\r\n')
                    time.sleep(0.06)
            except Exception:
                pass

def run_web_server():
    server = HTTPServer(('0.0.0.0', 8000), MJPEGStreamHandler)
    server.serve_forever()
#endregion
#region -----------INITIALIZATION & CONSTANTS------------
WIDTH = 1440
HEIGHT = 1080
CENTER_X = WIDTH//2
CENTER_Y = HEIGHT//2
UNIT_FACTOR = 0.001

## OV9281 - 72x48
## IMX296 - 53x41
FOVH = 53 * 3.14159265/180
FOVV = 41 * 3.14159265/180
subfolder = "qrcodecaptures"
lines = ["NONE"]

if not os.path.exists(subfolder):
    os.makedirs(subfolder)

cap = subprocess.Popen(
    ["rpicam-vid", "--codec", "mjpeg", "--width", f"{WIDTH}", "--height", f"{HEIGHT}",
     "--framerate", "15", "--timeout", "0", "gain", "1.0,1.0", #"--shutter", "1",
     "--output", "-"],
    stdout=subprocess.PIPE, stderr=subprocess.DEVNULL
)
web_thread = threading.Thread(target=run_web_server, daemon=True)
web_thread.start()
print("Web server running at http://[PI_IP]:8000")

master = mavutil.mavlink_connection("/dev/ttyACM0", baud=115200)
print("Checking Heartbeat")
master.wait_heartbeat()
print("Connected!")

bytes_buffer = b""
seen = set()
print("Init Complete")
#endregion
#region -----------MAIN-----------------------------------
try:
    while True:
        bytes_buffer += cap.stdout.read(4096)
        a = bytes_buffer.find(b"\xff\xd8")
        b = bytes_buffer.find(b"\xff\xd9")

        if a != -1 and b != -1 and b > a:
            jpg = bytes_buffer[a:b+2]
            bytes_buffer = bytes_buffer[b+2:]

            # Pass the raw camera bytes directly to the global frame handler for the network thread
            with frame_lock:
                latest_raw_mjpeg = jpg

            frame = cv2.imdecode(np.frombuffer(jpg, dtype=np.uint8), cv2.IMREAD_COLOR)

            if frame is not None:
                qr_scan(frame)
#endregion
#region -----------CLOSE---------------------------------
except KeyboardInterrupt:
    print("\nStopped.")
finally:
    cap.terminate()
#endregion