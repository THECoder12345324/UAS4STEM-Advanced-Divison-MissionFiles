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

def ack(out=False):
    ack = master.recv_match(type="COMMAND_ACK", timeout=2, blocking=True)
    if (out):
        print("Ack: ", ack)
    return ack

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


def writetoFile(lines):
    with open("qrcoords.txt", "a") as file:
        file.writelines(lines)
        file.write("\n")

def save_img(name, img):
    cv2.imwrite(os.path.join(subfolder, name), img)
    print("Saved!")

def qr_scan(frame):
    global lines, seen
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    
    # Fast barcode/QR detection optimized for Raspberry Pi
    results = zxingcpp.read_barcodes(gray, formats=zxingcpp.BarcodeFormat.QRCode)
    
    if keyboard.is_pressed("s"):
        save_img(f"processed_img_{time.time()}.jpg", frame)

    for result in results:
        pos = result.position
        
        # Map 2D image frame coordinates from the QR reader corner positions
        img_points = np.array([
            [pos.top_left.x, pos.top_left.y],
            [pos.top_right.x, pos.top_right.y],
            [pos.bottom_right.x, pos.bottom_right.y],
            [pos.bottom_left.x, pos.bottom_left.y]
        ], dtype=np.float32)
        
        # Solve Perspective-n-Point to get 3D translation vector (tvec)
        success, rvec, tvec = cv2.solvePnP(OBJ_POINTS, img_points, CAMERA_MATRIX, DIST_COEFFS)
        
        if success:
            x_m, y_m, z_m = tvec.flatten() # This now points to the center of the WHOLE BOARD!
            
            angle_x = np.arctan2(x_m, z_m)
            angle_y = np.arctan2(y_m, z_m)
            
            # Calculate the dynamic angular footprint of the entire 3ft x 3ft board
            size_x = 2 * np.arctan2(BOARD_SIZE / 2, z_m)
            size_y = 2 * np.arctan2(BOARD_SIZE / 2, z_m)
            
            print(f"[UAS4STEM Center Target] Dist: {z_m:.2f}m | Target X: {x_m:.2f}m | Y: {y_m:.2f}m")
            if (angle_x < 0.5 and angle_x > -0.5 and angle_y < .5 and angle_y > -0.5):
                print("Ready to drop")

            precision_land(angle_x, angle_y, z_m, size_x, size_y, descend_only=False)


#PLND_ENABLED = 1
#PLND_TYPE = 1

def precision_land(angle_x, angle_y, distance, size_x, size_y, descend_only=False):
    set_mode("GUIDED")
    target_num = 0       
    frame = 2            # MAV_FRAME_BODY_NED (Target relative to downward-facing vehicle frame)
    
    # Broadcast landing target packet using true mathematical radians and distances
    master.mav.landing_target_send(
        time_usec=int(time.time() * 1000000),
        target_num=target_num,
        frame=frame,
        angle_x=angle_x,   # Horizontal angle to target center (radians)
        angle_y=angle_y,   # Vertical angle to target center (radians)
        size_x=size_x,     # Target width footprint inside camera lens (radians)
        size_y=size_y,     # Target height footprint inside camera lens (radians)
        distance=distance, # Hard physical distance calculated directly via PnP (meters)
        position_valid=0   # 0 instructs ArduPilot to parse using angular vectors
    )
    
    if not descend_only:
        print(f"Centering Target -> Angular X: {np.degrees(angle_x):.2f}°, Y: {np.degrees(angle_y):.2f}°")
    else:
        # Descend safety fallback routine
        master.mav.set_position_target_local_ned_send(
            0,                                      
            master.target_system,                   
            master.target_component,                
            mavutil.mavlink.MAV_FRAME_BODY_NED,     
            0b0000111111000111,                     
            0, 0, 0,                                
            0, 0, 0.3,
            0, 0, 0,                                
            0, 0                                    
        )

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

CAMERA_MATRIX = np.array([
    [1739.13,    0.00,  720.00],  
    [   0.00, 1739.13,  540.00],  
    [   0.00,    0.00,    1.00]
], dtype=np.float32)

DIST_COEFFS = np.zeros((5, 1), dtype=np.float32) 

# 1. Total physical size of the outer landing board (3 feet = 0.9144 meters)
BOARD_SIZE = 0.9144  

QR_CODE_WIDTH = 0.914

QR_CENTER_X = 0 
QR_CENTER_Y =  0   

# Redefining the 4 corners of the QR code relative to the TOTAL BOARD CENTER (0,0,0)
OBJ_POINTS = np.array([
    [QR_CENTER_X - QR_CODE_WIDTH / 2, QR_CENTER_Y - QR_CODE_WIDTH / 2, 0], # Top-Left
    [QR_CENTER_X + QR_CODE_WIDTH / 2, QR_CENTER_Y - QR_CODE_WIDTH / 2, 0], # Top-Right
    [QR_CENTER_X + QR_CODE_WIDTH / 2, QR_CENTER_Y + QR_CODE_WIDTH / 2, 0], # Bottom-Right
    [QR_CENTER_X - QR_CODE_WIDTH / 2, QR_CENTER_Y + QR_CODE_WIDTH / 2, 0]  # Bottom-Left
], dtype=np.float32)

#endregion
#region -----------MAIN-----------------------------------
POILAT, POILON = get_coords()
with open("example.txt", "r") as file:
    for line in file:
        result = line.split(", ")
        if (result[0] == "MOVING-TARGET"):
            POILAT = float(result[1])
            POILON = float(result[2])
arm()
ack()
takeoff(10)
ack()
send_global_gps_setpoint(POILAT, POILON, 5)
ack()

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