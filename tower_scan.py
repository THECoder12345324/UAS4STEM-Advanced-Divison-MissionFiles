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
    
    
def set_mode(mode):
    if mode in master.mode_mapping():
        mode_id = master.mode_mapping()[mode]
        master.set_mode(mode_id)
    
    ack = master.recv_match(type="COMMAND_ACK", timeout=2, blocking=True)
    #print("ACK:", ack)

def writetoFile(lines):
    with open("qrcoords.txt", "a") as file:
        file.writelines(lines)
        file.write("\n")

def save_img(name, img):
    cv2.imwrite(os.path.join(subfolder, name), img)
    print("Saved!")

def qr_scan(frame, tryharder=False):
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
    
    if (tryharder==True):
        if (len(results) == 0):
            found_square, err_x, err_y = find_square_target(frame)
            if found_square:
                print("QR text unreadable, but square frame detected. Descending to get closer...")
                precision_land(err_x, err_y, descend_only=True)

    for result in results:
        pos = result.position
        qrx = (pos.top_left.x + pos.bottom_right.x)//2
        qry = (pos.top_left.y + pos.bottom_right.y)//2

        error_x = qrx - CENTER_X
        error_y = CENTER_Y - qry

        if result.text:
            end_part = result.text.rstrip("/").rsplit("/", 1)[-1]
            if (result.text not in seen):
                #set_mode("GUIDED")
                save_img(f"frame_imx296_{time.time()}.jpg", frame)
                seen.add(result.text)
                print(f"DEFECT: {result.text} | QR: {end_part} | Drone Coords: {lat},{lon} | Time: {time.time()}")
                time.sleep(2)
                #set_mode("AUTO")
            print(f"Still scanning {end_part}")

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
     "--framerate", "15", "--timeout", "0", "gain", "2.0,2.0", #"--shutter", "1",
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