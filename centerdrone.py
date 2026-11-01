import cv2
import subprocess
import numpy as np
import zxingcpp

cap = subprocess.Popen(
    ["rpicam-vid", "--codec", "mjpeg", "--width", "1280", "--height", "960",
     "--framerate", "30", "--timeout", "0", "--autofocus-mode", "continuous",
     "--lens-position", "0", "--output", "-"],
    stdout=subprocess.PIPE, stderr=subprocess.DEVNULL
)

Kp_x = 0.5  # Proportional gain for X-axis (forward/backward or roll)
Kd_x = 0.1  # Derivative gain for X-axis

Kp_y = 0.5  # Proportional gain for Y-axis (yaw or left/right)
Kd_y = 0.1

previous_error_x = 0
previous_error_y = 0

bytes_buffer = b""
seen = set()

print("Scanning for QR codes... Press Ctrl+C to stop.")

try:
    while True:
        bytes_buffer += cap.stdout.read(4096)
        a = bytes_buffer.find(b"\xff\xd8")
        b = bytes_buffer.find(b"\xff\xd9")

        if a != -1 and b != -1 and b > a:
            jpg = bytes_buffer[a:b+2]
            bytes_buffer = bytes_buffer[b+2:]

            frame = cv2.imdecode(np.frombuffer(jpg, dtype=np.uint8), cv2.IMREAD_COLOR)

            if frame is not None:
                height, width, _ = frame.shape
                frame_center_x = width // 2
                frame_center_y = height // 2

                results = zxingcpp.read_barcodes(frame)
                for result in results:
                    pos = result.position
                    qr_x = sum([p.x for p in [pos.top_left, pos.top_right, pos.bottom_left, pos.bottom_right]]) // 4
                    qr_y = sum([p.y for p in [pos.top_left, pos.top_right, pos.bottom_left, pos.bottom_right]]) // 4
                    offset_x = qr_x - frame_center_x
                    offset_y = qr_y - frame_center_y
                    vx, vy = centerdrone(offset_x, offset_y)
                    sendPID(vx, vy)
                    if result.text and result.text not in seen:
                        seen.add(result.text)
                        print(f"[{result.format}] {result.text}")

def centerdrone(x_offset, y_offset):
    global previous_error_x, previous_error_y
    error_x = target_x - current_x
    error_y = target_y - current_y
    
    # 3. Calculate the Derivative (rate of change)
    delta_time = 1.0 # Assume 1 second intervals for loop
    derivative_x = (error_x - previous_error_x) / delta_time
    derivative_y = (error_y - previous_error_y) / delta_time
    
    # 4. Compute Control Output (PID formula)
    output_x = (Kp_x * error_x) + (Kd_x * derivative_x)
    output_y = (Kp_y * error_y) + (Kd_y * derivative_y)
    
    # 5. Save current error for the next cycle
    previous_error_x = error_x
    previous_error_y = error_y
    
    # Clip speeds so they don't exceed the drone's maximum allowed velocity
    output_x = np.clip(output_x, -20, 20)
    output_y = np.clip(output_y, -20, 20)
    
    return output_x, output_y

def sendPID(vx, vy):
    master.mav.set_position_target_local_ned_send(
        time_boot_ms=int(time.time() * 1000) & 0xFFFFFFFF,
        target_system=master.target_system,
        target_component=master.target_component,
        coordinate_frame=mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED, # Moves relative to drone heading
        type_mask=0b0000111111000111, # Ignores position and yaw, uses velocity
        x=vx, # Forward/backward velocity
        y=vy, # Left/right velocity
        z=0.0, # Up/down velocity
        vx=0.0, vy=0.0, vz=0.0,
        afx=0.0, afy=0.0, afz=0.0,
        yaw=0.0, yaw_rate=0.0
    )
    

except KeyboardInterrupt:
    print("\nStopped.")
finally:
    cap.terminate()


