import time
from smbus2 import SMBus

# Constants for PCA9685/Arducam controller
I2C_ADDRESS = 0x40  # Default Arducam servo board address
MODE1_REG = 0x00
PRESCALE_REG = 0xFE
LED0_ON_L = 0x06

# Servo configuration
SERVO_PAN_CHANNEL = 0
SERVO_TILT_CHANNEL = 1

# Pulse lengths (out of 4096) for standard servos
# Adjust these min/max values if your servos hit mechanical bounds
SERVOMIN = 150  # Minimum pulse length (0 degrees)
SERVOMAX = 600  # Maximum pulse length (180 degrees)

def init_pca9685(bus):
    """Initializes the PWM controller at 50Hz for standard RC servos."""
    # Reset device
    bus.write_byte_data(I2C_ADDRESS, MODE1_REG, 0x00)
    time.sleep(0.05)
    
    # Set frequency to 50Hz
    # Formula: prescale = round(25000000 / (4096 * 50)) - 1 = 121 (0x79)
    bus.write_byte_data(I2C_ADDRESS, MODE1_REG, 0x10) # Go to sleep to set prescale
    bus.write_byte_data(I2C_ADDRESS, PRESCALE_REG, 0x79)
    bus.write_byte_data(I2C_ADDRESS, MODE1_REG, 0x00) # Wake up
    time.sleep(0.05)
    bus.write_byte_data(I2C_ADDRESS, MODE1_REG, 0xa1) # Auto-increment enabled

def set_pwm(bus, channel, on, off):
    """Sets a specific PWM channel output."""
    reg_base = LED0_ON_L + 4 * channel
    bus.write_byte_data(I2C_ADDRESS, reg_base, on & 0xFF)
    bus.write_byte_data(I2C_ADDRESS, reg_base + 1, (on >> 8) & 0xFF)
    bus.write_byte_data(I2C_ADDRESS, reg_base + 2, off & 0xFF)
    bus.write_byte_data(I2C_ADDRESS, reg_base + 3, (off >> 8) & 0xFF)

def set_angle(bus, channel, angle):
    """Maps an angle (0 to 180) to a PWM pulse and updates the servo."""
    # Constrain angle limits to protect hardware
    angle = max(0, min(180, angle))
    
    # Map 0-180 degrees to SERVOMIN-SERVOMAX pulse duration
    pulse = SERVOMIN + int((angle / 180.0) * (SERVOMAX - SERVOMIN))
    
    # In PCA9685, we keep 'on' time at 0 and define the tick to turn 'off'
    set_pwm(bus, channel, 0, pulse)

# Main Execution Loop
try:
    with SMBus(1) as bus:
        print("Initializing Arducam Servo Controller...")
        init_pca9685(bus)
        
        print("Centering both servos to 90 degrees...")
        set_angle(bus, SERVO_PAN_CHANNEL, 90)
        set_angle(bus, SERVO_TILT_CHANNEL, 90)
        time.sleep(1)
        
        print("Starting Pan and Tilt demonstration cycle. Press Ctrl+C to exit.")
        while True:
            # Sweep Pan left and right
            
            # Reset Pan, Sweep Tilt up and down
            set_angle(bus, SERVO_PAN_CHANNEL, 90)
            time.sleep(0.5)
            
            print("Tilting Down...")
            set_angle(bus, SERVO_TILT_CHANNEL, 90)
            time.sleep(1)
            
            # Reset Tilt
            set_angle(bus, SERVO_TILT_CHANNEL, 90)
            time.sleep(0.5)

except KeyboardInterrupt:
    print("\nProgram stopped by user.")
except Exception as e:
    print(f"An error occurred: {e}")