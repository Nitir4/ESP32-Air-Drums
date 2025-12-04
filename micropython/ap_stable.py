"""
ap_stable.py — ESP32-C3 Air Drum (Access Point/Server)
-------------------------------------------------------
Hosts an AP (ESPDRUMS) and receives UDP MIDI packets from station ESP32.
Can also forward MIDI to a connected PC via UDP broadcast.

Improvements:
- Non-blocking Note-Off (no sleep during send)
- Helper functions for cleaner code
- Configurable constants at top
- Proper WiFi initialization with error handling
- MPU6050 sensor integration with calibration

Upload to ESP32:
    mpremote connect COM3 fs put ap_stable.py :/main.py
    mpremote connect COM3 fs put mpu6050.py :/mpu6050.py
    mpremote connect COM3 reset

Or using ampy:
    ampy --port COM3 put ap_stable.py main.py
    ampy --port COM3 put mpu6050.py mpu6050.py
"""

from machine import Pin, I2C
from time import sleep, ticks_ms, ticks_diff
from mpu6050 import MPU6050
import network, socket, struct, gc

# ========================================
# Configuration Block
# ========================================
SSID = "ESPDRUMS"
PASSWORD = "Nitirocks"
AP_IP = "192.168.4.1"
SERVER_PORT = 6000
BROADCAST = '192.168.4.255'  # Broadcast to all connected devices

# MIDI settings
MIDI_NOTE = 38  # Different note from STA (snare drum)
MIDI_VELOCITY = 100
MIDI_CHANNEL = 9  # MIDI channel 10 (drums) is 9 when zero-based
NOTE_OFF_DELAY_MS = 50  # delay before sending Note Off

# MPU6050 I2C pins (XIAO ESP32-C3)
I2C_SCL_PIN = 7
I2C_SDA_PIN = 6

# Motion detection
SAMPLES_CAL = 500       # calibration sample count
DT = 0.05               # main loop sleep (seconds)
ALPHA_ACC_LP = 0.5      # low-pass filter coefficient
THRESHOLD_HIT = -0.3    # downward accel threshold (g) - more sensitive!
DEBOUNCE_MS = 200       # min time between hits (reduced for faster response)
DEBUG_SENSOR = True     # Set to False to disable sensor value printing

# Forward MIDI to PC (set to True to enable)
FORWARD_TO_PC = True
# ========================================


def setup_ap():
    """Setup Access Point mode."""
    # Disable STA if active (prevents internal error)
    sta = network.WLAN(network.STA_IF)
    if sta.active():
        print("Disabling STA interface...")
        sta.active(False)
        sleep(1.0)
    
    # Configure AP interface
    ap = network.WLAN(network.AP_IF)
    
    # Deactivate first to ensure clean state
    if ap.active():
        print("Deactivating existing AP interface...")
        ap.active(False)
        sleep(1.0)
    
    print("Activating AP interface...")
    ap.active(True)
    sleep(1.0)  # Wait for interface to fully activate
    
    # Configure AP settings
    print("Configuring AP: SSID='{}', Password='{}'".format(SSID, PASSWORD))
    ap.config(essid=SSID, password=PASSWORD, authmode=3)  # authmode=3 is WPA2-PSK
    sleep(0.5)
    
    # Verify configuration
    print("AP activated successfully!")
    print("AP ifconfig:", ap.ifconfig())
    print("Devices can connect to SSID:", SSID)
    return ap


def create_socket():
    """Create and configure UDP socket for both sending and receiving."""
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    try:
        s.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
    except Exception as e:
        print("Broadcast socket option warning:", e)
    
    # Bind to receive on SERVER_PORT
    try:
        s.bind(('0.0.0.0', SERVER_PORT))
        print("Socket bound to port {} for receiving".format(SERVER_PORT))
    except Exception as e:
        print("Bind warning:", e)
    
    s.setblocking(False)  # Non-blocking for receiving
    return s


def send_note_on(sock, dest, note, velocity=127):
    """Send MIDI Note On message."""
    try:
        status = 0x90 | (MIDI_CHANNEL & 0x0F)
        msg = struct.pack("BBB", status, note, velocity)
        sock.sendto(msg, dest)
    except OSError as e:
        print("Note On send error:", e)
        gc.collect()


def send_note_off(sock, dest, note):
    """Send MIDI Note Off message."""
    try:
        status = 0x80 | (MIDI_CHANNEL & 0x0F)
        msg = struct.pack("BBB", status, note, 0)
        sock.sendto(msg, dest)
    except OSError as e:
        print("Note Off send error:", e)
        gc.collect()


# ========================================
# Initialize Wi-Fi and Socket
# ========================================
try:
    ap = setup_ap()
    udp = create_socket()
    DEST = (BROADCAST, SERVER_PORT)
    print("Socket created and ready to send to", DEST)
except Exception as e:
    print("Initialization error:", e)
    raise

# ========================================
# Initialize MPU6050 and Calibrate
# ========================================
try:
    i2c = I2C(0, scl=Pin(I2C_SCL_PIN), sda=Pin(I2C_SDA_PIN))
    sensor = MPU6050(i2c)   # Initialize MPU6050 at default address.
                            # TO initialize at different address, use: MPU6050(i2c, addr=0x69)
    print("MPU6050 initialized successfully")
except Exception as e:
    print("MPU6050 initialization error:", e)
    raise

print("Starting calibration — keep board still.")
gx_bias = gy_bias = gz_bias = 0.0
ax_bias = ay_bias = az_bias = 0.0

for i in range(SAMPLES_CAL):
    vals = sensor.raw_values()
    ax, ay, az, t, gx, gy, gz = vals
    ax_bias += ax / 16384.0
    ay_bias += ay / 16384.0
    az_bias += az / 16384.0
    gx_bias += gx / 131.0
    gy_bias += gy / 131.0
    gz_bias += gz / 131.0
    sleep(0.01)

ax_bias /= SAMPLES_CAL
ay_bias /= SAMPLES_CAL
az_bias /= SAMPLES_CAL
gx_bias /= SAMPLES_CAL
gy_bias /= SAMPLES_CAL
gz_bias /= SAMPLES_CAL
az_bias -= 1.0  # gravity correction

print("Calibration complete. Running...")

# ========================================
# Main Loop
# ========================================
ax_f = ay_f = az_f = 0.0
last_hit = 0
note_off_time = 0  # timestamp when Note Off should be sent
note_off_pending = False
debug_counter = 0  # For periodic debug output

try:
    while True:
        now = ticks_ms()

        # Check for incoming MIDI packets from STA
        try:
            data, addr = udp.recvfrom(128)
            if len(data) >= 3:
                status = data[0]
                note = data[1]
                velocity = data[2]
                msg_type = "Note On" if (status & 0xF0) == 0x90 else "Note Off"
                print("<<< RECEIVED from {}: {} - Note={}, Vel={}".format(addr, msg_type, note, velocity))
                
                # Forward to broadcast (PC/laptop)
                if FORWARD_TO_PC:
                    udp.sendto(data, DEST)
                    print(">>> FORWARDED to {}".format(DEST))
        except OSError:
            pass  # No data available

        # Read sensor (only if available)
        if sensor:
            vals = sensor.raw_values()
            ax_r, ay_r, az_r, t, gx_r, gy_r, gz_r = vals

            ax = ax_r / 16384.0 - ax_bias
            ay = ay_r / 16384.0 - ay_bias
            az = az_r / 16384.0 - az_bias

            # Low-pass filter accel
            ax_f = ALPHA_ACC_LP * ax_f + (1 - ALPHA_ACC_LP) * ax
            ay_f = ALPHA_ACC_LP * ay_f + (1 - ALPHA_ACC_LP) * ay
            az_f = ALPHA_ACC_LP * az_f + (1 - ALPHA_ACC_LP) * az

            # Debug output every 20 loops (~1 second)
            if DEBUG_SENSOR:
                debug_counter += 1
                if debug_counter >= 20:
                    print("az_f: {:.3f} (threshold: {})".format(az_f, THRESHOLD_HIT))
                    debug_counter = 0

            # Hit Detection
            if az_f < THRESHOLD_HIT and ticks_diff(now, last_hit) > DEBOUNCE_MS:
                print("*** AP HIT! *** → sending MIDI note", MIDI_NOTE, "| az_f:", az_f)
                send_note_on(udp, DEST, MIDI_NOTE, MIDI_VELOCITY)
                print(">>> SENT to {}".format(DEST))
                note_off_time = now + NOTE_OFF_DELAY_MS
                note_off_pending = True
                last_hit = now

            # Non-blocking Note Off
            if note_off_pending and ticks_diff(now, note_off_time) >= 0:
                send_note_off(udp, DEST, MIDI_NOTE)
                note_off_pending = False

        # Memory management
        if gc.mem_free() < 20000:
            gc.collect()

        sleep(DT)

except KeyboardInterrupt:
    print("Stopped.")
finally:
    try:
        udp.close()
    except Exception:
        pass
