"""
main_sta.py — ESP32-C3 Air Drum (Station Stick)
------------------------------------------------
Connects to the AP (ESPDRUMS) hosted by another ESP32-C3.
Sends UDP MIDI packets to the server on port 6000.

Each downward acceleration beyond threshold sends:
    Note On  (0x99, note, velocity)
    Note Off (0x89, note, 0) after NOTE_OFF_DELAY_MS

Improvements:
- Non-blocking Note-Off (no sleep during send)
- Auto-reconnect if Wi-Fi drops
- Helper functions for cleaner code
- Configurable constants at top

Upload to ESP32:
    mpremote connect COM3 fs put main_sta.py :/main.py
    mpremote connect COM3 fs put mpu6050.py :/mpu6050.py
    mpremote connect COM3 reset

Or using ampy:
    ampy --port COM3 put main_sta.py main.py
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
SERVER_IP = "192.168.4.1"   # IP of AP (ESP #1)
SERVER_PORT = 6000

# MIDI settings
MIDI_NOTE = 40
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

# Auto-reconnect
RECONNECT_CHECK_MS = 3000  # check connection every 3s (more frequent)
# ========================================


def keep_alive_ping():
    """Send a small packet to keep connection alive."""
    # This will be called periodically to keep the WiFi connection active
    pass


def setup_sta():
    """Disable AP interface and connect to Station network."""
    # Disable AP if active (prevents internal error)
    ap = network.WLAN(network.AP_IF)
    if ap.active():
        print("Disabling AP interface...")
        ap.active(False)
        sleep(1.0)  # Longer delay for AP to fully deactivate
    
    # Configure Station interface
    sta = network.WLAN(network.STA_IF)
    
    # Deactivate first to ensure clean state
    if sta.active():
        print("Deactivating existing STA interface...")
        sta.active(False)
        sleep(1.0)  # Increased delay
    
    print("Activating STA interface...")
    sta.active(True)
    sleep(1.0)  # Wait for interface to fully activate
    
    # Enable power management for stability (instead of disabling it)
    try:
        sta.config(pm=sta.PM_PERFORMANCE)  # Better than 0xa11140
        print("Power management configured")
    except Exception as e:
        print("Power config warning:", e)
    
    # Disconnect any existing connection
    try:
        if sta.isconnected():
            sta.disconnect()
            sleep(0.5)
    except Exception as e:
        print("Disconnect warning:", e)
    
    # Scan for available networks first
    print("Scanning for WiFi networks...")
    try:
        networks = sta.scan()
        print("Found {} networks:".format(len(networks)))
        ap_found = False
        for net in networks:
            ssid = net[0].decode('utf-8')
            rssi = net[3]
            print("  - {} (RSSI: {})".format(ssid, rssi))
            if ssid == SSID:
                ap_found = True
                print("    ^ Target AP found with signal strength:", rssi)
        
        if not ap_found:
            print("WARNING: Target AP '{}' not found in scan!".format(SSID))
            print("But will try to connect anyway...")
            print("Make sure the AP ESP32 is running and in range.")
    except Exception as e:
        print("Scan error (will try to connect anyway):", e)
    
    sleep(1.0)  # Give WiFi radio time to settle after scan
    
    print("Connecting to AP:", SSID)
    sta.connect(SSID, PASSWORD)
    sleep(1.0)  # Give more time for connection to start

    # Wait for connection with timeout
    timeout = 40  # 40 iterations * 0.5s = 20 seconds
    no_ap_count = 0
    while not sta.isconnected() and timeout > 0:
        status = sta.status()
        # Status codes:
        # 1001 = STAT_CONNECTING
        # 201 = STAT_NO_AP_FOUND
        # 202 = STAT_WRONG_PASSWORD
        # 203 = STAT_BEACON_TIMEOUT
        # 204 = STAT_ASSOC_FAIL
        # 1010 = STAT_GOT_IP (connected)
        
        if status == 201:  # NO_AP_FOUND
            no_ap_count += 1
            if no_ap_count == 5:
                print("Still searching for AP...")
            elif no_ap_count >= 15:
                print("Cannot find AP after many attempts.")
                print("Trying one more scan and reconnect...")
                try:
                    sta.disconnect()
                    sleep(0.5)
                    networks = sta.scan()
                    for net in networks:
                        if net[0].decode('utf-8') == SSID:
                            print("Found AP in rescan! Reconnecting...")
                            sta.connect(SSID, PASSWORD)
                            no_ap_count = 0
                            break
                except Exception as e:
                    print("Rescan error:", e)
                    
        elif status == 202:
            print("ERROR: Wrong password!")
            raise OSError("Wrong password for AP '{}'".format(SSID))
        elif status == 1001:
            if timeout % 4 == 0:  # Print every 2 seconds
                print("  ... connecting...")
        else:
            print("  ... status:", status)
            
        sleep(0.5)
        timeout -= 1

    if not sta.isconnected():
        status = sta.status()
        print("Connection failed! Final status:", status)
        print("This could mean:")
        print("  - AP is out of range (move ESP32s closer)")
        print("  - AP hasn't started yet (start AP first)")
        print("  - WiFi interference (try again)")
        raise OSError("Failed to connect to {} network (status: {})".format(SSID, status))

    print("Connected! ifconfig:", sta.ifconfig())
    return sta


def create_socket():
    """Create and configure UDP socket."""
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    s.settimeout(0.2)
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


def check_reconnect(sta, last_check):
    """Check connection and reconnect if needed. Returns updated last_check timestamp."""
    now = ticks_ms()
    if ticks_diff(now, last_check) < RECONNECT_CHECK_MS:
        return last_check
    
    if not sta.isconnected():
        print("Connection lost! Reconnecting...")
        
        # Full reset of the connection
        try:
            sta.disconnect()
            sleep(0.5)
        except Exception:
            pass
        
        # Reconnect with full timeout
        sta.connect(SSID, PASSWORD)
        timeout = 20  # 10 seconds
        retry_count = 0
        
        while not sta.isconnected() and timeout > 0:
            status = sta.status()
            
            if status == 201:  # NO_AP_FOUND
                retry_count += 1
                if retry_count >= 3:
                    print("AP lost! Scanning...")
                    try:
                        networks = sta.scan()
                        found = False
                        for net in networks:
                            if net[0].decode('utf-8') == SSID:
                                print("  AP found again, retrying connection...")
                                found = True
                                break
                        if not found:
                            print("  AP still not visible!")
                    except Exception as e:
                        print("  Scan error:", e)
                    retry_count = 0
                    
            elif status == 202:  # Wrong password
                print("Password error - stopping reconnect attempts")
                return now
                
            sleep(0.5)
            timeout -= 1
            
        if sta.isconnected():
            print("Reconnected successfully:", sta.ifconfig())
        else:
            print("Reconnect failed (status: {}), will retry in {}s".format(
                sta.status(), RECONNECT_CHECK_MS // 1000))
    
    return now


# ========================================
# Initialize Wi-Fi and Socket
# ========================================
try:
    sta = setup_sta()
    udp = create_socket()
    DEST = (SERVER_IP, SERVER_PORT)
    print("Socket created and ready to send to", DEST)
except Exception as e:
    print("Initialization error:", e)
    raise

# ========================================
# Initialize MPU6050 and Calibrate
# ========================================
sensor = None
try:
    print("Initializing I2C on SCL={}, SDA={}...".format(I2C_SCL_PIN, I2C_SDA_PIN))
    i2c = I2C(0, scl=Pin(I2C_SCL_PIN), sda=Pin(I2C_SDA_PIN), freq=400000)
    sleep(0.1)
    
    # Scan for I2C devices
    devices = i2c.scan()
    print("I2C devices found:", [hex(d) for d in devices])
    
    if len(devices) == 0:
        print("WARNING: No I2C devices found!")
        print("Check wiring: SCL->GPIO{}, SDA->GPIO{}".format(I2C_SCL_PIN, I2C_SDA_PIN))
        print("Continuing without MPU6050 (no hit detection)...")
    else:
        # Use address 0x69 for this STA device
        sensor = MPU6050(i2c, addr=0x69)
        print("MPU6050 initialized successfully at address 0x69")
except Exception as e:
    print("MPU6050 initialization error:", e)
    print("Continuing without MPU6050 (no hit detection)...")
    sensor = None

if sensor:
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
else:
    print("Skipping calibration (no sensor). Running in send-only mode...")
    ax_bias = ay_bias = az_bias = 0.0
    gx_bias = gy_bias = gz_bias = 0.0

# ========================================
# Main Loop
# ========================================
ax_f = ay_f = az_f = 0.0
last_hit = 0
last_reconnect_check = 0
note_off_time = 0  # timestamp when Note Off should be sent
note_off_pending = False
debug_counter = 0  # For periodic debug output

try:
    while True:
        now = ticks_ms()

        # Auto-reconnect check (more frequent)
        last_reconnect_check = check_reconnect(sta, last_reconnect_check)
        
        # Only proceed if connected
        if not sta.isconnected():
            print("Not connected, waiting for reconnection...")
            sleep(1.0)
            continue

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
                    print("az_f: {:.3f} (threshold: {}) | Connected: {}".format(
                        az_f, THRESHOLD_HIT, sta.isconnected()))
                    debug_counter = 0

            # Hit Detection
            if az_f < THRESHOLD_HIT and ticks_diff(now, last_hit) > DEBOUNCE_MS:
                if sta.isconnected():  # Double-check connection before sending
                    print("*** STA HIT! *** → sending MIDI note", MIDI_NOTE, "| az_f:", az_f)
                    try:
                        send_note_on(udp, DEST, MIDI_NOTE, MIDI_VELOCITY)
                        print(">>> SENT Note On to {}:{}".format(SERVER_IP, SERVER_PORT))
                        note_off_time = now + NOTE_OFF_DELAY_MS
                        note_off_pending = True
                        last_hit = now
                    except OSError as e:
                        print("Send failed (connection issue?):", e)
                        # Force reconnect check on next iteration
                        last_reconnect_check = 0
                else:
                    print("Hit detected but not connected!")

            # Non-blocking Note Off
            if note_off_pending and ticks_diff(now, note_off_time) >= 0:
                if sta.isconnected():
                    try:
                        send_note_off(udp, DEST, MIDI_NOTE)
                        print(">>> SENT Note Off to {}:{}".format(SERVER_IP, SERVER_PORT))
                    except OSError as e:
                        print("Note Off send failed:", e)
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
