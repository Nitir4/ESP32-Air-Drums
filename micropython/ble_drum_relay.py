"""
ble_drum_relay.py — ESP32-C3 BLE Relay (Central + Peripheral)
---------------------------------------------------------------
This ESP32 acts as BOTH:
1. BLE Central - Connects to another ESP32 drum (ESPDRUMS_1, ESPDRUMS_2, etc.)
2. BLE Peripheral - Advertises to laptop as "ESPDRUMS_RELAY"

Receives MIDI from other drum controllers and forwards to laptop.
Also has its own MPU6050 sensor to send its own drum hits.

Upload to ESP32:
    mpremote connect COM3 fs put ble_drum_relay.py :/main.py
    mpremote connect COM3 fs put mpu6050.py :/mpu6050.py
    mpremote connect COM3 reset
"""

from machine import Pin, I2C
from time import sleep, ticks_ms, ticks_diff
from mpu6050 import MPU6050
import bluetooth
import struct
import gc

# ========================================
# Configuration Block
# ========================================
# BLE Device Name for advertising to laptop
BLE_NAME = "ESPDRUMS"  # Name when connecting to laptop

# Remote drum to connect to (as central)
REMOTE_DRUM_NAME = "ESPDRUMS_1"  # Change to connect to different drum

# MIDI settings for this drum's sensor
MIDI_NOTE = 36  # Kick drum for relay (change per drum)
MIDI_VELOCITY = 100
MIDI_CHANNEL = 9  # MIDI channel 10 (drums) is 9 when zero-based
NOTE_OFF_DELAY_MS = 50

# MPU6050 I2C pins (XIAO ESP32-C3)
I2C_SCL_PIN = 7
I2C_SDA_PIN = 6

# Motion detection
SAMPLES_CAL = 500
DT = 0.05
ALPHA_ACC_LP = 0.5
THRESHOLD_HIT = -0.3
DEBOUNCE_MS = 200
DEBUG_SENSOR = False

# Power management
ENABLE_POWER_SAVE = True

# BLE UUIDs (must match ble_drum.py)
DRUM_SERVICE_UUID = bluetooth.UUID("4fafc201-1fb5-459e-8fcc-c5c9c331914b")
DRUM_CHAR_UUID = bluetooth.UUID("beb5483e-36e1-4688-b7f5-ea07361b26a8")

# BLE IRQ Events
_IRQ_CENTRAL_CONNECT = 1
_IRQ_CENTRAL_DISCONNECT = 2
_IRQ_GATTS_WRITE = 3
_IRQ_GATTS_READ_REQUEST = 4
_IRQ_SCAN_RESULT = 5
_IRQ_SCAN_DONE = 6
_IRQ_PERIPHERAL_CONNECT = 7
_IRQ_PERIPHERAL_DISCONNECT = 8
_IRQ_GATTC_SERVICE_RESULT = 9
_IRQ_GATTC_SERVICE_DONE = 10
_IRQ_GATTC_CHARACTERISTIC_RESULT = 11
_IRQ_GATTC_CHARACTERISTIC_DONE = 12
_IRQ_GATTC_NOTIFY = 15
_IRQ_MTU_EXCHANGED = 21
# ========================================


class BLEDrumRelay:
    """BLE Relay: Acts as both central (connects to drums) and peripheral (connects to laptop)."""
    
    def __init__(self, name=BLE_NAME, remote_name=REMOTE_DRUM_NAME):
        self.name = name
        self.remote_name = remote_name
        self.ble = bluetooth.BLE()
        self.ble.active(True)
        self.ble.irq(self._irq)
        
        # Peripheral mode (to laptop)
        self.laptop_connected = False
        self.laptop_conn_handle = None
        self.char_handle = None
        self._advertising = False
        
        # Central mode (to remote drum)
        self.remote_connected = False
        self.remote_conn_handle = None
        self.remote_char_handle = None
        self.remote_addr = None
        self._scanning = False
        
        # Register GATT services (peripheral mode)
        self._register_services()
        
        # Start scanning for remote drum
        self._start_scan()
        
        # Start advertising to laptop
        self._advertise()
        
    def _register_services(self):
        """Register GATT services for laptop connection (peripheral mode)."""
        CHAR_FLAG_READ = 0x0002
        CHAR_FLAG_NOTIFY = 0x0010
        
        handles = self.ble.gatts_register_services((
            (DRUM_SERVICE_UUID, (
                (DRUM_CHAR_UUID, CHAR_FLAG_READ | CHAR_FLAG_NOTIFY),
            )),
        ))
        
        self.char_handle = handles[0][0]
        print("🎹 Peripheral service registered (handle:", self.char_handle, ")")
        
    def _advertise(self, interval_us=500000):
        """Start BLE advertising to laptop."""
        if self.laptop_connected:
            return
        
        name = self.name.encode()
        adv_data = bytearray(b'\x02\x01\x06') + bytearray((len(name) + 1, 0x09)) + name
        
        self.ble.gap_advertise(interval_us, adv_data=adv_data)
        self._advertising = True
        print("📡 Advertising as:", self.name)
        
    def _start_scan(self):
        """Start scanning for remote drum (central mode)."""
        if self.remote_connected or self._scanning:
            return
        
        print("🔍 Scanning for:", self.remote_name)
        self._scanning = True
        # Scan: duration_ms=0 means continuous, interval/window in microseconds
        self.ble.gap_scan(0, 30000, 30000)
        
    def _irq(self, event, data):
        """Handle BLE events for both central and peripheral modes."""
        
        # === PERIPHERAL MODE (laptop connection) ===
        if event == _IRQ_CENTRAL_CONNECT:
            conn_handle, _, _ = data
            self.laptop_connected = True
            self.laptop_conn_handle = conn_handle
            self._advertising = False
            try:
                self.ble.gap_advertise(None)
            except:
                pass
            print("✅ Laptop connected! Handle:", conn_handle)
            
        elif event == _IRQ_CENTRAL_DISCONNECT:
            conn_handle, _, _ = data
            self.laptop_connected = False
            self.laptop_conn_handle = None
            self._advertising = False
            print("❌ Laptop disconnected")
            
        # === CENTRAL MODE (remote drum connection) ===
        elif event == _IRQ_SCAN_RESULT:
            addr_type, addr, adv_type, rssi, adv_data = data
            # Decode device name from advertising data
            try:
                name = self._decode_name(adv_data)
                if name and name == self.remote_name:
                    print(f"📡 Found {self.remote_name}! RSSI: {rssi}")
                    self.remote_addr = (addr_type, bytes(addr))
                    self.ble.gap_scan(None)  # Stop scanning
                    self._scanning = False
                    # Connect to remote drum
                    print(f"🔗 Connecting to {self.remote_name}...")
                    self.ble.gap_connect(addr_type, addr)
            except Exception as e:
                print("Scan decode error:", e)
                
        elif event == _IRQ_SCAN_DONE:
            self._scanning = False
            if not self.remote_connected and not self._scanning:
                # Retry scan after delay
                print("⏳ Scan complete, retrying in 2s...")
                
        elif event == _IRQ_PERIPHERAL_CONNECT:
            conn_handle, _, _ = data
            self.remote_connected = True
            self.remote_conn_handle = conn_handle
            print(f"✅ Connected to {self.remote_name}! Handle:", conn_handle)
            # Discover services and characteristics
            self.ble.gattc_discover_services(conn_handle)
            
        elif event == _IRQ_PERIPHERAL_DISCONNECT:
            conn_handle, _, _ = data
            self.remote_connected = False
            self.remote_conn_handle = None
            self.remote_char_handle = None
            print(f"❌ Disconnected from {self.remote_name}")
            # Restart scanning
            self._start_scan()
            
        elif event == _IRQ_GATTC_SERVICE_RESULT:
            conn_handle, start_handle, end_handle, uuid = data
            if bluetooth.UUID(uuid) == DRUM_SERVICE_UUID:
                print("🎵 Found drum service!")
                # Discover characteristics in this service
                self.ble.gattc_discover_characteristics(conn_handle, start_handle, end_handle)
                
        elif event == _IRQ_GATTC_CHARACTERISTIC_RESULT:
            conn_handle, def_handle, value_handle, properties, uuid = data
            if bluetooth.UUID(uuid) == DRUM_CHAR_UUID:
                print("🎵 Found drum characteristic! Handle:", value_handle)
                self.remote_char_handle = value_handle
                
        elif event == _IRQ_GATTC_CHARACTERISTIC_DONE:
            conn_handle, status = data
            if self.remote_char_handle:
                # Subscribe to notifications from remote drum
                print("🔔 Subscribing to remote drum notifications...")
                # Write 0x0001 to CCCD (Client Characteristic Configuration Descriptor)
                # CCCD handle is typically char_handle + 1
                self.ble.gattc_write(conn_handle, self.remote_char_handle + 1, b'\x01\x00', 1)
                print("✅ Subscribed to remote drum!")
                
        elif event == _IRQ_GATTC_NOTIFY:
            conn_handle, value_handle, notify_data = data
            # Received MIDI data from remote drum
            if value_handle == self.remote_char_handle:
                midi_data = bytes(notify_data)
                print(f"📨 Received from {self.remote_name}:", midi_data.hex())
                # Forward to laptop
                self._forward_to_laptop(midi_data)
                
        elif event == _IRQ_MTU_EXCHANGED:
            print("📊 MTU exchanged:", data)
            
    def _decode_name(self, adv_data):
        """Decode device name from advertising data."""
        i = 0
        while i < len(adv_data):
            length = adv_data[i]
            if length == 0:
                break
            ad_type = adv_data[i + 1]
            # 0x09 = Complete Local Name
            if ad_type == 0x09:
                return bytes(adv_data[i + 2:i + 1 + length]).decode('utf-8')
            i += 1 + length
        return None
        
    def _forward_to_laptop(self, midi_data):
        """Forward MIDI data from remote drum to laptop."""
        if self.laptop_connected and self.laptop_conn_handle is not None:
            try:
                self.ble.gatts_notify(self.laptop_conn_handle, self.char_handle, midi_data)
                print("✅ Forwarded to laptop")
            except Exception as e:
                print("⚠️ Forward error:", e)
        else:
            print("⚠️ Laptop not connected, dropping packet")
            
    def send_midi(self, status, note, velocity):
        """Send MIDI from this drum's sensor to laptop."""
        if not self.laptop_connected or self.laptop_conn_handle is None:
            return False
            
        try:
            msg = struct.pack("BBB", status, note, velocity)
            self.ble.gatts_notify(self.laptop_conn_handle, self.char_handle, msg)
            return True
        except OSError as e:
            print("⚠️ BLE send failed:", e)
            self.laptop_connected = False
            self.laptop_conn_handle = None
            return False
        except Exception as e:
            print("BLE send error:", e)
            return False
        
    def send_note_on(self, note, velocity=127):
        status = 0x90 | (MIDI_CHANNEL & 0x0F)
        return self.send_midi(status, note, velocity)
        
    def send_note_off(self, note):
        status = 0x80 | (MIDI_CHANNEL & 0x0F)
        return self.send_midi(status, note, 0)


# ========================================
# Power Management
# ========================================
try:
    import network
    wlan = network.WLAN(network.STA_IF)
    wlan.active(False)
    ap = network.WLAN(network.AP_IF)
    ap.active(False)
    print("WiFi disabled - BLE only mode")
except Exception as e:
    print("WiFi disable warning:", e)

if ENABLE_POWER_SAVE:
    import machine
    try:
        machine.freq(80000000)
        print("CPU frequency reduced to 80MHz for stability")
    except Exception as e:
        print("CPU frequency adjustment warning:", e)

# ========================================
# Initialize BLE Relay
# ========================================
print("Initializing BLE relay...")
try:
    ble_relay = BLEDrumRelay(BLE_NAME, REMOTE_DRUM_NAME)
    sleep(0.1)
    print("BLE relay initialized successfully")
except Exception as e:
    print("BLE initialization error:", e)
    raise

# ========================================
# Initialize MPU6050 and Calibrate
# ========================================
try:
    i2c = I2C(0, scl=Pin(I2C_SCL_PIN), sda=Pin(I2C_SDA_PIN))
    sensor = MPU6050(i2c)
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
az_bias -= 1.0

print("Calibration complete. Running...")

# ========================================
# Main Loop
# ========================================
ax_f = ay_f = az_f = 0.0
last_hit = 0
note_off_time = 0
note_off_pending = False
debug_counter = 0
last_scan_retry = 0
last_status_print = 0  # Track status message timing

try:
    while True:
        now = ticks_ms()

        # Print connection status every 5 seconds
        if ticks_diff(now, last_status_print) > 5000:
            print("\n" + "="*50)
            print(f"📊 STATUS: Relay={ble_relay.name}")
            print(f"  🔗 Remote: {ble_relay.remote_name} - {'✅ CONNECTED' if ble_relay.remote_connected else '❌ DISCONNECTED'}")
            print(f"  💻 Laptop: {'✅ CONNECTED' if ble_relay.laptop_connected else '❌ DISCONNECTED'}")
            print(f"  🔍 Scanning: {'YES' if ble_relay._scanning else 'NO'}")
            print("="*50 + "\n")
            last_status_print = now

        # Retry scanning if disconnected from remote drum
        if not ble_relay.remote_connected and not ble_relay._scanning:
            if ticks_diff(now, last_scan_retry) > 5000:
                ble_relay._start_scan()
                last_scan_retry = now

        # Re-advertise if laptop disconnected
        if not ble_relay.laptop_connected and not ble_relay._advertising:
            ble_relay._advertise()

        # Read sensor with error handling
        try:
            vals = sensor.raw_values()
            ax_r, ay_r, az_r, t, gx_r, gy_r, gz_r = vals

            ax = ax_r / 16384.0 - ax_bias
            ay = ay_r / 16384.0 - ay_bias
            az = az_r / 16384.0 - az_bias

            ax_f = ALPHA_ACC_LP * ax_f + (1 - ALPHA_ACC_LP) * ax
            ay_f = ALPHA_ACC_LP * ay_f + (1 - ALPHA_ACC_LP) * ay
            az_f = ALPHA_ACC_LP * az_f + (1 - ALPHA_ACC_LP) * az

            if DEBUG_SENSOR and ble_relay.laptop_connected:
                debug_counter += 1
                if debug_counter >= 40:
                    print("az_f: {:.3f} (threshold: {})".format(az_f, THRESHOLD_HIT))
                    debug_counter = 0

            # Hit Detection
            if az_f < THRESHOLD_HIT and ticks_diff(now, last_hit) > DEBOUNCE_MS:
                print("*** HIT DETECTED! *** → sending MIDI note", MIDI_NOTE, "| az_f:", az_f)
                
                if ble_relay.send_note_on(MIDI_NOTE, MIDI_VELOCITY):
                    print("✅ Note On sent to laptop")
                    note_off_time = now + NOTE_OFF_DELAY_MS
                    note_off_pending = True
                else:
                    print("⚠️ Note On failed (laptop not connected)")
                    
                last_hit = now

            # Non-blocking Note Off
            if note_off_pending and ticks_diff(now, note_off_time) >= 0:
                if ble_relay.send_note_off(MIDI_NOTE):
                    print("✅ Note Off sent to laptop")
                note_off_pending = False
                
        except OSError as e:
            print("⚠️ Sensor error:", e, "- Check MPU6050 wiring!")
            sleep(1.0)
            try:
                i2c = I2C(0, scl=Pin(I2C_SCL_PIN), sda=Pin(I2C_SDA_PIN))
                sensor = MPU6050(i2c)
                print("✅ Sensor reconnected")
            except Exception:
                pass

        # Memory management
        if gc.mem_free() < 20000:
            gc.collect()

        sleep(DT)

except KeyboardInterrupt:
    print("Stopped.")
finally:
    try:
        ble_relay.ble.active(False)
        print("BLE deactivated")
    except Exception:
        pass
