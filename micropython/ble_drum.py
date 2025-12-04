"""
ble_drum.py — ESP32-C3 Air Drum (Bluetooth Low Energy)
--------------------------------------------------------
BLE peripheral that sends MIDI data over Bluetooth when drum hits are detected.
Uses MPU6050 accelerometer for hit detection.

This replaces the WiFi/UDP version with BLE, allowing the desktop app
to stay connected to WiFi while receiving drum data.

Upload to ESP32:
    mpremote connect COM3 fs put ble_drum.py :/main.py
    mpremote connect COM3 fs put mpu6050.py :/mpu6050.py
    mpremote connect COM3 reset

Or using ampy:
    ampy --port COM3 put ble_drum.py main.py
    ampy --port COM3 put mpu6050.py mpu6050.py
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
# BLE Device Name (must match device_prefix in desktop app)
BLE_NAME = "ESPDRUMS_1"  # Change suffix for multiple drums (e.g., _2, _3)

# MIDI settings
MIDI_NOTE = 38  # Snare drum by default (change per drum)
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
DEBUG_SENSOR = True     # Set to True to enable sensor value printing (reduces stability)

# Power management (helps prevent brownout)
ENABLE_POWER_SAVE = True  # Reduce CPU frequency for stability

# BLE UUIDs (custom service for drum data)
# These must match the UUIDs in ble_server.py
DRUM_SERVICE_UUID = bluetooth.UUID("4fafc201-1fb5-459e-8fcc-c5c9c331914b")
DRUM_CHAR_UUID = bluetooth.UUID("beb5483e-36e1-4688-b7f5-ea07361b26a8")
# ========================================


class BLEDrum:
    """BLE MIDI drum peripheral."""
    
    def __init__(self, name=BLE_NAME):
        self.name = name
        self.ble = bluetooth.BLE()
        self.ble.active(True)
        self.ble.irq(self._irq)
        self.connected = False
        self.conn_handle = None
        self._advertising = False
        
        # Register GATT services
        self._register_services()
        
        # Start advertising
        self._advertise()
        
    def _register_services(self):
        """Register BLE GATT services and characteristics."""
        # Characteristic flags
        CHAR_FLAG_READ = 0x0002
        CHAR_FLAG_NOTIFY = 0x0010
        
        # Register service and characteristic
        # Returns ((char_value_handle,),)
        handles = self.ble.gatts_register_services((
            (DRUM_SERVICE_UUID, (
                (DRUM_CHAR_UUID, CHAR_FLAG_READ | CHAR_FLAG_NOTIFY),
            )),
        ))
        
        # Extract characteristic value handle (first tuple, first element, first item)
        self.char_handle = handles[0][0]
        
        print("BLE service registered")
        print("Characteristic handle:", self.char_handle)
        
    def _advertise(self, interval_us=500000):
        """Start BLE advertising."""
        if self.connected:
            return  # Don't advertise while connected
        
        # Advertising payload - slower interval for stability
        name = self.name.encode()
        adv_data = bytearray(b'\x02\x01\x06') + bytearray((len(name) + 1, 0x09)) + name
        
        self.ble.gap_advertise(interval_us, adv_data=adv_data)
        self._advertising = True
        print("Advertising as:", self.name)
        
    def _irq(self, event, data):
        """Handle BLE events."""
        # Connection successful
        if event == 1:  # _IRQ_CENTRAL_CONNECT
            conn_handle, _, _ = data
            self.connected = True
            self.conn_handle = conn_handle
            self._advertising = False
            # Stop advertising once connected
            try:
                self.ble.gap_advertise(None)
            except:
                pass
            print("✅ BLE connected! Handle:", conn_handle)
            
        # Disconnection
        elif event == 2:  # _IRQ_CENTRAL_DISCONNECT
            conn_handle, addr_type, addr = data
            self.connected = False
            self.conn_handle = None
            self._advertising = False
            print("❌ BLE disconnected from:", bytes(addr).hex())
            
        # MTU exchange
        elif event == 21:  # _IRQ_MTU_EXCHANGED
            print("📊 MTU exchanged:", data)
            
        # GATTS write (subscription request)
        elif event == 3:  # _IRQ_GATTS_WRITE
            conn_handle, attr_handle = data
            print("✍️ Client subscribed to notifications (handle:", attr_handle, ")")
            
        # Central subscribed to notifications
        elif event == 4:  # _IRQ_CENTRAL_SUBSCRIBE
            conn_handle, attr_handle = data
            print("🔔 Client enabled notifications (handle:", attr_handle, ")")
            
        # Central unsubscribed from notifications  
        elif event == 5:  # _IRQ_CENTRAL_UNSUBSCRIBE
            conn_handle, attr_handle = data
            print("🔕 Client disabled notifications (handle:", attr_handle, ")")
            
        # GATT complete (no-op, but common)
        elif event == 27:  # _IRQ_GATTC_*
            pass  # Ignore client GATT events
            
        # Other events
        elif event not in [28, 29, 30]:  # Ignore common client events
            print("🔔 BLE event:", event, "data:", data)
            
    def send_midi(self, status, note, velocity):
        """Send MIDI message over BLE."""
        if not self.connected or self.conn_handle is None:
            return False
            
        try:
            # Pack MIDI message (3 bytes: status, note, velocity)
            msg = struct.pack("BBB", status, note, velocity)
            
            # Send notification to connected client
            self.ble.gatts_notify(self.conn_handle, self.char_handle, msg)
            return True
        except OSError as e:
            # Connection lost during send
            print("⚠️ BLE send failed (connection lost):", e)
            self.connected = False
            self.conn_handle = None
            return False
        except Exception as e:
            print("BLE send error:", e)
            return False
        
    def send_note_on(self, note, velocity=127):
        """Send MIDI Note On message."""
        status = 0x90 | (MIDI_CHANNEL & 0x0F)
        return self.send_midi(status, note, velocity)
        
    def send_note_off(self, note):
        """Send MIDI Note Off message."""
        status = 0x80 | (MIDI_CHANNEL & 0x0F)
        return self.send_midi(status, note, 0)


# ========================================
# Power Management (prevent brownout)
# ========================================
# Disable WiFi to prevent RF interference with BLE
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
        # Reduce CPU frequency to 80MHz (default 160MHz) for power stability
        machine.freq(80000000)
        print("CPU frequency reduced to 80MHz for stability")
    except Exception as e:
        print("CPU frequency adjustment warning:", e)

# ========================================
# Initialize BLE
# ========================================
print("Initializing BLE drum controller...")
try:
    ble_drum = BLEDrum(BLE_NAME)
    # Use slower advertising for better stability (500ms instead of 100ms)
    sleep(0.1)
    print("BLE initialized successfully")
except Exception as e:
    print("BLE initialization error:", e)
    raise

# ========================================
# Initialize MPU6050 and Calibrate
# ========================================
try:
    i2c = I2C(0, scl=Pin(I2C_SCL_PIN), sda=Pin(I2C_SDA_PIN))
    sensor = MPU6050(i2c)   # Initialize MPU6050 at default address.
                            # To initialize at different address, use: MPU6050(i2c, addr=0x69)
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
print("Waiting for BLE connection...")

# ========================================
# Main Loop
# ========================================
ax_f = ay_f = az_f = 0.0
last_hit = 0
note_off_time = 0  # timestamp when Note Off should be sent
note_off_pending = False
debug_counter = 0  # For periodic debug output
connection_status_printed = False
last_advertise_check = 0  # Track when we last checked advertising status
last_keepalive = 0  # Track keepalive messages

try:
    while True:
        now = ticks_ms()

        # Re-advertise if disconnected (check every 5 seconds to avoid spam)
        if not ble_drum.connected and not ble_drum._advertising:
            if last_advertise_check == 0 or ticks_diff(now, last_advertise_check) > 5000:
                print("🔄 Re-advertising...")
                ble_drum._advertise()
                last_advertise_check = now

        # Print connection status changes
        if ble_drum.connected and not connection_status_printed:
            print("🎵 Ready to send MIDI data via BLE!")
            connection_status_printed = True
            last_keepalive = now
        elif not ble_drum.connected and connection_status_printed:
            print("⏳ Waiting for BLE connection...")
            connection_status_printed = False

        # Read sensor with error handling
        try:
            vals = sensor.raw_values()
            ax_r, ay_r, az_r, t, gx_r, gy_r, gz_r = vals

            ax = ax_r / 16384.0 - ax_bias
            ay = ay_r / 16384.0 - ay_bias
            az = az_r / 16384.0 - az_bias

            # Low-pass filter accel
            ax_f = ALPHA_ACC_LP * ax_f + (1 - ALPHA_ACC_LP) * ax
            ay_f = ALPHA_ACC_LP * ay_f + (1 - ALPHA_ACC_LP) * ay
            az_f = ALPHA_ACC_LP * az_f + (1 - ALPHA_ACC_LP) * az

            # Debug output every 40 loops (~2 seconds) - less frequent to reduce load
            if DEBUG_SENSOR and ble_drum.connected:
                debug_counter += 1
                if debug_counter >= 40:
                    print("az_f: {:.3f} (threshold: {})".format(az_f, THRESHOLD_HIT))
                    debug_counter = 0

            # Hit Detection
            if az_f < THRESHOLD_HIT and ticks_diff(now, last_hit) > DEBOUNCE_MS:
                print("*** HIT DETECTED! *** → sending MIDI note", MIDI_NOTE, "| az_f:", az_f)
                
                if ble_drum.send_note_on(MIDI_NOTE, MIDI_VELOCITY):
                    print("✅ Note On sent via BLE")
                    note_off_time = now + NOTE_OFF_DELAY_MS
                    note_off_pending = True
                else:
                    print("⚠️ Note On failed (not connected)")
                    
                last_hit = now

            # Non-blocking Note Off
            if note_off_pending and ticks_diff(now, note_off_time) >= 0:
                if ble_drum.send_note_off(MIDI_NOTE):
                    print("✅ Note Off sent via BLE")
                note_off_pending = False
                
        except OSError as e:
            print("⚠️ Sensor error:", e, "- Check MPU6050 wiring!")
            sleep(1.0)  # Wait before retrying
            # Try to reinitialize I2C
            try:
                i2c = I2C(0, scl=Pin(I2C_SCL_PIN), sda=Pin(I2C_SDA_PIN))
                sensor = MPU6050(i2c)
                print("✅ Sensor reconnected")
            except Exception:
                pass  # Will retry next loop

        # Memory management
        if gc.mem_free() < 20000:
            gc.collect()

        sleep(DT)

except KeyboardInterrupt:
    print("Stopped.")
finally:
    # Cleanup
    try:
        ble_drum.ble.active(False)
        print("BLE deactivated")
    except Exception:
        pass
