"""
ble_hihat_pedal.py — ESP32 Hi-Hat Pedal (BLE)
----------------------------------------------
Detects hi-hat pedal position using MPU6050.
Sends CLOSED note when pressed, OPEN note when released.

Upload to ESP32:
    mpremote connect COM5 fs put ble_hihat_pedal.py :/main.py
    mpremote connect COM5 fs put mpu6050.py :/mpu6050.py
    mpremote connect COM5 reset
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
BLE_NAME = "ESPDRUMS_HIHAT"

# MIDI settings - Hi-Hat has TWO sounds!
MIDI_NOTE_CLOSED = 42  # Closed Hi-Hat (foot down, tight sound)
MIDI_NOTE_OPEN = 46    # Open Hi-Hat (foot up, ringing sound)
MIDI_VELOCITY = 100
MIDI_CHANNEL = 9
NOTE_OFF_DELAY_MS = 50

# MPU6050 I2C pins (adjust for your ESP32 WROOM)
I2C_SCL_PIN = 22  # GPIO22 (default SCL)
I2C_SDA_PIN = 21  # GPIO21 (default SDA)
# For ESP32-C3: use 7, 6

# Pedal detection
SAMPLES_CAL = 500
DT = 0.02               # 20ms loop
ALPHA_ACC_LP = 0.3      # fast response
THRESHOLD_PRESS = -0.6  # foot pressing down
THRESHOLD_RELEASE = -0.2  # foot lifted up
DEBOUNCE_MS = 100       # fast response for hi-hat
DEBUG_SENSOR = True

# Power management
ENABLE_POWER_SAVE = True

# BLE UUIDs
DRUM_SERVICE_UUID = bluetooth.UUID("4fafc201-1fb5-459e-8fcc-c5c9c331914b")
DRUM_CHAR_UUID = bluetooth.UUID("beb5483e-36e1-4688-b7f5-ea07361b26a8")
# ========================================


class BLEDrum:
    """BLE MIDI drum peripheral."""
    
    def __init__(self, name):
        self.name = name
        self.ble = bluetooth.BLE()
        self.ble.active(True)
        self.ble.irq(self._irq)
        self.connected = False
        self.conn_handle = None
        self._advertising = False
        
        self._register_services()
        self._advertise()
        
    def _register_services(self):
        CHAR_FLAG_READ = 0x0002
        CHAR_FLAG_NOTIFY = 0x0010
        
        handles = self.ble.gatts_register_services((
            (DRUM_SERVICE_UUID, (
                (DRUM_CHAR_UUID, CHAR_FLAG_READ | CHAR_FLAG_NOTIFY),
            )),
        ))
        
        self.char_handle = handles[0][0]
        print("BLE service registered, handle:", self.char_handle)
        
    def _advertise(self, interval_us=500000):
        if self.connected:
            return
        
        name = self.name.encode()
        adv_data = bytearray(b'\x02\x01\x06') + bytearray((len(name) + 1, 0x09)) + name
        
        self.ble.gap_advertise(interval_us, adv_data=adv_data)
        self._advertising = True
        print("Advertising as:", self.name)
        
    def _irq(self, event, data):
        if event == 1:  # _IRQ_CENTRAL_CONNECT
            conn_handle, _, _ = data
            self.connected = True
            self.conn_handle = conn_handle
            self._advertising = False
            try:
                self.ble.gap_advertise(None)
            except:
                pass
            print("✅ BLE connected! Handle:", conn_handle)
            
        elif event == 2:  # _IRQ_CENTRAL_DISCONNECT
            self.connected = False
            self.conn_handle = None
            self._advertising = False
            print("❌ BLE disconnected")
            
        elif event == 21:  # _IRQ_MTU_EXCHANGED
            print("📊 MTU exchanged:", data)
            
        elif event == 4:  # _IRQ_CENTRAL_SUBSCRIBE
            print("🔔 Client subscribed to notifications")
            
    def send_midi(self, status, note, velocity):
        if not self.connected or self.conn_handle is None:
            return False
            
        try:
            msg = struct.pack("BBB", status, note, velocity)
            self.ble.gatts_notify(self.conn_handle, self.char_handle, msg)
            return True
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
        print("CPU frequency reduced to 80MHz")
    except Exception as e:
        print("CPU warning:", e)

# ========================================
# Initialize BLE
# ========================================
print("Initializing BLE hi-hat pedal...")
ble_drum = BLEDrum(BLE_NAME)
sleep(0.1)
print("BLE initialized")

# ========================================
# Initialize MPU6050
# ========================================
print(f"Initializing MPU6050 on SDA={I2C_SDA_PIN}, SCL={I2C_SCL_PIN}...")
try:
    i2c = I2C(0, scl=Pin(I2C_SCL_PIN), sda=Pin(I2C_SDA_PIN))
    sensor = MPU6050(i2c)
    print("MPU6050 initialized")
except Exception as e:
    print("MPU6050 error:", e)
    raise

# Calibration - assumes pedal is at REST (foot up = OPEN hi-hat)
print("Calibrating - KEEP FOOT OFF PEDAL (OPEN position)...")
ax_bias = ay_bias = az_bias = 0.0

for i in range(SAMPLES_CAL):
    vals = sensor.raw_values()
    ax, ay, az = vals[0], vals[1], vals[2]
    ax_bias += ax / 16384.0
    ay_bias += ay / 16384.0
    az_bias += az / 16384.0
    sleep(0.01)

ax_bias /= SAMPLES_CAL
ay_bias /= SAMPLES_CAL
az_bias /= SAMPLES_CAL
az_bias -= 1.0  # gravity correction

print(f"Calibration done! Open position: az_bias={az_bias:.3f}")
print("Ready - press pedal to close hi-hat!")

# ========================================
# Main Loop - Hi-Hat State Machine
# ========================================
ax_f = ay_f = az_f = 0.0
pedal_closed = False  # Track hi-hat state (False=OPEN, True=CLOSED)
last_trigger = 0
note_off_time = 0
note_off_pending = False
last_note = None  # Track which note was last sent
debug_counter = 0
last_advertise_check = 0

try:
    while True:
        now = ticks_ms()

        # Re-advertise if disconnected
        if not ble_drum.connected and not ble_drum._advertising:
            if ticks_diff(now, last_advertise_check) > 5000:
                ble_drum._advertise()
                last_advertise_check = now

        # Read sensor
        try:
            vals = sensor.raw_values()
            ax = vals[0] / 16384.0 - ax_bias
            ay = vals[1] / 16384.0 - ay_bias
            az = vals[2] / 16384.0 - az_bias

            # Low-pass filter
            ax_f = ALPHA_ACC_LP * ax_f + (1 - ALPHA_ACC_LP) * ax
            ay_f = ALPHA_ACC_LP * ay_f + (1 - ALPHA_ACC_LP) * ay
            az_f = ALPHA_ACC_LP * az_f + (1 - ALPHA_ACC_LP) * az

            # Debug output
            if DEBUG_SENSOR and ble_drum.connected:
                debug_counter += 1
                if debug_counter >= 20:  # Every ~400ms
                    state = "CLOSED" if pedal_closed else "OPEN"
                    print(f"az_f: {az_f:.3f} | State: {state} | Press: {THRESHOLD_PRESS}")
                    debug_counter = 0

            # STATE MACHINE: Detect CLOSE and OPEN transitions
            if not pedal_closed:
                # Hi-hat is OPEN, waiting for CLOSE (foot pressing down)
                if az_f < THRESHOLD_PRESS and ticks_diff(now, last_trigger) > DEBOUNCE_MS:
                    print(f"🎩 HI-HAT CLOSED! az_f={az_f:.3f}")
                    pedal_closed = True
                    
                    if ble_drum.send_note_on(MIDI_NOTE_CLOSED, MIDI_VELOCITY):
                        print(f"✅ Closed Hi-Hat Note On sent (note {MIDI_NOTE_CLOSED})")
                        note_off_time = now + NOTE_OFF_DELAY_MS
                        note_off_pending = True
                        last_note = MIDI_NOTE_CLOSED
                    
                    last_trigger = now
                    
            else:
                # Hi-hat is CLOSED, waiting for OPEN (foot lifted)
                if az_f > THRESHOLD_RELEASE and ticks_diff(now, last_trigger) > DEBOUNCE_MS:
                    print(f"🎩 HI-HAT OPENED! az_f={az_f:.3f}")
                    pedal_closed = False
                    
                    if ble_drum.send_note_on(MIDI_NOTE_OPEN, MIDI_VELOCITY):
                        print(f"✅ Open Hi-Hat Note On sent (note {MIDI_NOTE_OPEN})")
                        note_off_time = now + NOTE_OFF_DELAY_MS
                        note_off_pending = True
                        last_note = MIDI_NOTE_OPEN
                    
                    last_trigger = now

            # Non-blocking Note Off
            if note_off_pending and ticks_diff(now, note_off_time) >= 0:
                if last_note and ble_drum.send_note_off(last_note):
                    print(f"✅ Note Off sent (note {last_note})")
                note_off_pending = False
                
        except OSError as e:
            print("⚠️ Sensor error:", e)
            sleep(1.0)
            try:
                i2c = I2C(0, scl=Pin(I2C_SCL_PIN), sda=Pin(I2C_SDA_PIN))
                sensor = MPU6050(i2c)
                print("✅ Sensor reconnected")
            except:
                pass

        # Memory management
        if gc.mem_free() < 20000:
            gc.collect()

        sleep(DT)

except KeyboardInterrupt:
    print("Stopped.")
finally:
    try:
        ble_drum.ble.active(False)
        print("BLE deactivated")
    except:
        pass
