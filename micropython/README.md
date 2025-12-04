# MicroPython Firmware for ESP32 Air Drums

Welcome to the MicroPython side of things. If you prefer Python over C/C++, or just want to prototype faster, this is the place for you. These files do the exact same thing as the Arduino firmware, but they're written in Python and run on MicroPython (a tiny version of Python made for microcontrollers).

**Why MicroPython?**
- Write code in Python instead of C
- Upload new code in seconds (no compiling!)
- Easy to tweak and experiment
- Interactive REPL for debugging

**Trade-offs:**
- Slightly slower than C (but still fast enough for drums!)
- Uses more memory
- Slightly higher power consumption

---

## What's in Here?

### Drumstick Firmware

**Bluetooth Mode (Recommended for Python)**
- **`ble_drum.py`** - Your main drumstick firmware
  - Sends MIDI over Bluetooth
  - Works with desktop app in BLE mode
  - Battery-friendly and wireless
  - Just upload and it works!

- **`ble_drum_relay.py`** - Advanced relay setup
  - One drumstick acts as BLE relay for others
  - Good for multi-device setups
  - Experimental feature

**WiFi/UDP Mode (Classic)**
- **`ap_stable.py`** - Access Point version
  - Creates "ESPDRUMS" WiFi network
  - Receives data from other ESP32s via UDP
  - Also reads its own sensor
  - Like `tcp_ap_relay.ino` but in Python

- **`main_sta.py`** - Station version
  - Connects to "ESPDRUMS" network
  - Sends sensor data via UDP
  - Like `tcp_drum_sta.ino` but in Python

- **`sta_example.py`** - Learning template
  - Simplified station mode code
  - Great for understanding how things work
  - Start here if you're new to MicroPython

---

### Foot Pedal Firmware

**Kick Pedal (Bass Drum)**
- **`ble_kick_pedal.py`** - Bluetooth kick pedal
  - Sends MIDI note 36 when you stomp
  - Super simple, super wireless
  - Battery lasts for hours

**Hi-Hat Pedal**
- **`ble_hihat_pedal.py`** - Bluetooth hi-hat
  - Detects open (46) vs closed (42) positions
  - Smooth pedal response
  - Just like the real thing!

---

### Utility Scripts

- **`ota_ap.py`** - Over-The-Air updates
  - Update your firmware wirelessly
  - No USB cable needed once set up
  - Advanced feature for later

- **`test_mpu6050_address.py`** - I2C scanner
  - Finds your MPU6050 sensor
  - Shows if it's at 0x68 or 0x69
  - Use this when troubleshooting wiring

---

## Getting Started

### Step 1: Install MicroPython on Your ESP32

**First time? You need to flash MicroPython firmware:**

1. Download the ESP32-C3 firmware from [micropython.org](https://micropython.org/download/ESP32_GENERIC_C3/)
   - Get the latest `.bin` file

2. Install `esptool`:
   ```bash
   pip install esptool
   ```

3. Erase the flash (this wipes everything):
   ```bash
   esptool.py --chip esp32c3 --port COM3 erase_flash
   ```
   *Change `COM3` to your port (might be COM4, COM5, etc.)*

4. Flash MicroPython:
   ```bash
   esptool.py --chip esp32c3 --port COM3 write_flash -z 0x0 ESP32_GENERIC_C3-20240602-v1.23.0.bin
   ```
   *Use your actual firmware filename*

5. Success! Your ESP32 now runs Python

---

### Step 2: Install Upload Tools

You need a tool to copy files to your ESP32. Pick one:

**Option A: mpremote (Recommended)**
```bash
pip install mpremote
```

**Option B: ampy**
```bash
pip install adafruit-ampy
```

**Option C: Thonny IDE (Easiest for beginners)**
- Download from [thonny.org](https://thonny.org/)
- Built-in file manager for ESP32
- Great for learning!

---

### Step 3: Upload Your Code

**Using mpremote:**
```bash
# Upload drumstick BLE firmware
mpremote connect COM3 fs put ble_drum.py :/main.py
mpremote connect COM3 fs put mpu6050.py :/mpu6050.py
mpremote connect COM3 reset
```

**Using ampy:**
```bash
ampy --port COM3 put ble_drum.py main.py
ampy --port COM3 put mpu6050.py mpu6050.py
```

**Using Thonny:**
1. Open Thonny
2. Select **Tools → Options → Interpreter**
3. Choose "MicroPython (ESP32)"
4. Select your COM port
5. Right-click files in the editor → **Upload to /**
6. Rename `ble_drum.py` to `main.py` on the device

**Pro tip:** The file named `main.py` runs automatically when your ESP32 boots!

---

## Wiring (Same as Arduino)

Your MPU6050 sensor connects like this:
- **VCC** → 3.3V
- **GND** → GND
- **SDA** → GPIO6
- **SCL** → GPIO7

**I2C Addresses:**
- First sensor: 0x68 (AD0 pin to GND)
- Second sensor: 0x69 (AD0 pin to 3.3V)

---

## Configuration

Each `.py` file has a config section at the top. Here's what you can change:

### Bluetooth Settings (ble_drum.py)
```python
BLE_DEVICE_NAME = "ESPDRUMS_1"    # Change the name
MIDI_CHANNEL = 9                   # MIDI channel (usually 9 for drums)
```

### WiFi Settings (ap_stable.py)
```python
AP_SSID = "ESPDRUMS"
AP_PASSWORD = "Nitirocks"
```

### Hit Sensitivity
```python
THRESHOLD_HIT = 15.0    # Lower = more sensitive (10-20 is good)
DEBOUNCE_MS = 200       # Wait time between hits
```

### Velocity Modes
```python
VELOCITY_MODE_50 = True    # True = softer, False = harder hits
```

---

## Which File Should You Use?

**For a single drumstick (easiest):**
- Upload `ble_drum.py` as `main.py`
- Run desktop app: `python run.py --mode ble`
- Done!

**For two drumsticks (recommended):**
- Upload `ble_drum.py` to both sticks
- Change `BLE_DEVICE_NAME` to "ESPDRUMS_1" and "ESPDRUMS_2"
- Desktop app will find both automatically

**For kick pedal:**
- Upload `ble_kick_pedal.py` as `main.py`
- Name it "ESPDRUMS_KICK"

**For hi-hat pedal:**
- Upload `ble_hihat_pedal.py` as `main.py`
- Name it "ESPDRUMS_HIHAT"

**For WiFi network setup:**
- Right stick: Upload `ap_stable.py` as `main.py`
- Left stick: Upload `main_sta.py` as `main.py`
- More complex but lower latency

---

## Troubleshooting

**"My computer can't find the ESP32!"**
- Make sure you installed the USB driver (CH340 or similar)
- Try a different USB cable (data cable, not charge-only)
- Press and hold the BOOT button while plugging in USB
- Check Device Manager (Windows) or `ls /dev/tty*` (Mac/Linux)

**"MicroPython won't flash!"**
- Hold BOOT button while running `esptool.py erase_flash`
- Try `--baud 115200` if default speed fails
- Use a USB 2.0 port (USB 3.0 sometimes causes issues)

**"MPU6050 sensor not found!"**
- Run `test_mpu6050_address.py` to scan for sensors
- Check your wiring carefully
- Try swapping SDA and SCL (easy to mix up!)
- Make sure sensor has power (3.3V, not 5V!)

**"Code uploaded but nothing happens!"**
- Did you name it `main.py`? (not `ble_drum.py`)
- Open the REPL (serial terminal) to see error messages:
  ```bash
  mpremote connect COM3 repl
  ```
- Press Ctrl+D to soft-reset and watch for errors

**"Bluetooth won't connect!"**
- Desktop app must run with `--mode ble`
- Check that `BLE_DEVICE_NAME` matches in code and app
- Try restarting the ESP32 (unplug and replug)
- Some laptops have flaky Bluetooth—try getting closer

**"Hits feel delayed or laggy!"**
- BLE is naturally slower than WiFi (15-40ms vs 8-15ms)
- This is normal and still totally playable
- For lowest latency, use Arduino WiFi firmware instead

---

## Interactive Debugging (REPL)

One of the coolest things about MicroPython is the **REPL** (Read-Eval-Print Loop). It's like a Python console running on your ESP32!

**Connect to REPL:**
```bash
mpremote connect COM3 repl
```

**Now you can type Python commands live:**
```python
>>> from machine import Pin
>>> led = Pin(2, Pin.OUT)
>>> led.on()    # Turn on LED!
>>> led.off()   # Turn it off!

>>> from mpu6050 import MPU6050
>>> from machine import I2C
>>> i2c = I2C(0, scl=Pin(7), sda=Pin(6))
>>> mpu = MPU6050(i2c)
>>> mpu.accel    # Read current acceleration!
(0.05, -0.12, 9.81)
```

**This is perfect for:**
- Testing sensors before writing full code
- Debugging why something isn't working
- Learning how MicroPython works

**Exit REPL:** Press `Ctrl+]`

---

## Performance Comparison

| Feature | MicroPython | Arduino C |
|---------|-------------|-----------|
| Upload Speed | Seconds | ~30 seconds |
| Latency | 15-40ms (BLE) | 8-25ms (WiFi) |
| Memory Usage | Higher | Lower |
| Ease of Use | Very easy | Need C knowledge |
| Debugging | REPL is amazing | Serial print only |
| Battery Life | Good | Better |

**Bottom line:** MicroPython is great for learning, prototyping, and having fun. Arduino is better for final products and lowest latency.

---

## Quick Recipes

### Recipe 1: Single BLE Drumstick
```bash
# Flash MicroPython (first time only)
esptool.py --chip esp32c3 --port COM3 erase_flash
esptool.py --chip esp32c3 --port COM3 write_flash -z 0x0 firmware.bin

# Upload code
mpremote connect COM3 fs put ble_drum.py :/main.py
mpremote connect COM3 fs put mpu6050.py :/mpu6050.py
mpremote connect COM3 reset

# Run desktop app
python run.py --mode ble --device-prefix ESPDRUMS
```

### Recipe 2: Two BLE Drumsticks
```bash
# Stick 1
mpremote connect COM3 fs put ble_drum.py :/main.py
mpremote connect COM3 fs put mpu6050.py :/mpu6050.py
# Edit main.py and change: BLE_DEVICE_NAME = "ESPDRUMS_L"
mpremote connect COM3 reset

# Stick 2 (plug into different port)
mpremote connect COM4 fs put ble_drum.py :/main.py
mpremote connect COM4 fs put mpu6050.py :/mpu6050.py
# Edit main.py and change: BLE_DEVICE_NAME = "ESPDRUMS_R"
mpremote connect COM4 reset
```

### Recipe 3: Check Your Sensor
```bash
mpremote connect COM3 fs put test_mpu6050_address.py :/main.py
mpremote connect COM3 reset
mpremote connect COM3 repl
# Watch the output!
```

---

## Where's the `mpu6050.py` Library?

You need a MicroPython MPU6050 library! Here are some options:

**Option 1: Use the one in this project** (if it exists)
```bash
mpremote connect COM3 fs put mpu6050.py
```

**Option 2: Download from GitHub**
```bash
# From: https://github.com/adamjezek98/MPU6050-ESP8266-MicroPython
curl -o mpu6050.py https://raw.githubusercontent.com/adamjezek98/MPU6050-ESP8266-MicroPython/master/mpu6050.py
mpremote connect COM3 fs put mpu6050.py
```

**Option 3: Use `mip` (MicroPython package manager)**
```bash
mpremote connect COM3 mip install github:adamjezek98/MPU6050-ESP8266-MicroPython
```

---

## Tips for Success

- **Start simple:** Get one stick working with `ble_drum.py` before doing fancy stuff
- **Use the REPL:** It's your best friend for debugging
- **Name your devices:** Different `BLE_DEVICE_NAME` for each stick/pedal
- **Check the Serial Monitor:** When things go wrong, errors show up here
- **Battery indicator:** Monitor voltage with `machine.ADC` if you're curious
- **Have fun!** MicroPython makes experimenting easy—break things and learn!

---

## Learn More

- **MicroPython Docs:** [docs.micropython.org](https://docs.micropython.org/)
- **ESP32 Quick Reference:** [docs.micropython.org/en/latest/esp32/quickref.html](https://docs.micropython.org/en/latest/esp32/quickref.html)
- **MPU6050 Datasheet:** Understanding your sensor helps!

---

## Related Files

- **Arduino Version:** `../arduino/` - C/C++ firmware (faster but harder)
- **Desktop App:** `../src/run.py` - The Python program that plays sounds
- **Main README:** `../README.md` - Project overview

---

**Happy Python Drumming!**

Remember: The best code is code that works for you. Don't be afraid to experiment and make it your own.
