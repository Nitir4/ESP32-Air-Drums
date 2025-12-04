# ESP32 Air Drums - Dual Mode (TCP/WiFi + BLE)

Wireless air drum system using ESP32 microcontrollers with MPU6050 accelerometers. Python desktop app receives MIDI data over **TCP/WiFi** (default) or **Bluetooth Low Energy** and plays drum sounds.

## Features
- **Dual Mode**: Choose between TCP/WiFi or BLE communication
- **TCP Mode (Default)**: Fast, reliable WiFi communication
- **BLE Mode**: Battery-powered, wireless operation without WiFi infrastructure
- **Multi-device support** - handles multiple drum/pedal connections simultaneously
- **MIDI parsing** using `mido` (with fallback parser)
- **Audio engine** with real drum samples (WAV files) or synthesized sounds
- **Interactive UI** with animated drum kit visualization
- **Low latency** - suitable for live performance

## Hardware
- **ESP32-C3 XIAO** or **ESP32 WROOM** microcontrollers
- **MPU6050** 6-axis accelerometer/gyroscope sensors
- **WiFi network** (for TCP mode) or **Bluetooth 4.0+** (for BLE mode)

## Requirements
- Python 3.11+
- Install dependencies (recommended inside a venv):

```powershell
python -m venv .venv
.\.venv\Scripts\Activate.ps1
pip install -r requirements.txt
```

## Running

### TCP Mode (Default - WiFi)
Start the TCP server (listens on port 6000 by default):

```powershell
python run.py --mode tcp --host 0.0.0.0 --port 6000
```

Or simply:
```powershell
python run.py
```

### BLE Mode (Bluetooth)
Start in BLE mode to scan for drum devices:

```powershell
python run.py --mode ble --device-prefix ESPDRUMS
```

### Command-line Options
- `--mode` - Communication mode: `tcp` (WiFi) or `ble` (Bluetooth) (default: `tcp`)
- `--host` - [TCP mode] Host address to bind (default: `0.0.0.0`)
- `--port` - [TCP mode] TCP port to listen on (default: `6000`)
- `--device-prefix` - [BLE mode] BLE device name prefix to scan for (default: `ESPDRUMS`)

## Configuration

### Desktop App
- **Default TCP listen host**: `0.0.0.0`
- **Default TCP listen port**: `6000`
- **Drum samples**: Place WAV files in `samples/` directory
  - kick.wav, snare.wav, hihat_closed.wav, hihat_open.wav
  - tom_low.wav, tom_mid.wav, tom_high.wav
  - crash.wav, ride.wav

### ESP32 Firmware
Upload MicroPython or Arduino firmware to your ESP32 devices:

**MicroPython**:
```powershell
mpremote connect COM3 fs put micropython\wifi_drum.py :/main.py
mpremote connect COM3 fs put micropython\mpu6050.py :/mpu6050.py
mpremote connect COM3 reset
```

**Arduino**: Upload `.ino` files from `arduino/` directory using Arduino IDE

## Architecture

**TCP Mode (WiFi)**:
```
[ESP32 Drum 1] ─┐
[ESP32 Drum 2] ─┤
                ├─ WiFi/TCP ─→ [Desktop App: TCP Server]
[ESP32 Kick]   ─┤              ↓
[ESP32 Hi-Hat] ─┘              [Audio Engine + UI]
```

**BLE Mode (Bluetooth)**:
```
[ESP32 Drum 1] ─┐
[ESP32 Drum 2] ─┤
                ├─ BLE ─→ [Desktop App: BLE Scanner]
[ESP32 Kick]   ─┤          ↓
[ESP32 Hi-Hat] ─┘          [Audio Engine + UI]
```

## Notes
- **TCP Mode**: ESP32 devices must be on the same WiFi network as desktop app
- **BLE Mode**: No WiFi required, works on battery power
- Configure ESP32 firmware for your chosen mode (TCP or BLE)
- If `mido` is not installed, a fallback MIDI parser is used
- WAV samples are optional - synthesized beeps are used as fallback
- BLE mode requires `bleak` library (`pip install bleak`)

## License
MIT
