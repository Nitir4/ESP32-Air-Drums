# 🥁 Arduino Firmware for ESP32 Air Drums

Welcome! This folder contains all the firmware you need to bring your wireless air drums to life. Think of these files as the "brains" for each drumstick and pedal—they tell your ESP32 what to do when you swing your arms or stomp your feet.

Don't worry if you see a lot of files here. Most of them do similar things but in slightly different ways depending on how you want to set up your drum kit.

---

## 🤔 What's in Here?

### Your Drumsticks 🥁

Think of your drumsticks as the stars of the show. You'll typically use two of them:

**The Right Stick** - Your WiFi Hub (`tcp_ap_relay.ino`)
- This is the "boss" drumstick that creates a WiFi network called "ESPDRUMS"
- It reads its own sensor to detect when you hit drums (Snare, Tom1, Tom2, and Ride)
- It also acts like a relay station, collecting MIDI signals from your left stick and pedals
- Finally, it sends everything to your computer
- **Think of it as:** The band leader who coordinates everyone

**The Left Stick** - Your WiFi Client (`tcp_drum_sta.ino`)
- This stick connects to the right stick's WiFi network
- It detects hits for Tom3, Snare, and Crash cymbal
- Sends its data to the right stick, which then forwards it to your computer
- **Think of it as:** A team player that reports to the leader

**Single Stick Mode** (`tcp_drum_direct.ino`)
- Just want to test one stick? Use this!
- Connects directly to your home WiFi and talks straight to your computer
- No relay, no fuss—great for testing

**Bluetooth Mode** (`ble_drum.ino`)
- Prefers Bluetooth over WiFi
- Saves battery but has a bit more delay (still totally playable!)
- Good if you don't want to deal with WiFi setup

---

### Your Foot Pedals 🦶

**Kick Pedal** - The Bass Drum Beat
- **WiFi Mode** (`tcp_kick_pedal_sta.ino`): Connects to your right stick's network
- **Direct Mode** (`tcp_kick_pedal_direct.ino`): Connects straight to your computer
- **Bluetooth Mode** (`ble_kick_pedal.ino`): Uses Bluetooth instead
- **What it does:** Detects when you stomp and sends a "BOOM" (MIDI note 36)

**Hi-Hat Pedal** - Open and Closed
- **WiFi Mode** (`tcp_hihat_pedal_sta.ino`): Connects to your right stick
- **Direct Mode** (`tcp_hihat_pedal_direct.ino`): Connects to your computer
- **Bluetooth Mode** (`ble_hihat_pedal.ino`): Bluetooth version
- **What it does:** Knows if you're keeping it closed or opening it for that classic "chick" sound

---

### Old Files You Can Ignore

- `ap_drum.ino` and `sta_drum.ino` - These were our early attempts. The new `tcp_ap_relay.ino` and `tcp_drum_sta.ino` are way better!

---

## 🎸 How Does Everything Work Together?

Imagine you're setting up a band. Here's how your air drums communicate:

### The Recommended Setup (WiFi Network)

```
    Left Stick                                Right Stick
   (Tom3, Snare, Crash)                  (Snare, Tom1, Tom2, Ride)
         │                                        │
         │                                   Creates WiFi
         │                                   "ESPDRUMS"
         │                                        │
         └─────────> Connects via WiFi ──────────┤
                                                  │
    Kick Pedal ──────────> Connects ─────────────┤
                                                  │
    Hi-Hat Pedal ────────> Connects ─────────────┤
                                                  │
                                           Sends everything
                                           to your computer
                                                  │
                                                  ▼
                                            Your Computer
                                         (running run.py)
```

**In plain English:**
1. Your **right stick** creates a WiFi hotspot called "ESPDRUMS" (like your phone's hotspot)
2. Your **left stick and pedals** connect to this hotspot
3. Everyone sends their drum hits to the right stick
4. The right stick forwards everything to your computer on port 6000
5. Your computer plays the drum sounds!

**Why this is cool:** Super low lag (8-25 milliseconds), which feels instant when you play!

---

### The Simple Setup (Just One Stick)

```
    One Drumstick
         │
         │ Connects to your home WiFi
         │
         ▼
    Your Computer
```

Upload `tcp_drum_direct.ino` and you're good to go. Perfect for beginners or quick tests.

---

### The Bluetooth Setup (When WiFi is a Hassle)

```
    Drumstick
         │
         │ Bluetooth connection
         │
         ▼
    Your Computer
  (running: python run.py --mode ble)
```

Uses `ble_drum.ino`. A bit slower but totally wireless and saves battery!

---

## 🔌 Wiring Your Hardware

**Don't worry, it's just 4 wires!**

Your MPU6050 sensor connects to your ESP32 like this:
- **VCC** → 3.3V pin
- **GND** → Ground pin
- **SDA** → Pin GPIO6 (labeled D4 on the board)
- **SCL** → Pin GPIO7 (labeled D5 on the board)

**Pro tip:** If you have two sticks, one sensor should have its AD0 pin connected to ground (address 0x68), and the other to 3.3V (address 0x69). This way they don't get confused when talking on the same I2C "bus."

**Battery:** Connect a 3.7V LiPo battery to the BAT+ and BAT- pads on the back of your ESP32. It'll charge automatically when you plug in USB!

---

## 📲 Uploading Firmware to Your ESP32

### The Easy Way (Arduino IDE)

**Step 1: Get Ready**
1. Download Arduino IDE from [arduino.cc](https://www.arduino.cc/en/software)
2. Open Arduino IDE
3. Go to **File → Preferences**
4. In "Additional Board Manager URLs", paste:
   ```
   https://raw.githubusercontent.com/espressif/arduino-esp32/gh-pages/package_esp32_index.json
   ```
5. Click OK

**Step 2: Install ESP32 Support**
1. Go to **Tools → Board → Boards Manager**
2. Search for "esp32"
3. Install "esp32 by Espressif Systems"
4. Wait for it to finish (grab a coffee ☕)

**Step 3: Install Libraries**
1. Go to **Sketch → Include Library → Manage Libraries**
2. Search and install these one by one:
   - **Adafruit MPU6050**
   - **Adafruit Unified Sensor**
   - **Adafruit BusIO**

**Step 4: Configure Your Board**
1. Plug in your ESP32 via USB
2. Go to **Tools → Board → ESP32 Arduino → XIAO_ESP32C3**
3. Go to **Tools → Port** → Select your COM port (might be COM3, COM4, etc.)
4. Go to **Tools → Partition Scheme → Default**

**Step 5: Upload!**
1. Open the `.ino` file you want (start with `tcp_ap_relay.ino` for your right stick)
2. Click the **Upload** button (arrow pointing right)
3. Wait for "Done uploading" message
4. Open **Tools → Serial Monitor** to see if it's working!

**First time?** Upload `tcp_ap_relay.ino` to your right stick first, then `tcp_drum_sta.ino` to your left stick!

---

## ⚙️ Tweaking Settings (Optional)

You can customize how sensitive your drums are by editing these values at the top of each `.ino` file:

**Hit Sensitivity**
```cpp
#define THRESHOLD_HIT 10.0    // Lower = more sensitive (7-15 is good)
```
If you keep accidentally triggering drums, increase this number. If it's not detecting your hits, decrease it.

**Time Between Hits**
```cpp
#define DEBOUNCE_MS 200       // Wait 200ms before detecting another hit
```
Prevents double-triggering. 200ms is usually perfect.

**Velocity (How Hard You Hit)**
```cpp
#define VELOCITY_MODE_50 true    // true = softer, false = full power
```
Set to `true` for gentle playing, `false` for rock drumming!

**WiFi Network Name (if you want to change it)**
```cpp
const char* AP_SSID = "ESPDRUMS";           // The WiFi name
const char* AP_PASSWORD = "Nitirocks";      // The password
```

---

## 🎯 What Drums Can You Hit?

### Right Stick Drums
When you swing your right stick in different directions, it knows which drum you're aiming for:

| Direction You Swing | Drum You Hit | MIDI Note |
|---------------------|--------------|-----------|
| Straight down (center) | **Snare** | 38 |
| Slightly right | **Tom 1** (mid tom) | 48 |
| Far right | **Tom 2** (floor tom) | 45 |
| Upper right | **Ride Cymbal** | 51 |

### Left Stick Drums
| Direction You Swing | Drum You Hit | MIDI Note |
|---------------------|--------------|-----------|
| Left | **Tom 3** (hi tom) | 50 |
| Straight down (center) | **Snare** | 38 |
| Upper right | **Crash Cymbal** | 49 |

**The Secret Sauce:** The firmware looks at the direction your stick is moving when you hit. Swing left? Tom 3. Swing right? Tom 1. Swing down in the center? Snare! It's like muscle memory—once you get the hang of it, it feels natural.

---

## 😓 Troubleshooting (When Things Go Wrong)

**"My sensor isn't being detected!"**
- Check your wiring! SDA should go to GPIO6, SCL to GPIO7
- Make sure the sensor has power (VCC to 3.3V, GND to GND)
- Open **Tools → Serial Monitor** and see if it says "MPU6050 not found"
- Try changing the I2C address from 0x68 to 0x69 (or vice versa)

**"My stick won't connect to WiFi!"**
- Did you upload the right stick first? It creates the "ESPDRUMS" network
- Check that the SSID and password match in both files
- Make sure your right stick's Serial Monitor says "AP IS NOW BROADCASTING!"
- Try moving the devices closer together

**"My computer won't connect to the drums!"**
- Your computer needs to connect to the "ESPDRUMS" WiFi network (yes, like you'd connect to any WiFi)
- Then run: `python run.py --mode tcp --port 6000`
- Windows users: Check your firewall isn't blocking port 6000
- Mac/Linux users: Same deal with firewall

**"The drums feel delayed / laggy"**
- Are you using the latest firmware? We fixed packet buffering issues recently
- WiFi mode should feel instant (8-25ms)
- Bluetooth will always be a bit slower (15-40ms) but still playable
- Make sure you're not using old versions with `delay()` in the code

**"I keep getting false triggers (random drum hits)"**
- Increase `THRESHOLD_HIT` from 10.0 to something like 12.0 or 15.0
- Make sure you're holding the stick firmly—loose grips can cause wobble
- Try increasing `DEBOUNCE_MS` from 200 to 250

**"It's not detecting my soft hits!"**
- Decrease `THRESHOLD_HIT` to something like 8.0 or 7.0
- Make sure the sensor is securely attached to your stick
- Check if `VELOCITY_MODE_50` is set to `true`—this might help

**"Everything was working, now it's not!"**
- Restart everything: Unplug all ESP32s, wait 5 seconds, plug them back in
- Check your battery—might need charging!
- Look at the Serial Monitor to see what error messages pop up

---

## 📊 What to Expect (Performance-wise)

| Setup | How Fast? | Reliability | Battery Life |
|-------|-----------|-------------|--------------|
| WiFi (Recommended) | Super fast (8-25ms) | Rock solid | 4-6 hours |
| Bluetooth | Pretty fast (15-40ms) | Good (occasional hiccup) | 6-8 hours |
| Direct Mode | Very fast (10-20ms) | Rock solid | 4-6 hours |

**Translation:** WiFi feels instant. Bluetooth is still great but might have the occasional tiny delay. All modes work well for jamming!

---

## 💡 Quick Tips

- **Always upload the right stick first** - It needs to create the WiFi network before anyone else can connect
- **Label your sticks!** - Mark which is Left (Tom3/Crash) and which is Right (Tom1/Tom2/Ride)
- **Calibration happens automatically** - Just hold the stick still for 5 seconds when it boots up
- **Battery low?** - The LED on the ESP32 might dim, or you'll see weird behavior
- **Want to change drums?** - Edit the MIDI note numbers in the code (they're at the top of each file)

---

## 🎵 Ready to Play?

1. Upload `tcp_ap_relay.ino` to your right stick
2. Upload `tcp_drum_sta.ino` to your left stick
3. Upload pedal firmware if you have them
4. Power everything on
5. Connect your computer to "ESPDRUMS" WiFi
6. Run `python run.py --mode tcp --port 6000`
7. Start drumming! 🥁

---

## 🔗 Other Stuff in This Project

- **Desktop App**: `../src/run.py` - The Python program that plays drum sounds
- **UI Code**: `../src/ui.py` - The visual drum kit you see on screen
- **MicroPython Version**: `../micropython/` - Same thing but written in Python for ESP32

---

**Questions? Issues? Improvements?**

Check out the main project README or open an issue on GitHub. Happy drumming! 🎸🔥
