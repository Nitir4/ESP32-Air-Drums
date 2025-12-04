/*
 * tcp_kick_pedal_direct.ino — ESP32 Kick Pedal (Direct to Desktop - Debug)
 * -------------------------------------------------------------------------
 * ESP32 WROVER KIT connects to ESPDRUMS AP but sends MIDI DIRECTLY to desktop
 * Bypasses AP relay - sends MIDI straight to desktop TCP server on port 6000
 * 
 * Architecture:
 *   KICK (WROVER) → [ESPDRUMS WiFi] → Desktop App (port 6000)
 *   (AP Relay is bypassed - no forwarding through port 5000)
 * 
 * Use this for:
 *   - Debugging kick pedal behavior
 *   - Testing without AP relay overhead
 *   - Isolating kick pedal issues
 * 
 * Hardware:
 *   - ESP32 WROVER KIT
 *   - MPU6050 on I2C: SDA=GPIO21, SCL=GPIO22
 * 
 * Upload:
 *   - Board: ESP32 Dev Module
 */

#include <WiFi.h>
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

// ========================================
// Configuration
// ========================================
const char* WIFI_SSID = "ESPDRUMS";            // AP created by tcp_ap_relay.ino
const char* WIFI_PASSWORD = "Nitirocks";       // AP password

// ⚠️ IMPORTANT: Find desktop's IP on ESPDRUMS network
// After connecting laptop to ESPDRUMS WiFi, run: ipconfig | findstr IPv4
// Desktop will have IP like 192.168.4.100 or similar (NOT 192.168.4.1)
const char* DESKTOP_IP = "192.168.4.100";      // ⚠️ Change to your laptop's ESPDRUMS IP!
const int DESKTOP_PORT = 6000;                  // Desktop app port (NOT 5000!)

// MIDI Settings
#define MIDI_NOTE 36
#define MIDI_VELOCITY 127
#define MIDI_CHANNEL 9
#define NOTE_OFF_DELAY_MS 50

// MPU6050 I2C Pins
#define I2C_SDA 21
#define I2C_SCL 22

// Pedal Detection
#define SAMPLES_CAL 500
#define LOOP_DELAY_MS 20
#define ALPHA_ACC_LP 0.3
#define THRESHOLD_PRESS -0.7
#define THRESHOLD_RELEASE -0.2
#define DEBOUNCE_MS 100
#define DEBUG_SENSOR true

#define RECONNECT_INTERVAL_MS 5000

// ========================================
// Global Variables
// ========================================
WiFiClient tcpClient;
Adafruit_MPU6050 mpu;
sensors_event_t a, g, temp;

float ax_bias = 0.0, ay_bias = 0.0, az_bias = 0.0;
float ax_f = 0.0, ay_f = 0.0, az_f = 0.0;

bool pedal_pressed = false;
unsigned long last_trigger = 0;
unsigned long note_off_time = 0;
bool note_off_pending = false;
unsigned long last_reconnect_attempt = 0;

int debug_counter = 0;

// ========================================
// WiFi Connection
// ========================================
void connectWiFi() {
  Serial.println("\n📡 Connecting to WiFi...");
  Serial.printf("   SSID: %s\n", WIFI_SSID);
  
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  
  int attempts = 0;
  while (WiFi.status() != WL_CONNECTED && attempts < 30) {
    delay(500);
    Serial.print(".");
    attempts++;
  }
  Serial.println();
  
  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("✅ WiFi Connected!");
    Serial.printf("   IP Address: %s\n", WiFi.localIP().toString().c_str());
    Serial.printf("   RSSI: %d dBm\n", WiFi.RSSI());
  } else {
    Serial.println("❌ WiFi Connection Failed!");
    Serial.println("   Check SSID and password in code!");
  }
}

// ========================================
// TCP Connection
// ========================================
bool connectToDesktop() {
  if (tcpClient.connected()) {
    return true;
  }
  
  Serial.printf("🔗 Connecting to desktop %s:%d...\n", DESKTOP_IP, DESKTOP_PORT);
  
  if (tcpClient.connect(DESKTOP_IP, DESKTOP_PORT)) {
    Serial.println("✅ TCP Connected to Desktop!");
    Serial.println("   Make sure app is running: python run.py");
    return true;
  } else {
    Serial.println("❌ TCP Connection Failed!");
    Serial.println("   Check DESKTOP_IP in code and run: python run.py");
    return false;
  }
}

// ========================================
// MIDI Functions
// ========================================
void sendMIDI(uint8_t status, uint8_t note, uint8_t velocity) {
  if (!tcpClient.connected()) return;
  
  uint8_t midiMsg[3] = {status, note, velocity};
  tcpClient.write(midiMsg, 3);
}

void sendNoteOn(uint8_t note, uint8_t velocity = 127) {
  uint8_t status = 0x90 | (MIDI_CHANNEL & 0x0F);
  sendMIDI(status, note, velocity);
}

void sendNoteOff(uint8_t note) {
  uint8_t status = 0x80 | (MIDI_CHANNEL & 0x0F);
  sendMIDI(status, note, 0);
}

// ========================================
// I2C Scanner
// ========================================
void scanI2C() {
  Serial.println("🔍 Scanning I2C bus...");
  byte error, address;
  int nDevices = 0;

  for(address = 1; address < 127; address++ ) {
    Wire.beginTransmission(address);
    error = Wire.endTransmission();

    if (error == 0) {
      Serial.printf("   ✅ I2C device found at 0x%02X\n", address);
      nDevices++;
    }
    else if (error == 4) {
      Serial.printf("   ⚠️  Unknown error at 0x%02X\n", address);
    }
  }
  
  if (nDevices == 0) {
    Serial.println("   ❌ No I2C devices found!");
    Serial.println("   Check wiring:");
    Serial.printf("      SDA -> GPIO%d\n", I2C_SDA);
    Serial.printf("      SCL -> GPIO%d\n", I2C_SCL);
  } else {
    Serial.printf("   Found %d device(s)\n", nDevices);
  }
  Serial.println();
}

// ========================================
// Setup
// ========================================
void setup() {
  Serial.begin(115200);
  delay(100);
  Serial.println("\n========================================");
  Serial.println("KICK PEDAL - DIRECT DEBUG MODE");
  Serial.println("========================================");
  Serial.println("⚠️  Make sure you configured:");
  Serial.println("   - WIFI_SSID");
  Serial.println("   - WIFI_PASSWORD");
  Serial.println("   - DESKTOP_IP");
  Serial.println("========================================\n");

  WiFi.mode(WIFI_STA);
  connectWiFi();

  Serial.println("\nInitializing MPU6050...");
  Serial.printf("   I2C Pins: SDA=GPIO%d, SCL=GPIO%d\n", I2C_SDA, I2C_SCL);
  Wire.begin(I2C_SDA, I2C_SCL);
  delay(100);
  
  // Scan I2C bus first
  scanI2C();
  
  // Try both common MPU6050 addresses
  Serial.println("Trying MPU6050 at 0x68...");
  if (!mpu.begin(0x68, &Wire)) {
    Serial.println("⚠️  Failed at 0x68, trying 0x69...");
    
    if (!mpu.begin(0x69, &Wire)) {
      Serial.println("❌ MPU6050 not found at either address!");
      Serial.println("   Check your wiring and restart.");
      while (1) delay(1000);
    } else {
      Serial.println("✅ MPU6050 found at 0x69!");
    }
  } else {
    Serial.println("✅ MPU6050 found at 0x68!");
  }
  
  Serial.println("Configuring MPU6050...");

  mpu.setAccelerometerRange(MPU6050_RANGE_2_G);
  mpu.setGyroRange(MPU6050_RANGE_250_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

  Serial.println("📊 Calibrating - KEEP FOOT OFF PEDAL...");
  for (int i = 0; i < SAMPLES_CAL; i++) {
    mpu.getEvent(&a, &g, &temp);
    ax_bias += a.acceleration.x;
    ay_bias += a.acceleration.y;
    az_bias += a.acceleration.z;
    delay(10);
  }

  ax_bias /= SAMPLES_CAL;
  ay_bias /= SAMPLES_CAL;
  az_bias /= SAMPLES_CAL;
  az_bias -= 9.81;

  Serial.printf("✅ Calibration done! az_bias=%.3f\n", az_bias);
  
  // Try immediate connection to desktop
  Serial.println("\n🔍 Connecting to desktop...");
  connectToDesktop();
  
  Serial.println("\n🎵 Ready - press pedal to trigger kick!\n");
}

// ========================================
// Main Loop
// ========================================
void loop() {
  unsigned long now = millis();

  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("⚠️  WiFi disconnected! Reconnecting...");
    connectWiFi();
    return;
  }
  
  if (!tcpClient.connected()) {
    if (now - last_reconnect_attempt > RECONNECT_INTERVAL_MS) {
      connectToDesktop();
      last_reconnect_attempt = now;
    }
  }

  mpu.getEvent(&a, &g, &temp);

  float ax = a.acceleration.x - ax_bias;
  float ay = a.acceleration.y - ay_bias;
  float az = a.acceleration.z - az_bias;

  ax_f = ALPHA_ACC_LP * ax_f + (1 - ALPHA_ACC_LP) * ax;
  ay_f = ALPHA_ACC_LP * ay_f + (1 - ALPHA_ACC_LP) * ay;
  az_f = ALPHA_ACC_LP * az_f + (1 - ALPHA_ACC_LP) * az;

  if (DEBUG_SENSOR && tcpClient.connected()) {
    debug_counter++;
    if (debug_counter >= 20) {
      Serial.printf("az_f: %.3f | State: %s | TCP: OK\n",
                    az_f, pedal_pressed ? "PRESSED" : "RELEASED");
      debug_counter = 0;
    }
  }

  if (!pedal_pressed) {
    if (az_f < THRESHOLD_PRESS && (now - last_trigger) > DEBOUNCE_MS) {
      Serial.printf("🦶 KICK! az_f=%.3f\n", az_f);
      pedal_pressed = true;

      sendNoteOn(MIDI_NOTE, MIDI_VELOCITY);
      Serial.printf("✅ Note On → Desktop (note %d)\n", MIDI_NOTE);

      note_off_time = now + NOTE_OFF_DELAY_MS;
      note_off_pending = true;
      last_trigger = now;
    }
  } else {
    if (az_f > THRESHOLD_RELEASE) {
      Serial.printf("🦶 Released! az_f=%.3f\n", az_f);
      pedal_pressed = false;
    }
  }

  if (note_off_pending && (now >= note_off_time)) {
    sendNoteOff(MIDI_NOTE);
    note_off_pending = false;
  }

  delay(LOOP_DELAY_MS);
}
