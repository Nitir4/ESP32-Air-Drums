/*
 * tcp_hihat_pedal_direct.ino — ESP32 Hi-Hat Pedal (AP Mode - Direct)
 * ----------------------------------------------------------------------
 * ESP32 XIAO S3 creates its own WiFi AP and sends MIDI directly to desktop
 * Desktop connects to this device's WiFi network
 * 
 * Architecture:
 *   Desktop → [HI-HAT AP WiFi] → HI-HAT Device (this ESP32)
 * 
 * Use this for:
 *   - Standalone hi-hat operation
 *   - Single device testing
 *   - No central AP needed
 * 
 * Hardware:
 *   - ESP32 XIAO S3
 *   - MPU6050 on I2C: SDA=GPIO6, SCL=GPIO7
 * 
 * Upload:
 *   - Board: ESP32S3 Dev Module (or XIAO_ESP32S3)
 */

#include <WiFi.h>
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

// ========================================
// Configuration
// ========================================
// Access Point Settings
const char* AP_SSID = "YOUR_WIFI_SSID";          // This device's AP name
const char* AP_PASSWORD = "YOUR_WIFI_PASSWORD";        // AP password
const IPAddress AP_IP(192, 168, 5, 1);        // This AP's IP
const IPAddress GATEWAY(192, 168, 5, 1);
const IPAddress SUBNET(255, 255, 255, 0);

// Desktop will auto-discover (scans 192.168.5.2 - 192.168.5.254)
const int DESKTOP_PORT = 6000;                 // Desktop app port
IPAddress desktopIP;                           // Discovered automatically
bool desktopIPFound = false;

// MIDI Settings
#define MIDI_NOTE_CLOSED 42
#define MIDI_NOTE_OPEN 46
#define MIDI_VELOCITY 100
#define MIDI_CHANNEL 9
#define NOTE_OFF_DELAY_MS 50

// MPU6050 I2C Pins
#define I2C_SDA 6
#define I2C_SCL 7

// Pedal Detection
#define SAMPLES_CAL 500
// No loop delay - run at full speed for stable TCP connection
#define ALPHA_ACC_LP 0.3
#define THRESHOLD_PRESS -0.6
#define THRESHOLD_RELEASE -0.2
#define DEBOUNCE_MS 100
#define DEBUG_SENSOR true

#define RECONNECT_INTERVAL_MS 5000

// ========================================
// Global Variables
// ========================================
WiFiClient desktopClient;                      // TCP connection to desktop
Adafruit_MPU6050 mpu;
sensors_event_t a, g, temp;

float ax_bias = 0.0, ay_bias = 0.0, az_bias = 0.0;
float ax_f = 0.0, ay_f = 0.0, az_f = 0.0;

bool pedal_closed = false;
unsigned long last_trigger = 0;
unsigned long note_off_time = 0;
bool note_off_pending = false;
uint8_t last_note = 0;
unsigned long last_desktop_reconnect = 0;

int debug_counter = 0;

// ========================================
// Setup Access Point
// ========================================
void setupAP() {
  Serial.println("========================================");
  Serial.println("Setting up Access Point...");
  
  WiFi.mode(WIFI_AP);
  delay(500);
  
  WiFi.softAPConfig(AP_IP, GATEWAY, SUBNET);
  
  bool result = WiFi.softAP(AP_SSID, AP_PASSWORD);
  
  if (result) {
    Serial.println("✅ AP activated successfully!");
    Serial.printf("   SSID: %s\n", AP_SSID);
    Serial.printf("   Password: %s\n", AP_PASSWORD);
    Serial.printf("   IP: %s\n", WiFi.softAPIP().toString().c_str());
    Serial.printf("   Stations connected: %d\n", WiFi.softAPgetStationNum());
    Serial.println("   🔊 AP IS NOW BROADCASTING!");
  } else {
    Serial.println("❌ AP activation failed!");
  }
  Serial.println("========================================");
}

// ========================================
// Auto-discover Desktop IP
// ========================================
void discoverDesktopIP() {
  // Try all possible IPs in the AP subnet (192.168.5.2 - 192.168.5.254)
  for (int ip_end = 2; ip_end < 255; ip_end++) {
    IPAddress testIP(192, 168, 5, ip_end);
    
    WiFiClient testClient;
    if (testClient.connect(testIP, DESKTOP_PORT, 500)) {
      desktopIP = testIP;
      desktopIPFound = true;
      testClient.stop();
      Serial.printf("🔍 Auto-discovered desktop at: %s\n", desktopIP.toString().c_str());
      return;
    }
    testClient.stop();
  }
}

// ========================================
// Connect to Desktop
// ========================================
bool connectToDesktop() {
  if (desktopClient.connected()) {
    return true;
  }
  
  if (!desktopIPFound) {
    discoverDesktopIP();
  }
  
  if (!desktopIPFound) {
    return false;
  }
  
  Serial.printf("🔗 Connecting to desktop %s:%d...\n", desktopIP.toString().c_str(), DESKTOP_PORT);
  
  if (desktopClient.connect(desktopIP, DESKTOP_PORT)) {
    // Disable Nagle's algorithm to prevent TCP buffering delays
    desktopClient.setNoDelay(true);
    Serial.println("✅ TCP Connected to Desktop!");
    Serial.println("   TCP_NODELAY enabled for low latency");
    return true;
  } else {
    Serial.println("❌ TCP Connection Failed!");
    desktopIPFound = false;
    return false;
  }
}

// ========================================
// MIDI Functions
// ========================================
void sendMIDI(uint8_t status, uint8_t note, uint8_t velocity) {
  if (!desktopClient.connected()) return;
  
  uint8_t midiMsg[3] = {status, note, velocity};
  desktopClient.write(midiMsg, 3);
  desktopClient.flush();  // Force immediate send, don't buffer
}

void sendNoteOn(uint8_t note, uint8_t velocity = 100) {
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
  Serial.println("HI-HAT PEDAL - AP MODE");
  Serial.println("========================================");
  Serial.println("📡 Connect your laptop WiFi to:");
  Serial.printf("   SSID: %s\n", AP_SSID);
  Serial.printf("   Password: %s\n", AP_PASSWORD);
  Serial.println("   Then run: python run.py");
  Serial.println("========================================\n");

  setupAP();

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
    
    // Try alternate address
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

  Serial.println("📊 Calibrating - KEEP FOOT OFF (OPEN)...");
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
  
  Serial.println("\n========================================");
  Serial.println("🚀 Hi-Hat Pedal Running!");
  Serial.println("========================================");
  Serial.printf("\n📊 Debug Mode: %s\n", DEBUG_SENSOR ? "ENABLED" : "DISABLED");
  Serial.printf("📡 AP: %s on %s\n", AP_SSID, WiFi.softAPIP().toString().c_str());
  Serial.printf("🎵 Closed Note: %d | Open Note: %d\n\n", MIDI_NOTE_CLOSED, MIDI_NOTE_OPEN);
  Serial.println("📡 Waiting for desktop to connect and run: python run.py");
  
  // Try immediate connection to desktop
  Serial.println("\n🔍 Attempting immediate desktop discovery...");
  discoverDesktopIP();
  if (desktopIPFound) {
    connectToDesktop();
  }
  
  Serial.println("🎩 Ready - press to close, release to open!\n");
}

// ========================================
// Main Loop
// ========================================
void loop() {
  unsigned long now = millis();

  // Reconnect to Desktop if needed
  if (!desktopClient.connected()) {
    if (now - last_desktop_reconnect > RECONNECT_INTERVAL_MS) {
      if (!desktopIPFound || !connectToDesktop()) {
        discoverDesktopIP();
        if (desktopIPFound) {
          connectToDesktop();
        }
      }
      last_desktop_reconnect = now;
    }
  }
  
  mpu.getEvent(&a, &g, &temp);

  float ax = a.acceleration.x - ax_bias;
  float ay = a.acceleration.y - ay_bias;
  float az = a.acceleration.z - az_bias;

  ax_f = ALPHA_ACC_LP * ax_f + (1 - ALPHA_ACC_LP) * ax;
  ay_f = ALPHA_ACC_LP * ay_f + (1 - ALPHA_ACC_LP) * ay;
  az_f = ALPHA_ACC_LP * az_f + (1 - ALPHA_ACC_LP) * az;

  if (DEBUG_SENSOR && desktopClient.connected()) {
    debug_counter++;
    if (debug_counter >= 20) {
      Serial.printf("az_f: %.3f | State: %s | TCP: OK\n",
                    az_f, pedal_closed ? "CLOSED" : "OPEN");
      debug_counter = 0;
    }
  }

  if (!pedal_closed) {
    if (az_f < THRESHOLD_PRESS && (now - last_trigger) > DEBOUNCE_MS) {
      Serial.printf("🎩 CLOSED! az_f=%.3f\n", az_f);
      pedal_closed = true;

      sendNoteOn(MIDI_NOTE_CLOSED, MIDI_VELOCITY);
      Serial.printf("✅ Closed Note On → Desktop (note %d)\n", MIDI_NOTE_CLOSED);

      note_off_time = now + NOTE_OFF_DELAY_MS;
      note_off_pending = true;
      last_note = MIDI_NOTE_CLOSED;
      last_trigger = now;
    }
  } else {
    if (az_f > THRESHOLD_RELEASE && (now - last_trigger) > DEBOUNCE_MS) {
      Serial.printf("🎩 OPENED! az_f=%.3f\n", az_f);
      pedal_closed = false;

      sendNoteOn(MIDI_NOTE_OPEN, MIDI_VELOCITY);
      Serial.printf("✅ Open Note On → Desktop (note %d)\n", MIDI_NOTE_OPEN);

      note_off_time = now + NOTE_OFF_DELAY_MS;
      note_off_pending = true;
      last_note = MIDI_NOTE_OPEN;
      last_trigger = now;
    }
  }

  if (note_off_pending && (now >= note_off_time)) {
    sendNoteOff(last_note);
    note_off_pending = false;
  }

  // No delay - run as fast as possible for low latency and stable TCP
}
