/*
 * tcp_hihat_pedal_sta.ino — ESP32 Hi-Hat Pedal (TCP Station Mode)
 * ----------------------------------------------------------------
 * ESP32 XIAO S3 acting as WiFi Station (STA)
 * Connects to AP (ESPDRUMS_1) and forwards MIDI to it via TCP
 * 
 * Architecture:
 *   HI-HAT (XIAO S3) → [WiFi] → AP (ESPDRUMS_1) → [WiFi] → Desktop
 * 
 * Hardware:
 *   - ESP32 XIAO S3
 *   - MPU6050 on I2C: SDA=GPIO6, SCL=GPIO7
 * 
 * Upload:
 *   - Board: ESP32S3 Dev Module (or XIAO_ESP32S3)
 *   - Partition Scheme: Default
 */

#include <WiFi.h>
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

// ========================================
// Configuration
// ========================================
// WiFi Settings
const char* WIFI_SSID = "YOUR_WIFI_SSID";
const char* WIFI_PASSWORD = "YOUR_WIFI_PASSWORD";
const char* AP_SERVER_IP = "192.168.4.1";
const int AP_SERVER_PORT = 5000;

// MIDI Settings - Hi-Hat (simplified to single note)
#define MIDI_NOTE 46  // Using open hi-hat sound for all triggers
#define MIDI_VELOCITY 100
#define MIDI_CHANNEL 9
#define NOTE_OFF_DELAY_MS 50

// MPU6050 I2C Pins (ESP32 XIAO S3)
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

// Reconnection
#define RECONNECT_INTERVAL_MS 5000

// ========================================
// Global Variables
// ========================================
WiFiClient tcpClient;
Adafruit_MPU6050 mpu;
sensors_event_t a, g, temp;

// Calibration
float ax_bias = 0.0, ay_bias = 0.0, az_bias = 0.0;

// Filtering
float ax_f = 0.0, ay_f = 0.0, az_f = 0.0;

// State Machine
bool pedal_closed = false;
unsigned long last_trigger = 0;
unsigned long note_off_time = 0;
bool note_off_pending = false;
unsigned long last_reconnect_attempt = 0;

// Debug
int debug_counter = 0;

// ========================================
// WiFi Connection
// ========================================
void connectWiFi() {
  Serial.println("\n🔍 Scanning for WiFi networks...");
  int n = WiFi.scanNetworks();
  Serial.printf("Found %d networks\n", n);
  
  bool apFound = false;
  for (int i = 0; i < n; i++) {
    Serial.printf("  - %s (RSSI: %d)\n", WiFi.SSID(i).c_str(), WiFi.RSSI(i));
    if (WiFi.SSID(i) == WIFI_SSID) {
      apFound = true;
    }
  }
  
  if (!apFound) {
    Serial.printf("⚠️  AP '%s' not found in scan!\n", WIFI_SSID);
  }
  
  Serial.printf("\n📡 Connecting to WiFi: %s\n", WIFI_SSID);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  
  int attempts = 0;
  while (WiFi.status() != WL_CONNECTED && attempts < 20) {
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
  }
}

// ========================================
// TCP Connection to AP
// ========================================
bool connectToAP() {
  if (tcpClient.connected()) {
    return true;
  }
  
  Serial.printf("🔗 Connecting to AP server %s:%d...\n", AP_SERVER_IP, AP_SERVER_PORT);
  
  if (tcpClient.connect(AP_SERVER_IP, AP_SERVER_PORT)) {
    // Disable Nagle's algorithm to prevent TCP buffering delays
    tcpClient.setNoDelay(true);
    Serial.println("✅ TCP Connected to AP!");
    Serial.println("   TCP_NODELAY enabled for low latency");
    return true;
  } else {
    Serial.println("❌ TCP Connection to AP Failed!");
    return false;
  }
}

// ========================================
// MIDI Functions
// ========================================
void sendMIDI(uint8_t status, uint8_t note, uint8_t velocity) {
  if (!tcpClient.connected()) {
    return;
  }
  
  uint8_t midiMsg[3] = {status, note, velocity};
  tcpClient.write(midiMsg, 3);
  tcpClient.flush();  // Force immediate send, don't buffer
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
  Serial.println("ESP32 HI-HAT PEDAL - TCP Station Mode");
  Serial.println("========================================");

  // Setup WiFi
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  delay(100);
  
  connectWiFi();

  // Initialize I2C
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

  // Calibration
  Serial.println("📊 Calibrating - KEEP FOOT OFF PEDAL (OPEN)...");
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
  
  // Connect to AP
  connectToAP();
  
  Serial.println("========================================");
  Serial.println("🎵 Ready - press to close, release to open!");
  Serial.println("========================================\n");
}

// ========================================
// Main Loop
// ========================================
void loop() {
  unsigned long now = millis();

  // Check WiFi and TCP connection
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("⚠️  WiFi disconnected! Reconnecting...");
    connectWiFi();
    return;
  }
  
  if (!tcpClient.connected()) {
    if (now - last_reconnect_attempt > RECONNECT_INTERVAL_MS) {
      Serial.println("⚠️  TCP disconnected! Reconnecting...");
      connectToAP();
      last_reconnect_attempt = now;
    }
  }

  // Read sensor
  mpu.getEvent(&a, &g, &temp);

  float ax = a.acceleration.x - ax_bias;
  float ay = a.acceleration.y - ay_bias;
  float az = a.acceleration.z - az_bias;

  // Low-pass filter
  ax_f = ALPHA_ACC_LP * ax_f + (1 - ALPHA_ACC_LP) * ax;
  ay_f = ALPHA_ACC_LP * ay_f + (1 - ALPHA_ACC_LP) * ay;
  az_f = ALPHA_ACC_LP * az_f + (1 - ALPHA_ACC_LP) * az;

  // Debug output
  if (DEBUG_SENSOR && tcpClient.connected()) {
    debug_counter++;
    if (debug_counter >= 20) {
      Serial.printf("az_f: %.3f | State: %s | TCP: Connected\n",
                    az_f,
                    pedal_closed ? "CLOSED" : "OPEN");
      debug_counter = 0;
    }
  }

  // Trigger on pedal press (like kick pedal - simplified logic)
  if (!pedal_closed) {
    if (az_f < THRESHOLD_PRESS && (now - last_trigger) > DEBOUNCE_MS) {
      Serial.printf("🎩 HI-HAT PRESS! az_f=%.3f\n", az_f);
      pedal_closed = true;

      sendNoteOn(MIDI_NOTE, MIDI_VELOCITY);
      Serial.printf("✅ Hi-Hat Note On sent to AP (note %d)\n", MIDI_NOTE);

      note_off_time = now + NOTE_OFF_DELAY_MS;
      note_off_pending = true;
      last_trigger = now;
    }
  } else {
    // Release detection for state reset
    if (az_f > THRESHOLD_RELEASE) {
      Serial.printf("🎩 Released! az_f=%.3f\n", az_f);
      pedal_closed = false;
    }
  }

  // Non-blocking Note Off
  if (note_off_pending && (now >= note_off_time)) {
    sendNoteOff(MIDI_NOTE);
    note_off_pending = false;
  }

  // No delay - run as fast as possible for low latency and stable TCP
}
