/*
 * sta_drum.ino - ESP32-C3 Air Drum (Station Mode)
 * 
 * C++/Arduino version of the MicroPython main_sta.py
 * Connects to AP (ESPDRUMS) and sends UDP MIDI packets
 * 
 * Hardware:
 *  - XIAO ESP32-C3
 *  - MPU6050 (I2C addr 0x69, SCL=GPIO7, SDA=GPIO6)
 * 
 * Upload Instructions:
 *  1. Install Arduino IDE
 *  2. Add ESP32 board support: https://docs.espressif.com/projects/arduino-esp32/en/latest/installing.html
 *  3. Install library: Adafruit MPU6050 (Library Manager)
 *  4. Select Board: "XIAO_ESP32C3"
 *  5. Upload!
 */

#include <WiFi.h>
#include <WiFiUdp.h>
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

// ========================================
// Configuration
// ========================================
const char* SSID = "ESPDRUMS";
const char* PASSWORD = "Nitirocks";
const char* SERVER_IP = "192.168.4.1";
const int SERVER_PORT = 6000;

// MIDI Settings
const uint8_t MIDI_NOTE = 40;
const uint8_t MIDI_VELOCITY = 100;
const uint8_t MIDI_CHANNEL = 9;  // Channel 10 (drums) is 9 when zero-based
const int NOTE_OFF_DELAY_MS = 50;

// MPU6050 I2C Pins (XIAO ESP32-C3)
const int I2C_SDA_PIN = 6;
const int I2C_SCL_PIN = 7;
const uint8_t MPU_ADDR = 0x69;  // STA uses 0x69

// Motion Detection
const int SAMPLES_CAL = 500;
const float DT = 0.05;  // 50ms loop time
const float ALPHA_ACC_LP = 0.5;
const float THRESHOLD_HIT = -0.3;  // More sensitive
const int DEBOUNCE_MS = 200;
const bool DEBUG_SENSOR = true;

// Auto-reconnect
const int RECONNECT_CHECK_MS = 3000;

// ========================================
// Global Variables
// ========================================
WiFiUDP udp;
Adafruit_MPU6050 mpu;
bool sensorAvailable = false;

// Calibration biases
float ax_bias = 0, ay_bias = 0, az_bias = 0;
float gx_bias = 0, gy_bias = 0, gz_bias = 0;

// Filtered values
float ax_f = 0, ay_f = 0, az_f = 0;

// Timing
unsigned long last_hit = 0;
unsigned long last_reconnect_check = 0;
unsigned long note_off_time = 0;
bool note_off_pending = false;
int debug_counter = 0;

// ========================================
// Setup WiFi
// ========================================
void setupWiFi() {
  Serial.println("Setting up WiFi...");
  
  // Disconnect if previously connected
  WiFi.disconnect(true);
  delay(1000);
  
  // Set WiFi mode to Station
  WiFi.mode(WIFI_STA);
  delay(500);
  
  // Scan for networks
  Serial.println("Scanning for WiFi networks...");
  int n = WiFi.scanNetworks();
  Serial.printf("Found %d networks:\n", n);
  
  bool apFound = false;
  for (int i = 0; i < n; i++) {
    Serial.printf("  - %s (RSSI: %d)\n", WiFi.SSID(i).c_str(), WiFi.RSSI(i));
    if (WiFi.SSID(i) == SSID) {
      apFound = true;
      Serial.println("    ^ Target AP found!");
    }
  }
  
  if (!apFound) {
    Serial.printf("WARNING: Target AP '%s' not found in scan!\n", SSID);
  }
  
  // Connect to WiFi
  Serial.printf("Connecting to AP: %s\n", SSID);
  WiFi.begin(SSID, PASSWORD);
  
  int timeout = 40;  // 20 seconds
  while (WiFi.status() != WL_CONNECTED && timeout > 0) {
    delay(500);
    Serial.print(".");
    timeout--;
  }
  Serial.println();
  
  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("Connected!");
    Serial.printf("IP: %s\n", WiFi.localIP().toString().c_str());
  } else {
    Serial.println("Connection failed!");
    Serial.printf("Status: %d\n", WiFi.status());
  }
}

// ========================================
// Check and Reconnect WiFi
// ========================================
void checkReconnect() {
  unsigned long now = millis();
  if (now - last_reconnect_check < RECONNECT_CHECK_MS) {
    return;
  }
  last_reconnect_check = now;
  
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("Connection lost! Reconnecting...");
    WiFi.disconnect();
    delay(500);
    WiFi.begin(SSID, PASSWORD);
    
    int timeout = 20;  // 10 seconds
    while (WiFi.status() != WL_CONNECTED && timeout > 0) {
      delay(500);
      timeout--;
    }
    
    if (WiFi.status() == WL_CONNECTED) {
      Serial.println("Reconnected!");
      Serial.printf("IP: %s\n", WiFi.localIP().toString().c_str());
    } else {
      Serial.println("Reconnect failed, will retry later");
    }
  }
}

// ========================================
// Send MIDI Messages
// ========================================
void sendNoteOn(uint8_t note, uint8_t velocity) {
  uint8_t msg[3];
  msg[0] = 0x90 | (MIDI_CHANNEL & 0x0F);  // Note On
  msg[1] = note;
  msg[2] = velocity;
  
  udp.beginPacket(SERVER_IP, SERVER_PORT);
  udp.write(msg, 3);
  udp.endPacket();
}

void sendNoteOff(uint8_t note) {
  uint8_t msg[3];
  msg[0] = 0x80 | (MIDI_CHANNEL & 0x0F);  // Note Off
  msg[1] = note;
  msg[2] = 0;
  
  udp.beginPacket(SERVER_IP, SERVER_PORT);
  udp.write(msg, 3);
  udp.endPacket();
}

// ========================================
// Setup MPU6050
// ========================================
void setupMPU() {
  Serial.printf("Initializing I2C on SCL=%d, SDA=%d...\n", I2C_SCL_PIN, I2C_SDA_PIN);
  
  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
  
  // Try to initialize MPU6050 at address 0x69
  if (!mpu.begin(MPU_ADDR, &Wire)) {
    Serial.println("Failed to find MPU6050 at 0x69!");
    Serial.println("Continuing without MPU6050 (no hit detection)...");
    sensorAvailable = false;
    return;
  }
  
  Serial.println("MPU6050 initialized successfully at 0x69");
  
  // Configure MPU6050
  mpu.setAccelerometerRange(MPU6050_RANGE_2_G);
  mpu.setGyroRange(MPU6050_RANGE_250_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  
  sensorAvailable = true;
  
  // Calibration
  Serial.println("Starting calibration — keep board still...");
  
  for (int i = 0; i < SAMPLES_CAL; i++) {
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);
    
    ax_bias += a.acceleration.x;
    ay_bias += a.acceleration.y;
    az_bias += a.acceleration.z;
    gx_bias += g.gyro.x;
    gy_bias += g.gyro.y;
    gz_bias += g.gyro.z;
    
    delay(10);
  }
  
  ax_bias /= SAMPLES_CAL;
  ay_bias /= SAMPLES_CAL;
  az_bias /= SAMPLES_CAL;
  gx_bias /= SAMPLES_CAL;
  gy_bias /= SAMPLES_CAL;
  gz_bias /= SAMPLES_CAL;
  
  // Remove gravity from Z axis (assuming board is horizontal)
  az_bias -= 9.81;
  
  Serial.println("Calibration complete!");
  Serial.printf("Biases - ax:%.3f, ay:%.3f, az:%.3f\n", ax_bias, ay_bias, az_bias);
}

// ========================================
// Arduino Setup
// ========================================
void setup() {
  Serial.begin(115200);
  delay(1000);
  
  Serial.println("\n\n=================================");
  Serial.println("ESP32-C3 Air Drum - Station Mode");
  Serial.println("=================================\n");
  
  // Setup WiFi
  setupWiFi();
  
  // Setup UDP
  udp.begin(SERVER_PORT);
  Serial.printf("UDP ready to send to %s:%d\n", SERVER_IP, SERVER_PORT);
  
  // Setup MPU6050
  setupMPU();
  
  Serial.println("\nRunning...\n");
}

// ========================================
// Arduino Main Loop
// ========================================
void loop() {
  unsigned long now = millis();
  
  // Auto-reconnect check
  checkReconnect();
  
  // Only proceed if connected
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("Not connected, waiting for reconnection...");
    delay(1000);
    return;
  }
  
  // Read sensor (only if available)
  if (sensorAvailable) {
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);
    
    // Apply calibration
    float ax = a.acceleration.x - ax_bias;
    float ay = a.acceleration.y - ay_bias;
    float az = a.acceleration.z - az_bias;
    
    // Low-pass filter
    ax_f = ALPHA_ACC_LP * ax_f + (1 - ALPHA_ACC_LP) * ax;
    ay_f = ALPHA_ACC_LP * ay_f + (1 - ALPHA_ACC_LP) * ay;
    az_f = ALPHA_ACC_LP * az_f + (1 - ALPHA_ACC_LP) * az;
    
    // Debug output every ~1 second
    if (DEBUG_SENSOR) {
      debug_counter++;
      if (debug_counter >= 20) {
        Serial.printf("az_f: %.3f (threshold: %.2f) | Connected: %s\n", 
                      az_f, THRESHOLD_HIT, 
                      WiFi.status() == WL_CONNECTED ? "true" : "false");
        debug_counter = 0;
      }
    }
    
    // Hit Detection
    if (az_f < THRESHOLD_HIT && (now - last_hit) > DEBOUNCE_MS) {
      if (WiFi.status() == WL_CONNECTED) {
        Serial.printf("*** STA HIT! *** → sending MIDI note %d | az_f: %.3f\n", 
                      MIDI_NOTE, az_f);
        
        sendNoteOn(MIDI_NOTE, MIDI_VELOCITY);
        Serial.printf(">>> SENT Note On to %s:%d\n", SERVER_IP, SERVER_PORT);
        
        note_off_time = now + NOTE_OFF_DELAY_MS;
        note_off_pending = true;
        last_hit = now;
      } else {
        Serial.println("Hit detected but not connected!");
      }
    }
    
    // Non-blocking Note Off
    if (note_off_pending && now >= note_off_time) {
      if (WiFi.status() == WL_CONNECTED) {
        sendNoteOff(MIDI_NOTE);
        Serial.printf(">>> SENT Note Off to %s:%d\n", SERVER_IP, SERVER_PORT);
      }
      note_off_pending = false;
    }
  }
  
  delay(DT * 1000);  // 50ms loop time
}
