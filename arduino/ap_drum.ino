/*
 * ap_drum.ino - ESP32-C3 Air Drum (Access Point Mode)
 * 
 * C++/Arduino version of the MicroPython ap_stable.py
 * Hosts AP (ESPDRUMS), receives UDP MIDI, and forwards to broadcast
 * 
 * Hardware:
 *  - XIAO ESP32-C3
 *  - MPU6050 (I2C addr 0x68, SCL=GPIO7, SDA=GPIO6)
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
const char* AP_IP_STR = "192.168.4.1";
const IPAddress AP_IP(192, 168, 4, 1);
const IPAddress GATEWAY(192, 168, 4, 1);
const IPAddress SUBNET(255, 255, 255, 0);
const IPAddress BROADCAST(192, 168, 4, 255);
const int SERVER_PORT = 6000;

// MIDI Settings
const uint8_t MIDI_NOTE = 38;  // Different from STA (snare)
const uint8_t MIDI_VELOCITY = 100;
const uint8_t MIDI_CHANNEL = 9;
const int NOTE_OFF_DELAY_MS = 50;

// MPU6050 I2C Pins
const int I2C_SDA_PIN = 6;
const int I2C_SCL_PIN = 7;
const uint8_t MPU_ADDR = 0x68;  // AP uses 0x68

// Motion Detection
const int SAMPLES_CAL = 500;
const float DT = 0.05;
const float ALPHA_ACC_LP = 0.5;
const float THRESHOLD_HIT = -0.3;
const int DEBOUNCE_MS = 200;
const bool DEBUG_SENSOR = true;

// Forward MIDI to PC
const bool FORWARD_TO_PC = true;

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
unsigned long note_off_time = 0;
bool note_off_pending = false;
int debug_counter = 0;

// ========================================
// Setup Access Point
// ========================================
void setupAP() {
  Serial.println("Setting up Access Point...");
  
  // Disconnect WiFi if connected
  WiFi.disconnect(true);
  delay(1000);
  
  // Configure AP
  WiFi.mode(WIFI_AP);
  delay(500);
  
  WiFi.softAPConfig(AP_IP, GATEWAY, SUBNET);
  
  bool result = WiFi.softAP(SSID, PASSWORD);
  
  if (result) {
    Serial.println("AP activated successfully!");
    Serial.printf("SSID: %s\n", SSID);
    Serial.printf("Password: %s\n", PASSWORD);
    Serial.printf("IP: %s\n", WiFi.softAPIP().toString().c_str());
    Serial.println("Devices can now connect!");
  } else {
    Serial.println("AP activation failed!");
  }
}

// ========================================
// Send MIDI Messages
// ========================================
void sendNoteOn(uint8_t note, uint8_t velocity) {
  uint8_t msg[3];
  msg[0] = 0x90 | (MIDI_CHANNEL & 0x0F);
  msg[1] = note;
  msg[2] = velocity;
  
  udp.beginPacket(BROADCAST, SERVER_PORT);
  udp.write(msg, 3);
  udp.endPacket();
}

void sendNoteOff(uint8_t note) {
  uint8_t msg[3];
  msg[0] = 0x80 | (MIDI_CHANNEL & 0x0F);
  msg[1] = note;
  msg[2] = 0;
  
  udp.beginPacket(BROADCAST, SERVER_PORT);
  udp.write(msg, 3);
  udp.endPacket();
}

// ========================================
// Forward received MIDI
// ========================================
void forwardMIDI(uint8_t* data, int len, IPAddress remoteIP, uint16_t remotePort) {
  if (len >= 3 && FORWARD_TO_PC) {
    uint8_t status = data[0];
    uint8_t note = data[1];
    uint8_t velocity = data[2];
    
    String msgType = ((status & 0xF0) == 0x90) ? "Note On" : "Note Off";
    
    Serial.printf("<<< RECEIVED from %s:%d: %s - Note=%d, Vel=%d\n",
                  remoteIP.toString().c_str(), remotePort, 
                  msgType.c_str(), note, velocity);
    
    // Forward to broadcast
    udp.beginPacket(BROADCAST, SERVER_PORT);
    udp.write(data, len);
    udp.endPacket();
    
    Serial.printf(">>> FORWARDED to %s:%d\n", 
                  BROADCAST.toString().c_str(), SERVER_PORT);
  }
}

// ========================================
// Setup MPU6050
// ========================================
void setupMPU() {
  Serial.printf("Initializing I2C on SCL=%d, SDA=%d...\n", I2C_SCL_PIN, I2C_SDA_PIN);
  
  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
  
  if (!mpu.begin(MPU_ADDR, &Wire)) {
    Serial.printf("Failed to find MPU6050 at 0x%02X!\n", MPU_ADDR);
    Serial.println("Continuing without MPU6050 (receive-only mode)...");
    sensorAvailable = false;
    return;
  }
  
  Serial.printf("MPU6050 initialized successfully at 0x%02X\n", MPU_ADDR);
  
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
  
  az_bias -= 9.81;  // Gravity correction
  
  Serial.println("Calibration complete!");
}

// ========================================
// Arduino Setup
// ========================================
void setup() {
  Serial.begin(115200);
  delay(1000);
  
  Serial.println("\n\n==================================");
  Serial.println("ESP32-C3 Air Drum - Access Point");
  Serial.println("==================================\n");
  
  // Setup Access Point
  setupAP();
  
  // Setup UDP (for both receiving and sending)
  udp.begin(SERVER_PORT);
  Serial.printf("UDP listening on port %d\n", SERVER_PORT);
  Serial.printf("UDP ready to send to %s:%d\n", 
                BROADCAST.toString().c_str(), SERVER_PORT);
  
  // Setup MPU6050
  setupMPU();
  
  Serial.println("\nRunning...\n");
}

// ========================================
// Arduino Main Loop
// ========================================
void loop() {
  unsigned long now = millis();
  
  // Check for incoming UDP packets (from STA devices)
  int packetSize = udp.parsePacket();
  if (packetSize > 0) {
    uint8_t buffer[128];
    int len = udp.read(buffer, sizeof(buffer));
    
    IPAddress remoteIP = udp.remoteIP();
    uint16_t remotePort = udp.remotePort();
    
    forwardMIDI(buffer, len, remoteIP, remotePort);
  }
  
  // Read sensor (only if available)
  if (sensorAvailable) {
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);
    
    float ax = a.acceleration.x - ax_bias;
    float ay = a.acceleration.y - ay_bias;
    float az = a.acceleration.z - az_bias;
    
    // Low-pass filter
    ax_f = ALPHA_ACC_LP * ax_f + (1 - ALPHA_ACC_LP) * ax;
    ay_f = ALPHA_ACC_LP * ay_f + (1 - ALPHA_ACC_LP) * ay;
    az_f = ALPHA_ACC_LP * az_f + (1 - ALPHA_ACC_LP) * az;
    
    // Debug output
    if (DEBUG_SENSOR) {
      debug_counter++;
      if (debug_counter >= 20) {
        Serial.printf("az_f: %.3f (threshold: %.2f) | Clients: %d\n", 
                      az_f, THRESHOLD_HIT, WiFi.softAPgetStationNum());
        debug_counter = 0;
      }
    }
    
    // Hit Detection
    if (az_f < THRESHOLD_HIT && (now - last_hit) > DEBOUNCE_MS) {
      Serial.printf("*** AP HIT! *** → sending MIDI note %d | az_f: %.3f\n", 
                    MIDI_NOTE, az_f);
      
      sendNoteOn(MIDI_NOTE, MIDI_VELOCITY);
      Serial.printf(">>> SENT to %s:%d\n", 
                    BROADCAST.toString().c_str(), SERVER_PORT);
      
      note_off_time = now + NOTE_OFF_DELAY_MS;
      note_off_pending = true;
      last_hit = now;
    }
    
    // Non-blocking Note Off
    if (note_off_pending && now >= note_off_time) {
      sendNoteOff(MIDI_NOTE);
      note_off_pending = false;
    }
  }
  
  delay(DT * 1000);  // 50ms loop time
}
