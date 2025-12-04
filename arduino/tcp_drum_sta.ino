/*
 * tcp_drum_sta.ino — ESP32 Air Drum (TCP Station Mode)
 * -----------------------------------------------------
 * ESP32 XIAO C3 (ESPDRUMS_2) acting as WiFi Station (STA)
 * Connects to AP (ESPDRUMS_1) and forwards MIDI to it via TCP
 * 
 * Architecture:
 *   DRUM (XIAO C3 #2) → [WiFi] → AP (ESPDRUMS_1) → [WiFi] → Desktop
 * 
 * Hardware:
 *   - ESP32 XIAO C3
 *   - MPU6050 on I2C: SDA=GPIO6, SCL=GPIO7, Address=0x69
 * 
 * Upload:
 *   - Board: XIAO_ESP32C3
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
const char* WIFI_SSID = "ESPDRUMS";
const char* WIFI_PASSWORD = "Nitirocks";
const char* AP_SERVER_IP = "192.168.4.1";
const int AP_SERVER_PORT = 5000;

// MIDI Settings
#define MIDI_CHANNEL 9
#define NOTE_OFF_DELAY_MS 50

// Physical Drum Dimensions (in cm)
// LEFT STICK Starting Position: x=-17.5cm (left edge of snare)
#define SNARE_DIAMETER 35.6
#define TOM1_DIAMETER 25.4   // 10" tom
#define TOM2_DIAMETER 30.5   // 12" floor tom
#define TOM3_DIAMETER 25.4   // 10" tom (left side)
#define CRASH_DIAMETER 40.6  // 16" crash cymbal

// Zone Classification - LEFT STICK ONLY
// Uses acceleration direction to determine which drum was hit
// Based on physical layout with left stick starting at left edge of snare
// LEFT STICK DRUMS: Tom3 (hi-tom), Snare, Crash
struct DrumZone {
  const char* name;
  float ax_min, ax_max;  // X acceleration range (normalized)
  float ay_min, ay_max;  // Y acceleration range (normalized)
  float az_threshold;    // Z acceleration threshold (down = negative)
  uint8_t midiNote;
};

DrumZone zones[3] = {
  {"Tom3",    -1.00,  -0.30,  -0.70, +0.70,  -0.65,  50},  // Left direction - hi tom
  {"Snare",   -0.30,  +0.30,  -0.70, +0.70,  -0.70,  38},  // Center - snare
  {"Crash",   +0.30,  +1.00,  +0.50, +1.00,  -0.40,  49}   // Upper right - crash cymbal
};

// MPU6050 I2C Pins (ESP32 XIAO C3)
#define I2C_SDA 6
#define I2C_SCL 7
#define MPU_ADDR 0x69  // STA drum uses 0x69

// Motion Detection
#define SAMPLES_CAL 500
#define ALPHA_ACC_LP 0.5
#define THRESHOLD_HIT 10.0     // Acceleration magnitude threshold (m/s²) - lowered for easier triggering
#define DEBOUNCE_MS 200
#define DEBUG_SENSOR true
#define VELOCITY_MODE_50 true  // true = 50% mode, false = 100% mode

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
float gx_bias = 0.0, gy_bias = 0.0, gz_bias = 0.0;

// Filtering
float ax_f = 0.0, ay_f = 0.0, az_f = 0.0;
// State
unsigned long last_hit = 0;
unsigned long note_off_time = 0;
bool note_off_pending = false;
unsigned long last_reconnect_attempt = 0;

// Current hit state
uint8_t current_note = 0;
bool waiting_for_note_off = false;

// Debug
static int debug_counter = 0;

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

void sendNoteOn(uint8_t note, uint8_t velocity) {
  uint8_t status = 0x90 | (MIDI_CHANNEL & 0x0F);
  sendMIDI(status, note, velocity);
}

void sendNoteOff(uint8_t note) {
  uint8_t status = 0x80 | (MIDI_CHANNEL & 0x0F);
  sendMIDI(status, note, 0);
}

// ========================================
// Zone Classification Algorithm
// ========================================
int classifyZone(float ax, float ay, float az) {
  // Normalize acceleration vector
  float mag = sqrt(ax*ax + ay*ay + az*az);
  if (mag < 0.01) return -1;  // Avoid division by zero
  
  float ax_norm = ax / mag;
  float ay_norm = ay / mag;
  float az_norm = az / mag;
  
  // PRIORITIZE SNARE: if mostly downward and centered horizontally, always use snare
  if (az_norm < -0.70 && abs(ax_norm) < 0.30 && abs(ay_norm) < 0.70) {
    return 1;  // Snare (index 1) - check this FIRST before other zones
  }
  
  // Check each zone's direction boundaries
  for (int i = 0; i < 3; i++) {
    // Check if acceleration direction is within zone's direction range
    if (ax_norm >= zones[i].ax_min && ax_norm <= zones[i].ax_max &&
        ay_norm >= zones[i].ay_min && ay_norm <= zones[i].ay_max) {
      
      // Check if downward component exceeds zone's threshold
      if (az_norm < zones[i].az_threshold) {
        return i;
      }
    }
  }
  
  // Final fallback: any downward motion defaults to snare
  if (az_norm < -0.60) {
    return 1;  // Snare (index 1)
  }
  
  return -1;  // No zone matched
}

// ========================================
// Velocity Mapping
// ========================================
uint8_t mapVelocity(float accel_magnitude) {
  uint8_t velocity;
  
  if (VELOCITY_MODE_50) {
    // 50% mode: accel / 2, clamped to 127
    velocity = (uint8_t)min(127, (int)(accel_magnitude / 2.0));
  } else {
    // 100% mode: direct mapping, clamped to 127
    velocity = (uint8_t)min(127, (int)accel_magnitude);
  }
  
  // Ensure minimum velocity of 1
  if (velocity < 1) velocity = 1;
  
  return velocity;
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
  Serial.println("ESP32 AIR DRUM - TCP Station Mode");
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

  // This drum uses MPU at 0x69 (AD0=HIGH)
  Serial.printf("Trying MPU6050 at 0x%02X...\n", MPU_ADDR);
  if (!mpu.begin(MPU_ADDR, &Wire)) {
    Serial.printf("⚠️  Failed at 0x%02X, trying 0x68...\n", MPU_ADDR);
    
    if (!mpu.begin(0x68, &Wire)) {
      Serial.println("❌ MPU6050 not found at either address!");
      Serial.println("   Check your wiring and restart.");
      while (1) delay(1000);
    } else {
      Serial.println("✅ MPU6050 found at 0x68!");
    }
  } else {
    Serial.printf("✅ MPU6050 found at 0x%02X!\n", MPU_ADDR);
  }
  
  Serial.println("Configuring MPU6050...");
  mpu.setAccelerometerRange(MPU6050_RANGE_2_G);
  mpu.setGyroRange(MPU6050_RANGE_250_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  
  // Calibration
  Serial.println("📊 Calibrating - keep board still...");
  for (int i = 0; i < SAMPLES_CAL; i++) {
    sensors_event_t a_temp, g_temp, temp_temp;
    mpu.getEvent(&a_temp, &g_temp, &temp_temp);
    
    ax_bias += a_temp.acceleration.x;
    ay_bias += a_temp.acceleration.y;
    az_bias += a_temp.acceleration.z;
    gx_bias += g_temp.gyro.x;
    gy_bias += g_temp.gyro.y;
    gz_bias += g_temp.gyro.z;
    
    delay(10);
  }
  
  ax_bias /= SAMPLES_CAL;
  ay_bias /= SAMPLES_CAL;
  az_bias /= SAMPLES_CAL;
  gx_bias /= SAMPLES_CAL;
  gy_bias /= SAMPLES_CAL;
  gz_bias /= SAMPLES_CAL;
  az_bias -= 9.81;
  
  Serial.println("✅ Calibration complete!");
  
  // Try initial TCP connection
  connectToAP();
  
  Serial.println("\n========================================");
  Serial.println("🚀 Drum Ready!");
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

  // Apply calibration bias
  float ax = a.acceleration.x - ax_bias;
  float ay = a.acceleration.y - ay_bias;
  float az = a.acceleration.z - az_bias;
    
    // Low-pass filter
    ax_f = ALPHA_ACC_LP * ax_f + (1 - ALPHA_ACC_LP) * ax;
    ay_f = ALPHA_ACC_LP * ay_f + (1 - ALPHA_ACC_LP) * ay;
    az_f = ALPHA_ACC_LP * az_f + (1 - ALPHA_ACC_LP) * az;
    
    // Calculate acceleration magnitude
    float accel_mag = sqrt(ax_f*ax_f + ay_f*ay_f + az_f*az_f);
    
    if (DEBUG_SENSOR) {
      debug_counter++;
      if (debug_counter >= 50) {  // Reduced frequency to prevent blocking
        Serial.printf("LEFT: a=(%.2f,%.2f,%.2f) mag=%.2f | TCP: %s\n",
                      ax_f, ay_f, az_f, accel_mag,
                      tcpClient.connected() ? "Connected" : "Disconnected");
        debug_counter = 0;
      }
    }

  // Hit Detection - check if magnitude exceeds threshold
  if (accel_mag > THRESHOLD_HIT && (now - last_hit) > DEBOUNCE_MS) {
    // Classify which drum zone was hit
    int zone = classifyZone(ax_f, ay_f, az_f);
    
      if (zone >= 0 && zone < 3) {
        // Map acceleration to MIDI velocity
        uint8_t velocity = mapVelocity(accel_mag);
        
        Serial.printf("💥 LEFT HIT! %s | dir=(%.2f,%.2f,%.2f) mag=%.2f vel=%d\n",
                      zones[zone].name, ax_f/accel_mag, ay_f/accel_mag, az_f/accel_mag, accel_mag, velocity);
        
        // Send Note On
        sendNoteOn(zones[zone].midiNote, velocity);
      
      // Store for Note Off
      current_note = zones[zone].midiNote;
      waiting_for_note_off = true;
      note_off_time = now + NOTE_OFF_DELAY_MS;
      last_hit = now;
    }
  }

  // Non-blocking Note Off
  if (waiting_for_note_off && (now >= note_off_time)) {
    sendNoteOff(current_note);
    waiting_for_note_off = false;
  }
}
