/*
 * tcp_ap_relay.ino — ESP32 AP Relay (TCP Mode)
 * ---------------------------------------------
 * ESP32 XIAO C3 (ESPDRUMS_1) acting as Access Point + TCP Relay
 * 
 * Architecture:
 *   STA Devices → [WiFi] → THIS AP → [WiFi/TCP] → Desktop
 * 
 * Functions:
 *   1. Creates WiFi Access Point "ESPDRUMS"
 *   2. Accepts TCP connections from STA devices (kick, hi-hat, drum) on port 5000
 *   3. Forwards all received MIDI to desktop via TCP on port 6000
 *   4. Also reads its own MPU6050 and sends MIDI
 * 
 * Hardware:
 *   - ESP32 XIAO C3
 *   - MPU6050 on I2C: SDA=GPIO6, SCL=GPIO7, Address=0x68
 * 
 * Upload:
 *   - Board: XIAO_ESP32C3
 */

#include <WiFi.h>
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <esp_wifi.h>

// ========================================
// Configuration
// ========================================
// Access Point Settings
const char* AP_SSID = "ESPDRUMS";
const char* AP_PASSWORD = "Nitirocks";
const IPAddress AP_IP(192, 168, 4, 1);
const IPAddress GATEWAY(192, 168, 4, 1);
const IPAddress SUBNET(255, 255, 255, 0);

// TCP Server for STA devices
const int RELAY_SERVER_PORT = 5000;
WiFiServer relayServer(RELAY_SERVER_PORT);
WiFiClient staClients[4];  // Support up to 4 STA devices

// Desktop TCP Connection
// Auto-discovers desktop IP from connected WiFi clients
const int DESKTOP_PORT = 6000;              // Must match --port in: python run.py --port 6000
WiFiClient desktopClient;
IPAddress desktopIP;  // Discovered automatically
bool desktopIPFound = false;

// MIDI Settings (for this AP's own drum)
#define MIDI_CHANNEL 9
#define NOTE_OFF_DELAY_MS 50

// Physical Drum Dimensions (in cm)
// RIGHT STICK Starting Position: x=+17.5cm (right edge of snare)
#define SNARE_DIAMETER 35.6
#define TOM1_DIAMETER 25.4   // 10" tom
#define TOM2_DIAMETER 30.5   // 12" floor tom
#define TOM3_DIAMETER 25.4   // 10" tom (left side)
#define RIDE_DIAMETER 50.8   // 20" ride cymbal
#define CRASH_DIAMETER 40.6  // 16" crash cymbal

// Zone Classification - RIGHT STICK ONLY
// Uses acceleration direction to determine which drum was hit
// Based on physical layout with right stick starting at right edge of snare
// RIGHT STICK DRUMS: Snare, Tom1, Tom2, Ride
struct DrumZone {
  const char* name;
  float ax_min, ax_max;  // X acceleration range (normalized)
  float ay_min, ay_max;  // Y acceleration range (normalized)
  float az_threshold;    // Z acceleration threshold (down = negative)
  uint8_t midiNote;
};

DrumZone zones[4] = {
  {"Snare",  -0.30,  +0.30,  -0.70, +0.70,  -0.70,  38},  // Center - snare
  {"Tom1",   +0.30,  +0.60,  -0.70, +0.70,  -0.65,  48},  // Right near - mid tom
  {"Tom2",   +0.60,  +1.00,  -0.70, +0.50,  -0.60,  45},  // Right far - floor tom
  {"Ride",   +0.30,  +1.00,  +0.50, +1.00,  -0.40,  51}   // Upper right - ride cymbal
};

// MPU6050 I2C Pins
#define I2C_SDA 6
#define I2C_SCL 7
#define MPU_ADDR 0x68

// Motion Detection
#define SAMPLES_CAL 500
#define ALPHA_ACC_LP 0.5
#define THRESHOLD_HIT 10.0     // Acceleration magnitude threshold (m/s²) - lowered for easier triggering
#define DEBOUNCE_MS 200
#define DEBUG_SENSOR true
#define VELOCITY_MODE_50 true  // true = 50% mode, false = 100% mode

// Desktop Reconnection
#define DESKTOP_RECONNECT_MS 5000

// ========================================
// Global Variables
// ========================================
Adafruit_MPU6050 mpu;
bool sensorAvailable = false;

// Calibration
float ax_bias = 0.0, ay_bias = 0.0, az_bias = 0.0;
float gx_bias = 0.0, gy_bias = 0.0, gz_bias = 0.0;

// Filtering
float ax_f = 0.0, ay_f = 0.0, az_f = 0.0;

// Timing
unsigned long last_hit = 0;
unsigned long note_off_time = 0;
bool note_off_pending = false;
unsigned long last_desktop_reconnect = 0;
static int debug_counter = 0;

// Current hit state
uint8_t current_note = 0;
bool waiting_for_note_off = false;

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
  Serial.println("🔍 Scanning for desktop...");
  
  // Common desktop IPs first (most likely to be assigned by DHCP)
  int commonIPs[] = {2, 3, 4, 5, 6};
  for (int i = 0; i < 5; i++) {
    IPAddress testIP(192, 168, 4, commonIPs[i]);
    
    WiFiClient testClient;
    if (testClient.connect(testIP, DESKTOP_PORT, 100)) {  // Fast 100ms timeout
      desktopIP = testIP;
      desktopIPFound = true;
      testClient.stop();
      Serial.printf("✅ Desktop found at: %s\n", desktopIP.toString().c_str());
      return;
    }
    testClient.stop();
  }
  
  // If not found in common range, scan .7 to .20 (still reasonable)
  for (int ip_end = 7; ip_end <= 20; ip_end++) {
    IPAddress testIP(192, 168, 4, ip_end);
    
    WiFiClient testClient;
    if (testClient.connect(testIP, DESKTOP_PORT, 100)) {
      desktopIP = testIP;
      desktopIPFound = true;
      testClient.stop();
      Serial.printf("✅ Desktop found at: %s\n", desktopIP.toString().c_str());
      return;
    }
    testClient.stop();
  }
  
  Serial.println("❌ Desktop not found in scan range (.2 - .20)");
}

// ========================================
// Connect to Desktop
// ========================================
bool connectToDesktop() {
  if (desktopClient.connected()) {
    return true;
  }
  
  // If we haven't found desktop IP yet, try to discover it
  if (!desktopIPFound) {
    discoverDesktopIP();
  }
  
  if (!desktopIPFound) {
    // No desktop found yet
    return false;
  }
  
  Serial.printf("🖥️  Connecting to desktop %s:%d...\n", desktopIP.toString().c_str(), DESKTOP_PORT);
  
  if (desktopClient.connect(desktopIP, DESKTOP_PORT)) {
    // Disable Nagle's algorithm to prevent TCP buffering delays
    desktopClient.setNoDelay(true);
    Serial.println("✅ TCP Connected to Desktop!");
    Serial.println("   TCP_NODELAY enabled for low latency");
    return true;
  } else {
    Serial.println("❌ TCP Connection to Desktop Failed!");
    desktopIPFound = false;  // Reset to try discovery again
    return false;
  }
}

// ========================================
// Forward MIDI to Desktop
// ========================================
void forwardToDesktop(uint8_t* data, int len) {
  if (desktopClient.connected() && len >= 3) {
    desktopClient.write(data, len);
    desktopClient.flush();  // Force immediate send, don't buffer
  }
}

// ========================================
// MIDI Functions (for this AP's own sensor)
// ========================================
void sendMIDI(uint8_t status, uint8_t note, uint8_t velocity) {
  uint8_t midiMsg[3] = {status, note, velocity};
  forwardToDesktop(midiMsg, 3);
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
// Zone Classification Algorithm (Direction-based)
// ========================================
int classifyZone(float ax, float ay, float az) {
  // Normalize acceleration to get direction
  float mag = sqrt(ax*ax + ay*ay + az*az);
  if (mag < 0.01) return -1;
  
  float ax_norm = ax / mag;
  float ay_norm = ay / mag;
  float az_norm = az / mag;
  
  // PRIORITIZE SNARE: if mostly downward and centered horizontally, always use snare
  if (az_norm < -0.70 && abs(ax_norm) < 0.30 && abs(ay_norm) < 0.70) {
    return 0;  // Snare (index 0) - check this FIRST before other zones
  }
  
  // Check each zone's direction boundaries
  for (int i = 0; i < 4; i++) {
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
    return 0;  // Snare (index 0)
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
// Setup MPU6050
// ========================================
void setupMPU() {
  Serial.println("Initializing MPU6050...");
  Serial.printf("   I2C Pins: SDA=GPIO%d, SCL=GPIO%d\n", I2C_SDA, I2C_SCL);
  Wire.begin(I2C_SDA, I2C_SCL);
  delay(100);
  
  // Scan I2C bus first
  scanI2C();
  
  // Try both common MPU6050 addresses
  Serial.printf("Trying MPU6050 at 0x%02X...\n", MPU_ADDR);
  if (!mpu.begin(MPU_ADDR, &Wire)) {
    Serial.printf("⚠️  Failed at 0x%02X, trying 0x69...\n", MPU_ADDR);
    
    // Try alternate address
    if (!mpu.begin(0x69, &Wire)) {
      Serial.println("❌ MPU6050 not found at either address!");
      Serial.println("   Continuing in relay-only mode (no local drum)");
      sensorAvailable = false;
      return;
    } else {
      Serial.println("✅ MPU6050 found at 0x69!");
    }
  } else {
    Serial.printf("✅ MPU6050 found at 0x%02X!\n", MPU_ADDR);
  }
  
  Serial.println("Configuring MPU6050...");
  mpu.setAccelerometerRange(MPU6050_RANGE_2_G);
  mpu.setGyroRange(MPU6050_RANGE_250_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  
  sensorAvailable = true;
  
  // Calibration
  Serial.println("📊 Calibrating - keep board still...");
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
  az_bias -= 9.81;
  
  Serial.println("✅ Calibration complete!");
}

// ========================================
// Setup
// ========================================
void setup() {
  Serial.begin(115200);
  delay(1000);
  
  Serial.println("\n\n========================================");
  Serial.println("ESP32 AP RELAY - TCP Mode");
  Serial.println("========================================\n");
  
  // Setup Access Point
  setupAP();
  
  // Start TCP Server for STA devices
  relayServer.begin();
  Serial.printf("📡 TCP Relay Server listening on port %d\n", RELAY_SERVER_PORT);
  Serial.println("   Waiting for STA devices to connect...");
  
  // Setup MPU6050
  setupMPU();
  
  // Desktop will be auto-discovered when it connects to the AP
  Serial.println("\n📡 Waiting for desktop to connect to ESPDRUMS WiFi...");
  Serial.println("   Desktop will be auto-discovered and connected automatically");
  Serial.println("   Make sure to run: python run.py");
  
  // Try immediate connection to desktop
  Serial.println("\n🔍 Attempting immediate desktop discovery...");
  discoverDesktopIP();
  if (desktopIPFound) {
    connectToDesktop();
  }
  
  Serial.println("\n========================================");
  Serial.println("🚀 AP Relay Running!");
  Serial.println("========================================");
  Serial.printf("\n📊 Debug Mode: %s\n", DEBUG_SENSOR ? "ENABLED" : "DISABLED");
  Serial.printf("📡 AP Broadcasting: ESPDRUMS on %s\n", WiFi.softAPIP().toString().c_str());
  Serial.printf("🎵 Sensor Available: %s\n\n", sensorAvailable ? "YES" : "NO");
}

// ========================================
// Main Loop
// ========================================
void loop() {
  unsigned long now = millis();
  
  // ========== 1. Accept new STA client connections ==========
  if (relayServer.hasClient()) {
    WiFiClient newClient = relayServer.available();
    IPAddress clientIP = newClient.remoteIP();
    
    // Reject desktop if it tries to connect to relay port (should only use port 6000)
    if (desktopIPFound && clientIP == desktopIP) {
      Serial.printf("⚠️  Desktop trying to connect to relay port, rejecting %s (use port 6000 only)\n", 
                    clientIP.toString().c_str());
      newClient.stop();
      return;  // Skip to next loop iteration
    }
    
    // Find empty slot for STA device
    bool added = false;
    for (int i = 0; i < 4; i++) {
      if (!staClients[i] || !staClients[i].connected()) {
        if (staClients[i]) staClients[i].stop();
        staClients[i] = newClient;
        staClients[i].setNoDelay(true);  // Disable Nagle's algorithm
        Serial.printf("✅ STA client %d connected: %s\n", 
                      i, staClients[i].remoteIP().toString().c_str());
        added = true;
        break;
      }
    }
    
    if (!added) {
      Serial.println("⚠️  All STA slots full, rejecting client");
      newClient.stop();
    }
  }
  
  // ========== 2. Relay MIDI from STA devices to Desktop ==========
  for (int i = 0; i < 4; i++) {
    if (staClients[i] && staClients[i].connected()) {
      while (staClients[i].available()) {
        uint8_t buffer[32];
        int len = staClients[i].read(buffer, sizeof(buffer));
        if (len > 0) {
          // Forward to desktop
          forwardToDesktop(buffer, len);
        }
      }
    }
  }
  
  // ========== 3. Reconnect to Desktop if needed ==========
  if (!desktopClient.connected() && (now - last_desktop_reconnect) > DESKTOP_RECONNECT_MS) {
    last_desktop_reconnect = now;
    connectToDesktop();
  }
  
  // ========== 4. Read own MPU6050 sensor ==========
  if (sensorAvailable) {
    sensors_event_t a, g, temp;
    if (!mpu.getEvent(&a, &g, &temp)) {
      Serial.println("⚠️  Failed to read MPU6050!");
      return;
    }
    
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
        Serial.printf("RIGHT: a=(%.2f,%.2f,%.2f) mag=%.2f | Desktop: %s\n",
                      ax_f, ay_f, az_f, accel_mag,
                      desktopClient.connected() ? "Connected" : "Disconnected");
        debug_counter = 0;
      }
    }
    
    // Hit Detection - check if magnitude exceeds threshold
    if (accel_mag > THRESHOLD_HIT && (now - last_hit) > DEBOUNCE_MS) {
      // Classify which drum zone was hit
      int zone = classifyZone(ax_f, ay_f, az_f);
      
      if (zone >= 0 && zone < 4) {
        // Map acceleration to MIDI velocity
        uint8_t velocity = mapVelocity(accel_mag);
        
        Serial.printf("💥 RIGHT HIT! %s | dir=(%.2f,%.2f,%.2f) mag=%.2f vel=%d\n",
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
  
  // No delay - run as fast as possible for low latency
}
