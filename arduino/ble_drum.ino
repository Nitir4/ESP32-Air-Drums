/*
 * ble_drum.ino — ESP32-C3 Air Drum (Bluetooth Low Energy)
 * --------------------------------------------------------
 * BLE peripheral that sends MIDI data over Bluetooth when drum hits are detected.
 * Uses MPU6050 accelerometer for hit detection.
 * 
 * Hardware:
 *   - ESP32-C3 XIAO
 *   - MPU6050 (I2C: SDA=GPIO6, SCL=GPIO7)
 * 
 * Upload:
 *   1. Install ESP32 board support in Arduino IDE
 *   2. Install libraries: Adafruit MPU6050, Adafruit Unified Sensor
 *   3. Select Board: "XIAO_ESP32C3"
 *   4. Upload this sketch
 */

#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

// ========================================
// Configuration
// ========================================
#define BLE_NAME "ESPDRUMS_1"  // Change for multiple drums (_2, _3, etc.)

// MIDI Settings
#define MIDI_NOTE 38           // Snare drum (change per drum: 36=kick, 42=hihat, etc.)
#define MIDI_VELOCITY 100
#define MIDI_CHANNEL 9         // MIDI channel 10 (drums) is 9 when zero-based
#define NOTE_OFF_DELAY_MS 50   // Delay before sending Note Off

// MPU6050 I2C Pins (XIAO ESP32-C3)
#define I2C_SDA 6
#define I2C_SCL 7

// Motion Detection
#define SAMPLES_CAL 500        // Calibration sample count
#define LOOP_DELAY_MS 50       // Main loop delay
#define ALPHA_ACC_LP 0.5       // Low-pass filter coefficient
#define THRESHOLD_HIT -0.3     // Downward accel threshold (g)
#define DEBOUNCE_MS 200        // Min time between hits

// Debug
#define DEBUG_SENSOR false     // Set to true to print sensor values

// BLE UUIDs (must match ble_server.py)
#define SERVICE_UUID        "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define CHARACTERISTIC_UUID "beb5483e-36e1-4688-b7f5-ea07361b26a8"

// ========================================
// Global Variables
// ========================================
Adafruit_MPU6050 mpu;
BLEServer *pServer = NULL;
BLECharacteristic *pCharacteristic = NULL;
bool deviceConnected = false;
bool oldDeviceConnected = false;

// Calibration offsets
float ax_bias = 0, ay_bias = 0, az_bias = 0;
float gx_bias = 0, gy_bias = 0, gz_bias = 0;

// Filtered values
float ax_f = 0, ay_f = 0, az_f = 0;

// Hit detection
unsigned long last_hit = 0;
unsigned long note_off_time = 0;
bool note_off_pending = false;

// Debug counter
int debug_counter = 0;

// ========================================
// BLE Callbacks
// ========================================
class MyServerCallbacks: public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) {
      deviceConnected = true;
      Serial.println("✅ BLE connected!");
    };

    void onDisconnect(BLEServer* pServer) {
      deviceConnected = false;
      Serial.println("❌ BLE disconnected");
    }
};

// ========================================
// MIDI Functions
// ========================================
void sendMIDI(uint8_t status, uint8_t note, uint8_t velocity) {
  if (deviceConnected && pCharacteristic != NULL) {
    uint8_t midiPacket[3] = {status, note, velocity};
    pCharacteristic->setValue(midiPacket, 3);
    pCharacteristic->notify();
  }
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
// Setup
// ========================================
void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("\n=== ESP32-C3 BLE Air Drum ===");

  // Reduce CPU frequency for power stability
  setCpuFrequencyMhz(80);
  Serial.println("CPU frequency set to 80MHz");

  // Initialize I2C
  Wire.begin(I2C_SDA, I2C_SCL);
  Serial.println("I2C initialized");

  // Initialize MPU6050
  if (!mpu.begin()) {
    Serial.println("❌ MPU6050 initialization failed!");
    while (1) {
      delay(10);
    }
  }
  Serial.println("✅ MPU6050 initialized");

  // Configure MPU6050
  mpu.setAccelerometerRange(MPU6050_RANGE_2_G);
  mpu.setGyroRange(MPU6050_RANGE_250_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

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
  az_bias -= 9.81;  // Gravity correction (m/s²)

  Serial.println("✅ Calibration complete");
  Serial.printf("Biases - ax:%.2f ay:%.2f az:%.2f\n", ax_bias, ay_bias, az_bias);

  // Initialize BLE
  Serial.println("Initializing BLE...");
  BLEDevice::init(BLE_NAME);
  
  // Create BLE Server
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());

  // Create BLE Service
  BLEService *pService = pServer->createService(SERVICE_UUID);

  // Create BLE Characteristic
  pCharacteristic = pService->createCharacteristic(
                      CHARACTERISTIC_UUID,
                      BLECharacteristic::PROPERTY_READ |
                      BLECharacteristic::PROPERTY_NOTIFY
                    );

  // Add descriptor for notifications
  pCharacteristic->addDescriptor(new BLE2902());

  // Start the service
  pService->start();

  // Start advertising
  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);
  pAdvertising->setScanResponse(true);
  pAdvertising->setMinPreferred(0x06);  // Help with iPhone connections
  pAdvertising->setMinPreferred(0x12);
  BLEDevice::startAdvertising();
  
  Serial.println("✅ BLE initialized");
  Serial.printf("📡 Advertising as: %s\n", BLE_NAME);
  Serial.println("Waiting for connection...");
}

// ========================================
// Main Loop
// ========================================
void loop() {
  unsigned long now = millis();

  // Handle BLE connection state changes
  if (deviceConnected && !oldDeviceConnected) {
    Serial.println("🎵 Ready to send MIDI data via BLE!");
    oldDeviceConnected = deviceConnected;
  }
  
  if (!deviceConnected && oldDeviceConnected) {
    Serial.println("⏳ Waiting for BLE connection...");
    delay(500); // Brief delay before re-advertising
    pServer->startAdvertising();
    Serial.println("📡 Restarting advertising...");
    oldDeviceConnected = deviceConnected;
  }

  // Read sensor
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  // Apply calibration (convert from m/s² to g)
  float ax = (a.acceleration.x - ax_bias) / 9.81;
  float ay = (a.acceleration.y - ay_bias) / 9.81;
  float az = (a.acceleration.z - az_bias) / 9.81;

  // Low-pass filter
  ax_f = ALPHA_ACC_LP * ax_f + (1 - ALPHA_ACC_LP) * ax;
  ay_f = ALPHA_ACC_LP * ay_f + (1 - ALPHA_ACC_LP) * ay;
  az_f = ALPHA_ACC_LP * az_f + (1 - ALPHA_ACC_LP) * az;

  // Debug output (every 2 seconds)
  if (DEBUG_SENSOR && deviceConnected) {
    debug_counter++;
    if (debug_counter >= 40) {
      Serial.printf("az_f: %.3f (threshold: %.2f)\n", az_f, THRESHOLD_HIT);
      debug_counter = 0;
    }
  }

  // Hit detection
  if (az_f < THRESHOLD_HIT && (now - last_hit) > DEBOUNCE_MS) {
    Serial.printf("*** HIT DETECTED! *** → MIDI note %d | az_f: %.3f\n", MIDI_NOTE, az_f);
    
    if (deviceConnected) {
      sendNoteOn(MIDI_NOTE, MIDI_VELOCITY);
      Serial.println("✅ Note On sent via BLE");
      note_off_time = now + NOTE_OFF_DELAY_MS;
      note_off_pending = true;
    } else {
      Serial.println("⚠️ Note On failed (not connected)");
    }
    
    last_hit = now;
  }

  // Non-blocking Note Off
  if (note_off_pending && (now >= note_off_time)) {
    if (deviceConnected) {
      sendNoteOff(MIDI_NOTE);
      Serial.println("✅ Note Off sent via BLE");
    }
    note_off_pending = false;
  }

  delay(LOOP_DELAY_MS);
}
