/*
 * ble_kick_pedal.ino — ESP32 Kick Pedal (Arduino BLE)
 * ----------------------------------------------------
 * Detects kick pedal press using MPU6050.
 * Sends MIDI note when pressed, waits for release.
 * 
 * Hardware:
 *   - ESP32 WROOM (or any ESP32 with BLE)
 *   - MPU6050 on I2C: SDA=GPIO21, SCL=GPIO22
 * 
 * Upload:
 *   - Board: ESP32 Dev Module
 *   - Partition Scheme: Default 4MB with spiffs
 */

#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <WiFi.h>
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

// ========================================
// Configuration
// ========================================
#define BLE_NAME "ESPDRUMS_KICK"

// MIDI Settings
#define MIDI_NOTE 36          // Bass Drum 1
#define MIDI_VELOCITY 127
#define MIDI_CHANNEL 9
#define NOTE_OFF_DELAY_MS 50

// MPU6050 I2C Pins
#define I2C_SDA 21
#define I2C_SCL 22

// Pedal Detection
#define SAMPLES_CAL 500
#define LOOP_DELAY_MS 20
#define ALPHA_ACC_LP 0.3      // Low-pass filter
#define THRESHOLD_PRESS -0.7  // Foot pressing down
#define THRESHOLD_RELEASE -0.2 // Foot lifted
#define DEBOUNCE_MS 100
#define DEBUG_SENSOR true

// BLE UUIDs
#define DRUM_SERVICE_UUID "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define DRUM_CHAR_UUID "beb5483e-36e1-4688-b7f5-ea07361b26a8"

// ========================================
// Global Variables
// ========================================
BLEServer *pServer = nullptr;
BLECharacteristic *pCharacteristic = nullptr;
bool deviceConnected = false;
bool oldDeviceConnected = false;

Adafruit_MPU6050 mpu;
sensors_event_t a, g, temp;

// Calibration
float ax_bias = 0.0, ay_bias = 0.0, az_bias = 0.0;

// Filtering
float ax_f = 0.0, ay_f = 0.0, az_f = 0.0;

// State Machine
bool pedal_pressed = false;
unsigned long last_trigger = 0;
unsigned long note_off_time = 0;
bool note_off_pending = false;

// Debug
int debug_counter = 0;

// ========================================
// BLE Callbacks
// ========================================
class MyServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer *pServer) {
    deviceConnected = true;
    Serial.println("✅ BLE connected!");
  }

  void onDisconnect(BLEServer *pServer) {
    deviceConnected = false;
    Serial.println("❌ BLE disconnected");
  }
};

// ========================================
// MIDI Functions
// ========================================
void sendMIDI(uint8_t status, uint8_t note, uint8_t velocity) {
  if (!deviceConnected || pCharacteristic == nullptr) return;

  uint8_t midiMsg[3] = {status, note, velocity};
  pCharacteristic->setValue(midiMsg, 3);
  pCharacteristic->notify();
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
// Setup
// ========================================
void setup() {
  Serial.begin(115200);
  delay(100);
  Serial.println("\n========================================");
  Serial.println("ESP32 Kick Pedal - BLE MIDI");
  Serial.println("========================================");

  // Disable WiFi
  WiFi.mode(WIFI_OFF);
  btStop();
  Serial.println("WiFi disabled - BLE only mode");

  // Initialize I2C
  Wire.begin(I2C_SDA, I2C_SCL);
  Serial.printf("I2C initialized: SDA=%d, SCL=%d\n", I2C_SDA, I2C_SCL);

  // Initialize MPU6050
  if (!mpu.begin()) {
    Serial.println("❌ Failed to find MPU6050!");
    while (1) {
      delay(1000);
    }
  }
  Serial.println("✅ MPU6050 initialized");

  mpu.setAccelerometerRange(MPU6050_RANGE_2_G);
  mpu.setGyroRange(MPU6050_RANGE_250_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

  // Calibration
  Serial.println("Calibrating - KEEP FOOT OFF PEDAL...");
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
  az_bias -= 9.81; // Gravity correction

  Serial.printf("Calibration done! az_bias=%.3f\n", az_bias);
  Serial.println("Ready - press pedal to trigger kick!");

  // Initialize BLE
  BLEDevice::init(BLE_NAME);
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());

  BLEService *pService = pServer->createService(DRUM_SERVICE_UUID);

  pCharacteristic = pService->createCharacteristic(
      DRUM_CHAR_UUID,
      BLECharacteristic::PROPERTY_READ |
          BLECharacteristic::PROPERTY_NOTIFY);

  pCharacteristic->addDescriptor(new BLE2902());

  pService->start();

  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(DRUM_SERVICE_UUID);
  pAdvertising->setScanResponse(true);
  pAdvertising->setMinPreferred(0x06);
  pAdvertising->setMinPreferred(0x12);
  BLEDevice::startAdvertising();

  Serial.printf("BLE advertising as: %s\n", BLE_NAME);
  Serial.println("========================================");
}

// ========================================
// Main Loop
// ========================================
void loop() {
  unsigned long now = millis();

  // Handle disconnection
  if (!deviceConnected && oldDeviceConnected) {
    delay(500);
    pServer->startAdvertising();
    Serial.println("Re-advertising...");
    oldDeviceConnected = deviceConnected;
  }

  if (deviceConnected && !oldDeviceConnected) {
    oldDeviceConnected = deviceConnected;
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
  if (DEBUG_SENSOR && deviceConnected) {
    debug_counter++;
    if (debug_counter >= 20) { // Every ~400ms
      Serial.printf("az_f: %.3f | State: %s | Threshold: %.2f\n",
                    az_f,
                    pedal_pressed ? "PRESSED" : "RELEASED",
                    THRESHOLD_PRESS);
      debug_counter = 0;
    }
  }

  // STATE MACHINE: Detect press and release
  if (!pedal_pressed) {
    // Waiting for press (foot pushing down)
    if (az_f < THRESHOLD_PRESS && (now - last_trigger) > DEBOUNCE_MS) {
      Serial.printf("🦶 KICK PRESSED! az_f=%.3f\n", az_f);
      pedal_pressed = true;

      sendNoteOn(MIDI_NOTE, MIDI_VELOCITY);
      Serial.printf("✅ Kick Note On sent (note %d)\n", MIDI_NOTE);

      note_off_time = now + NOTE_OFF_DELAY_MS;
      note_off_pending = true;
      last_trigger = now;
    }
  } else {
    // Waiting for release (foot lifted)
    if (az_f > THRESHOLD_RELEASE) {
      Serial.printf("🦶 KICK RELEASED! az_f=%.3f\n", az_f);
      pedal_pressed = false;
    }
  }

  // Non-blocking Note Off
  if (note_off_pending && (now >= note_off_time)) {
    sendNoteOff(MIDI_NOTE);
    Serial.printf("✅ Note Off sent (note %d)\n", MIDI_NOTE);
    note_off_pending = false;
  }

  delay(LOOP_DELAY_MS);
}
