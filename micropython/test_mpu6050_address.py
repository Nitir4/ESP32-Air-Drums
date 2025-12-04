"""
test_mpu6050_address.py - Find MPU6050 I2C Address
---------------------------------------------------
Scans I2C bus and tries to initialize MPU6050 at both possible addresses.

Upload and run:
    mpremote connect COM3 fs put test_mpu6050_address.py :test.py
    mpremote connect COM3 run :test.py
    
Or:
    ampy --port COM3 put test_mpu6050_address.py test.py
    ampy --port COM3 run test.py
"""

from machine import Pin, I2C
from time import sleep

# XIAO ESP32-C3 I2C pins
I2C_SCL_PIN = 7
I2C_SDA_PIN = 6

print("\n" + "="*50)
print("MPU6050 I2C Address Scanner")
print("="*50)

# Initialize I2C
try:
    i2c = I2C(0, scl=Pin(I2C_SCL_PIN), sda=Pin(I2C_SDA_PIN))
    print(f"✅ I2C initialized on SDA={I2C_SDA_PIN}, SCL={I2C_SCL_PIN}")
except Exception as e:
    print(f"❌ I2C initialization failed: {e}")
    raise

print("\n🔍 Scanning I2C bus...")
sleep(0.1)

# Scan for all I2C devices
devices = i2c.scan()
print(f"Found {len(devices)} device(s) on I2C bus:")

if len(devices) == 0:
    print("❌ No I2C devices found!")
    print("\nTroubleshooting:")
    print("  - Check MPU6050 wiring:")
    print("    VCC → 3.3V")
    print("    GND → GND")
    print("    SDA → GPIO6")
    print("    SCL → GPIO7")
    print("  - Check solder joints")
    print("  - Verify MPU6050 is powered (LED should be on)")
else:
    for addr in devices:
        print(f"  📍 Address: 0x{addr:02X} ({addr})")

# MPU6050 possible addresses
MPU6050_ADDR_LOW = 0x68   # AD0 pin connected to GND (default)
MPU6050_ADDR_HIGH = 0x69  # AD0 pin connected to VCC

print("\n🎯 Testing MPU6050 at known addresses...")

# Try address 0x68
print("\nTrying 0x68 (AD0=LOW)...")
try:
    from mpu6050 import MPU6050
    sensor_low = MPU6050(i2c, addr=0x68)
    vals = sensor_low.raw_values()
    print("✅ MPU6050 FOUND at 0x68!")
    print(f"   Sample data: ax={vals[0]}, ay={vals[1]}, az={vals[2]}")
    print("\n🎉 Use: sensor = MPU6050(i2c)  # Default address")
except Exception as e:
    print(f"❌ Not found at 0x68: {e}")

# Try address 0x69
print("\nTrying 0x69 (AD0=HIGH)...")
try:
    from mpu6050 import MPU6050
    sensor_high = MPU6050(i2c, addr=0x69)
    vals = sensor_high.raw_values()
    print("✅ MPU6050 FOUND at 0x69!")
    print(f"   Sample data: ax={vals[0]}, ay={vals[1]}, az={vals[2]}")
    print("\n🎉 Use: sensor = MPU6050(i2c, addr=0x69)")
except Exception as e:
    print(f"❌ Not found at 0x69: {e}")

print("\n" + "="*50)
print("Scan complete!")
print("="*50 + "\n")
