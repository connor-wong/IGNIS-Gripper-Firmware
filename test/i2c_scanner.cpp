#include <Arduino.h>
#include <Wire.h>

// ESP32-S3 I2C pins
#define SDA_PIN 8
#define SCL_PIN 9

// I2C scan range (7-bit addresses: 0x08–0x77)
// 0x00–0x07 and 0x78–0x7F are reserved by the I2C spec
#define I2C_ADDR_MIN 0x08
#define I2C_ADDR_MAX 0x77

// Known devices on this firmware stack — for annotation in scan output
struct KnownDevice {
    uint8_t     address;
    const char* name;
};

static const KnownDevice KNOWN_DEVICES[] = {
    { 0x18, "MCP9808 (PCB temp, A2A1A0=000)"     },
    { 0x19, "MCP9808 (A2A1A0=001)"                },
    { 0x1A, "MCP9808 (A2A1A0=010)"                },
    { 0x1B, "MCP9808 (A2A1A0=011)"                },
    { 0x1C, "MCP9808 (A2A1A0=100)"                },
    { 0x1D, "MCP9808 (A2A1A0=101)"                },
    { 0x1E, "MCP9808 (A2A1A0=110)"                },
    { 0x1F, "MCP9808 (A2A1A0=111)"                },
    { 0x29, "VL53L5CX (ToF, alt address)"         },
    { 0x52, "VL53L5CX (ToF, default)"             },
    // { 0x5A, "MLX90614 (IR temp, default)"         },
    { 0x68, "MPU-9265 (IMU, AD0=LOW)"             },
    { 0x69, "MPU-9265 (IMU, AD0=HIGH)"            },
};

static const uint8_t KNOWN_COUNT = sizeof(KNOWN_DEVICES) / sizeof(KNOWN_DEVICES[0]);

// Look up a known device name by address; returns nullptr if not in table
static const char* lookupDevice(uint8_t address)
{
    for (uint8_t i = 0; i < KNOWN_COUNT; i++) {
        if (KNOWN_DEVICES[i].address == address) return KNOWN_DEVICES[i].name;
    }
    return nullptr;
}

void setup()
{
    Serial.begin(115200);
    while (!Serial) {}  // Wait for USB serial on ESP32-S3

    Wire.begin(SDA_PIN, SCL_PIN);
    Wire.setClock(100000);  // 100kHz — safe for all devices during scan

    Serial.println("========================================");
    Serial.println(" I2C Bus Scanner");
    Serial.print  (" SDA: GPIO"); Serial.print(SDA_PIN);
    Serial.print  ("  SCL: GPIO"); Serial.println(SCL_PIN);
    Serial.println(" Clock: 100kHz");
    Serial.println("========================================");
    Serial.println();

    uint8_t found = 0;

    for (uint8_t addr = I2C_ADDR_MIN; addr <= I2C_ADDR_MAX; addr++) {
        Wire.beginTransmission(addr);
        uint8_t err = Wire.endTransmission();

        if (err == 0) {
            // Device ACKed — present on bus
            const char* name = lookupDevice(addr);

            Serial.print("  [FOUND] 0x");
            if (addr < 0x10) Serial.print("0");
            Serial.print(addr, HEX);

            if (name) {
                Serial.print("  →  ");
                Serial.print(name);
            } else {
                Serial.print("  →  Unknown device");
            }
            Serial.println();
            found++;

        } else if (err == 4) {
            // Error 4 = unknown/bus error at this address (not just NACK)
            Serial.print("  [ERROR] 0x");
            if (addr < 0x10) Serial.print("0");
            Serial.print(addr, HEX);
            Serial.println("  →  Bus error (check pull-ups)");
        }
        // err == 2 (NACK) is the normal "nothing here" response — skip silently
    }

    Serial.println();
    Serial.print("Scan complete. ");
    Serial.print(found);
    Serial.println(found == 1 ? " device found." : " devices found.");
    Serial.println("========================================");
}

void loop()
{
    // Nothing — scan runs once on startup.
    // Reset the ESP32-S3 to re-scan.
}