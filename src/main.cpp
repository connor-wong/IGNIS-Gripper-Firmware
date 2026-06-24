#include <Arduino.h>

/* Custom Libraries */
#include "ble_driver.h"
#include "imu_driver.h"
#include "motor_driver.h"
#include "encoder_driver.h"
// #include "mlx90614_driver.h"
#include "mcp9808_driver.h"
#include "vl53l5cx_driver.h"

// ---------------------------------------------------------------------------
// BLE
// ---------------------------------------------------------------------------

void onBLEReceive(uint8_t* data, size_t length);

BLEDriver ble;  // Uses default UUIDs, device name "IGNIS", MTU 512

// ---------------------------------------------------------------------------
// Motor and Encoder
// ---------------------------------------------------------------------------

MotorDriver   motor(4, 5, 2, 3);    // IN1=GPIO4, IN2=GPIO5, LEDC ch 2 & 3
EncoderDriver encoder(18, 19);       // yellow → GPIO18, green → GPIO19

// ---------------------------------------------------------------------------
// I2C Drivers (shared SDA=GPIO8, SCL=GPIO9)
// ---------------------------------------------------------------------------

#define SDA_PIN 8
#define SCL_PIN 9

ImuDriver      imu(SDA_PIN, SCL_PIN, MPU_DEFAULT_ADDR);
// MLX90614Driver objTemp(SDA_PIN, SCL_PIN, MLX90614_DEFAULT_ADDR);
MCP9808Driver  pcbTemp(SDA_PIN, SCL_PIN, MCP9808_DEFAULT_ADDR);
VL53L5CXDriver tof(SDA_PIN, SCL_PIN, 5, VL53L5CX_RES_8X8, 15);

// ---------------------------------------------------------------------------
// BLE write handler — called when client sends a command
// ---------------------------------------------------------------------------

void onBLEReceive(uint8_t* data, size_t length)
{
    if (length == 0) return;
    // TODO: parse incoming commands (e.g. motor speed, config changes)
}

// ---------------------------------------------------------------------------
// Timing
// ---------------------------------------------------------------------------

const unsigned long LOOP_INTERVAL_MS = 10;
unsigned long previousMillis = 0;

// ---------------------------------------------------------------------------
// Setup
// ---------------------------------------------------------------------------

void setup()
{
    Serial.begin(115200);

    // BLE
    ble.onWrite(onBLEReceive);
    ble.begin();

    // IMU
    if (!imu.begin()) {
        Serial.println("IMU init failed");
        while (true) {}
    }
    Serial.println("MPU initialized.");
    imu.calibrate();

    // ToF
    if (!tof.begin()) {
        Serial.println("VL53L5CX init failed");
        while (true) {}
    }

    // Temperature sensors
    // objTemp.begin();
    // pcbTemp.begin();
    // pcbTemp.setResolution(MCP9808_RES_0_0625);

    // Motor and encoder
    // motor.begin();
    // encoder.begin();
    // motor.setSpeed(180);
}

// ---------------------------------------------------------------------------
// Loop
// ---------------------------------------------------------------------------

void loop()
{
    unsigned long now = millis();

    if (now - previousMillis >= LOOP_INTERVAL_MS) {
        previousMillis = now;

        // Read IMU and send over BLE
        ImuData data;
        imu.read(data);

        // Build JSON into a stack buffer and send via BLE indicate
        char buf[128];
        snprintf(buf, sizeof(buf),
            "{\"ax\":%.3f,\"ay\":%.3f,\"az\":%.3f,"
            "\"gx\":%.3f,\"gy\":%.3f,\"gz\":%.3f}",
            data.accelX, data.accelY, data.accelZ,
            data.gyroX,  data.gyroY,  data.gyroZ);

        if (ble.isConnected()) {
            ble.indicate(buf);
        } else {
            Serial.println(buf);    // Fall back to Serial when no BLE client
        }
    }

    // encoder.update();
    // objTemp.printJson();
    // pcbTemp.printJson();
    // tof.printJson();
}