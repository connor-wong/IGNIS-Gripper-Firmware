#include <Arduino.h>

/* Custom Libraries */
#include "imu_driver.h"
#include "motor_driver.h"
#include "encoder_driver.h"
#include "mlx90614_driver.h"
#include "mcp9808_driver.h"
    
// Motor Driver
MotorDriver motor(4, 5, 2, 3); // IN1=GPIO4, IN2=GPIO5, LEDC channels 2 & 3

// Encoder Driver
EncoderDriver encoder(18, 19);   // yellow → 18, green → 19

// ---------------------------------------------------------------------------
// I2C Drivers (shared SDA=GPIO8, SCL=GPIO9)
// ---------------------------------------------------------------------------

#define SDA_PIN 8
#define SCL_PIN 9

// IMU Driver | I2C address 0x68
ImuDriver imu(SDA_PIN, SCL_PIN, MPU_DEFAULT_ADDR);

// MLX90614 Driver | I2C address 0x5A
MLX90614Driver objTemp(SDA_PIN, SCL_PIN, MLX90614_DEFAULT_ADDR);

// MCP9808 Driver | I2C address 0x18
MCP9808Driver pcbTemp(SDA_PIN, SCL_PIN, MCP9808_DEFAULT_ADDR);

const unsigned long LOOP_INTERVAL_MS = 10;
unsigned long previousMillis = 0;

void setup() {
    Serial.begin(115200);

    // ---------------------------------------------------------------------------
    // Sensors Initialization
    // ---------------------------------------------------------------------------

    if (!imu.begin()) {
        Serial.println("IMU init failed – check wiring.");
        while (true) {}   // Halt; nothing useful to do without the sensor
    }

    Serial.println("MPU initialized.");
    imu.calibrate();

    // objTemp.begin();
    // pcbTemp.begin();
    // pcbTemp.setResolution(MCP9808_RES_0_0625);

    // ---------------------------------------------------------------------------
    // Motor and Encoder Initialization
    // ---------------------------------------------------------------------------

    // motor.begin();
    // encoder.begin();
    // motor.setSpeed(180);
}

void loop() {
    unsigned long now = millis();

    if (now - previousMillis >= LOOP_INTERVAL_MS) {
        previousMillis = now;

        ImuData data;
        imu.read(data);
        ImuDriver::printJson(data);
    }

    // encoder.update();
    // Serial.println(encoder.getRPM());
    
    // objTemp.printJson();
    // pcbTemp.printJson();
}