#pragma once

#include <Arduino.h>
#include <Wire.h>

// MPU-9265 Register Map
#define MPU_DEFAULT_ADDR    0x68
#define MPU_REG_ACCEL_XOUT  0x3B
#define MPU_REG_PWR_MGMT_1  0x6B
#define MPU_CALIBRATION_SAMPLES 500
#define MPU_CALIBRATION_DELAY_MS 2

// Sensitivity scale factors (default full-scale ranges)
// Accel: ±2g  → 16384 LSB/g
// Gyro:  ±250°/s → 131 LSB/°/s
#define ACCEL_SCALE_FACTOR  16384.0f
#define GYRO_SCALE_FACTOR   131.0f

struct ImuData {
    float accelX;   // g
    float accelY;   // g
    float accelZ;   // g
    float gyroX;    // °/s
    float gyroY;    // °/s
    float gyroZ;    // °/s
};

class ImuDriver {
public:
    /**
     * @param sda     SDA pin (default 8 for ESP32-S3)
     * @param scl     SCL pin (default 9 for ESP32-S3)
     * @param address I2C address (default 0x68)
     */
    ImuDriver(uint8_t sda = 8, uint8_t scl = 9, uint8_t address = MPU_DEFAULT_ADDR);

    /**
     * Initializes I2C and wakes the MPU. Returns false if the device
     * does not acknowledge on the bus.
     */
    bool begin();

    /**
     * Blocking 500-sample calibration routine. Keep the sensor flat
     * and still while this runs. Offsets are stored internally.
     */
    void calibrate();

    /**
     * Reads raw sensor values, applies calibration offsets and scale
     * factors, and writes the result into `data`.
     */
    void read(ImuData& data);

    /**
     * Serialises an ImuData struct to a compact JSON string and writes
     * it to Serial (terminated with '\n').
     *
     * Example output:
     *   {"ax":0.002,"ay":-0.001,"az":1.000,"gx":0.015,"gy":-0.008,"gz":0.003}
     */
    static void printJson(const ImuData& data, uint8_t decimals = 3);

private:
    uint8_t _sda;
    uint8_t _scl;
    uint8_t _addr;

    // Raw reads (populated by readRaw())
    int16_t _ax, _ay, _az;
    int16_t _gx, _gy, _gz;

    // Calibration offsets (in raw LSB units, except az which accounts for 1 g)
    float _axOff, _ayOff, _azOff;
    float _gxOff, _gyOff, _gzOff;

    /** Burst-reads all 6 axes in one I2C transaction. */
    void readRaw();
};