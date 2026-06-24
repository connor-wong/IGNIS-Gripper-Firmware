#pragma once

#include <Arduino.h>
#include <Wire.h>
#include <SparkFun_VL53L5CX_Library.h>

// I2C clock — 1MHz is required for 8x8 at 15Hz+ to fit within the bus budget.
// Lower speeds will work but may limit achievable ranging frequency.
#define VL53L5CX_I2C_CLOCK      1000000

// Valid target status code (ST ULD definition: 5 = valid ranging data)
#define VL53L5CX_STATUS_VALID   5

// Resolution options (number of zones)
#define VL53L5CX_RES_4X4        16      // 4x4 = 16 zones
#define VL53L5CX_RES_8X8        64      // 8x8 = 64 zones (max)

// Max ranging frequency per resolution (from VL53L5CX datasheet):
//   4x4: up to 60Hz
//   8x8: up to 15Hz
#define VL53L5CX_MAX_FREQ_4X4   60
#define VL53L5CX_MAX_FREQ_8X8   15

/**
 * VL53L5CXDriver
 *
 * Wraps the SparkFun VL53L5CX Arduino library to provide a clean,
 * consistent driver interface matching the rest of the firmware stack.
 *
 * The SparkFun library handles the ST ULD firmware upload (~84KB) and
 * all proprietary register interactions internally on begin().
 *
 * Wiring:
 *   VIN  → 3.3V
 *   GND  → GND
 *   SCL  → ESP32-S3 SCL (default pin 9)
 *   SDA  → ESP32-S3 SDA (default pin 8)
 *   INT  → optional interrupt GPIO (not used in polling mode)
 *   LPn  → GPIO output (must be driven HIGH to enable the sensor)
 *
 * Important:
 *   LPn (Low Power enable, active-LOW) must be HIGH for normal operation.
 *   Driving it LOW puts the sensor in hardware standby. This driver
 *   controls it via the lpnPin constructor parameter.
 *
 * Dependencies:
 *   SparkFun VL53L5CX Arduino Library
 *   Install via Arduino Library Manager: "SparkFun VL53L5CX Arduino Library"
 *   https://github.com/sparkfun/SparkFun_VL53L5CX_Arduino_Library
 */
class VL53L5CXDriver {
public:
    /**
     * @param sda         SDA GPIO pin
     * @param scl         SCL GPIO pin
     * @param lpnPin      GPIO connected to sensor LPn (enable) pin
     * @param resolution  VL53L5CX_RES_4X4 (16 zones) or VL53L5CX_RES_8X8 (64 zones)
     * @param freqHz      Ranging frequency in Hz (max 60 for 4x4, 15 for 8x8)
     */
    VL53L5CXDriver(uint8_t sda = 8, uint8_t scl = 9,
                   uint8_t lpnPin = 5,
                   uint8_t resolution = VL53L5CX_RES_8X8,
                   uint8_t freqHz = 15);

    /**
     * Drives LPn HIGH, initialises I2C, uploads firmware, configures
     * resolution and frequency, then starts ranging.
     * Returns false if the sensor is not found or firmware upload fails.
     * Note: begin() can take up to ~1 second due to firmware upload.
     */
    bool begin();

    /**
     * Polls the sensor for new data (non-blocking).
     * Must be called regularly in loop(). Returns true if fresh data
     * is available and has been latched into internal buffers.
     */
    bool update();

    /**
     * Returns the distance in mm for a given zone index.
     * Zone layout (8x8, top-left origin, row-major):
     *   index = row * cols + col  (0–63 for 8x8, 0–15 for 4x4)
     * Only valid after update() returns true.
     * Returns -1 if the zone has an invalid status.
     *
     * @param zone  Zone index (0 to resolution-1)
     */
    int16_t getDistance(uint8_t zone) const;

    /**
     * Returns the raw target status for a zone.
     * Status 5 = valid; other values indicate errors or no target.
     */
    uint8_t getStatus(uint8_t zone) const;

    /** Returns true if the last zone reading has a valid status (== 5). */
    bool isValid(uint8_t zone) const;

    /** Returns the configured number of zones (16 or 64). */
    uint8_t getResolution() const;

    /** Returns the configured ranging frequency in Hz. */
    uint8_t getFrequency() const;

    /**
     * Changes resolution at runtime. Stops and restarts ranging.
     * @param resolution  VL53L5CX_RES_4X4 or VL53L5CX_RES_8X8
     */
    bool setResolution(uint8_t resolution);

    /**
     * Changes ranging frequency at runtime.
     * @param freqHz  1–60 (4x4) or 1–15 (8x8)
     */
    bool setFrequency(uint8_t freqHz);

    /** Stops ranging. Call startRanging() to resume. */
    bool stopRanging();

    /** Resumes ranging after stopRanging(). */
    bool startRanging();

    /**
     * Prints a compact JSON packet to Serial when new data is available.
     * Format:
     *   {"d":[d0,d1,...,d63],"s":[s0,s1,...,s63]}
     *
     * Where "d" = distance_mm per zone (-1 if invalid),
     *       "s" = target_status per zone.
     *
     * Returns false if no new data was available this call.
     */
    bool printJson();

private:
    uint8_t _sda;
    uint8_t _scl;
    uint8_t _lpnPin;
    uint8_t _resolution;
    uint8_t _freqHz;
    bool    _ranging;

    SparkFun_VL53L5CX   _sensor;
    VL53L5CX_ResultsData _data;
    bool                 _dataReady;
};