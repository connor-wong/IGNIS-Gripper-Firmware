#pragma once

#include <Arduino.h>
#include <Wire.h>

// MCP9808 default I2C address (A2=A1=A0=0 → 0b0011000)
#define MCP9808_DEFAULT_ADDR        0x18

// Register map
#define MCP9808_REG_CONFIG          0x01    // Configuration register
#define MCP9808_REG_ALERT_UPPER     0x02    // Alert upper boundary
#define MCP9808_REG_ALERT_LOWER     0x03    // Alert lower boundary
#define MCP9808_REG_CRITICAL        0x04    // Critical temperature
#define MCP9808_REG_TEMP            0x05    // Ambient temperature (read-only)
#define MCP9808_REG_MANUF_ID        0x06    // Manufacturer ID (should read 0x0054)
#define MCP9808_REG_DEVICE_ID       0x07    // Device ID (should read 0x0400)
#define MCP9808_REG_RESOLUTION      0x08    // Resolution register (1 byte)

// Config register bits (Table 5-1 in MCP9808 datasheet)
#define MCP9808_CONFIG_ALERT_MODE   (1 << 0)    // 0=comparator, 1=interrupt
#define MCP9808_CONFIG_ALERT_POL    (1 << 1)    // 0=active-low, 1=active-high
#define MCP9808_CONFIG_ALERT_SEL    (1 << 2)    // 0=upper+lower+crit, 1=crit only
#define MCP9808_CONFIG_ALERT_CTRL   (1 << 3)    // 1=enable alert output
#define MCP9808_CONFIG_ALERT_STAT   (1 << 4)    // Alert status (read-only)
#define MCP9808_CONFIG_SHUTDOWN     (1 << 8)    // 1=shutdown (low power)

// Temperature register flag bits [15:13]
#define MCP9808_TEMP_ABOVE_CRIT     (1 << 15)
#define MCP9808_TEMP_ABOVE_UPPER    (1 << 14)
#define MCP9808_TEMP_BELOW_LOWER    (1 << 13)
#define MCP9808_TEMP_SIGN_BIT       (1 << 12)   // 1=negative temperature

// Resolution settings (written to REG_RESOLUTION, 1 byte)
// Conversion times: 0.5°C=30ms, 0.25°C=65ms, 0.125°C=130ms, 0.0625°C=250ms
#define MCP9808_RES_0_5             0x00    // ±0.5°C   (typical)
#define MCP9808_RES_0_25            0x01    // ±0.25°C
#define MCP9808_RES_0_125           0x02    // ±0.125°C
#define MCP9808_RES_0_0625          0x03    // ±0.0625°C (default, highest)

// Temperature encoding: 1 LSB = 0.0625°C (fixed for the temp register)
#define MCP9808_TEMP_LSB            0.0625f

/**
 * MCP9808Driver
 *
 * Reads PCB/ambient temperature from the MCP9808 precision sensor via I2C.
 *
 * Wiring:
 *   VDD  → 3.3V or 5V
 *   GND  → GND
 *   SCL  → ESP32-S3 SCL (default pin 9), with 4.7kΩ pull-up
 *   SDA  → ESP32-S3 SDA (default pin 8), with 4.7kΩ pull-up
 *   A0–A2 → GND (default address 0x18); bridge pads to 3V3 to change address
 *   ALERT → optional interrupt pin for thermal alert
 *
 * I2C address:
 *   7-bit: 0011 A2 A1 A0 → 0x18 (default) to 0x1F (all pads high)
 */
class MCP9808Driver {
public:
    /**
     * @param sda     SDA GPIO pin
     * @param scl     SCL GPIO pin
     * @param address 7-bit I2C address (default 0x18)
     */
    MCP9808Driver(uint8_t sda = 8, uint8_t scl = 9,
                  uint8_t address = MCP9808_DEFAULT_ADDR);

    /**
     * Initialises I2C and verifies the device by checking manufacturer ID.
     * Returns false if the device does not respond or ID mismatch.
     */
    bool begin();

    /**
     * Reads the ambient temperature in °C.
     * Returns true on success; false on bus error.
     * @param tempC  Output temperature in degrees Celsius
     */
    bool readTemperature(float& tempC);

    /**
     * Sets the measurement resolution.
     * Lower resolution = faster conversion, less precision.
     * @param resolution  One of MCP9808_RES_0_5 / _0_25 / _0_125 / _0_0625
     */
    bool setResolution(uint8_t resolution = MCP9808_RES_0_0625);

    /**
     * Configures the ALERT pin as a comparator output (active-low).
     * The pin goes LOW when temperature exits the window [lowerC, upperC]
     * or exceeds criticalC.
     *
     * @param lowerC     Lower boundary in °C
     * @param upperC     Upper boundary in °C
     * @param criticalC  Critical (hard limit) in °C
     */
    bool setAlertLimits(float lowerC, float upperC, float criticalC);

    /**
     * Enables the ALERT output pin (comparator mode, active-low).
     * Must be called after setAlertLimits().
     */
    bool enableAlert();

    /**
     * Disables the ALERT output and clears the alert configuration.
     */
    bool disableAlert();

    /**
     * Places the sensor in low-power shutdown mode (~0.1μA).
     * No temperature conversions occur while shut down.
     */
    bool shutdown();

    /**
     * Wakes the sensor from shutdown mode.
     */
    bool wake();

    /**
     * Prints JSON to Serial:
     *   {"pcb":25.1250}
     * Returns false if the read failed.
     */
    bool printJson(uint8_t decimals = 4);

private:
    uint8_t _sda;
    uint8_t _scl;
    uint8_t _addr;

    /** Reads a 16-bit big-endian register. */
    bool readRegister16(uint8_t reg, uint16_t& out);

    /** Writes a 16-bit big-endian value to a register. */
    bool writeRegister16(uint8_t reg, uint16_t value);

    /** Writes a single byte to a register (used for resolution). */
    bool writeRegister8(uint8_t reg, uint8_t value);

    /**
     * Converts a float temperature in °C to the MCP9808 16-bit
     * boundary register format (section 5.1.3 of MCP9808 datasheet).
     * Format: [15:12] flags/sign, [11:4] integer, [3:0] fractional (0.0625°C)
     */
    uint16_t celsiusToRaw(float tempC);
};