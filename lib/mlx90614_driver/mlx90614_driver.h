#pragma once

#include <Arduino.h>
#include <Wire.h>

// MLX90614 default SMBus slave address (factory default)
#define MLX90614_DEFAULT_ADDR   0x5A

// RAM register addresses (Table 14)
#define MLX90614_REG_TA         0x06    // Ambient temperature
#define MLX90614_REG_TOBJ1      0x07    // Object temperature (IR sensor 1)
#define MLX90614_REG_TOBJ2      0x08    // Object temperature (IR sensor 2, dual-zone only)

// EEPROM register addresses (Table 8)
#define MLX90614_REG_EMISSIVITY 0x04    // Emissivity correction (EEPROM)

// SMBus command opcodes (Table 15)
#define MLX90614_CMD_RAM        0x00    // RAM access base (000x xxxx)
#define MLX90614_CMD_EEPROM     0x20    // EEPROM access base (001x xxxx)
#define MLX90614_CMD_FLAGS      0xF0    // Read status flags
#define MLX90614_CMD_SLEEP      0xFF    // Enter sleep mode (3V version only)

// Datasheet section 4.1.8: raw LSB = 0.02 K, subtract 273.15 for °C
#define MLX90614_LSB_K          0.02f
#define MLX90614_KELVIN_OFFSET  273.15f

// MSb of 16-bit RAM read is an error flag (section 4.1.8.2)
#define MLX90614_ERROR_FLAG     0x8000

// Post-POR settling time before first valid read (section 3.2, Tvalid = 250ms typ)
#define MLX90614_POR_SETTLE_MS  250

/**
 * MLX90614Driver
 *
 * Reads ambient (Ta) and object (To) temperatures from the MLX90614
 * IR thermometer via SMBus (I2C-compatible, 10–100 kHz).
 *
 * Wiring:
 *   VDD → 3.3V or 5V (match your variant: Axx=5V, Bxx=3V)
 *   GND → GND
 *   SCL → ESP32-S3 SCL pin (default 9), with 4.7kΩ pull-up to VDD
 *   SDA → ESP32-S3 SDA pin (default 8), with 4.7kΩ pull-up to VDD
 *
 * Notes:
 *   - SMBus uses repeated-START for read operations (handled by Wire library)
 *   - Each read response includes a PEC (CRC-8) byte for error detection
 *   - Factory default slave address is 0x5A; do NOT put two MLX90614
 *     devices with the same address on the same bus (no ARP support)
 */
class MLX90614Driver {
public:
    /**
     * @param sda     SDA GPIO pin
     * @param scl     SCL GPIO pin
     * @param address 7-bit SMBus slave address (default 0x5A)
     */
    MLX90614Driver(uint8_t sda = 8, uint8_t scl = 9,
                   uint8_t address = MLX90614_DEFAULT_ADDR);

    /**
     * Initialises I2C and waits for the POR settling time.
     * Returns false if the device does not ACK on the bus.
     */
    bool begin();

    /**
     * Reads the sensor die (ambient) temperature in °C.
     * Returns true on success; false if a bus error or error flag is set.
     * @param tempC  Output temperature in degrees Celsius
     */
    bool readAmbient(float& tempC);

    /**
     * Reads the IR object temperature (sensor 1) in °C.
     * Returns true on success; false if a bus error or error flag is set.
     * @param tempC  Output temperature in degrees Celsius
     */
    bool readObject(float& tempC);

    /**
     * Reads both ambient and object temperatures in one call.
     * Returns true only if both reads succeed.
     */
    bool read(float& ambientC, float& objectC);

    /**
     * Prints a JSON packet to Serial:
     *   {"ta":25.140,"to":32.660}
     * Returns false if either read failed (prints nothing).
     */
    bool printJson(uint8_t decimals = 3);

    /**
     * Reads the device status flags register.
     * Bit 4 (EE_DEAD): 1 = fatal EEPROM error.
     * Bit 7 (EEBUSY): 1 = EEPROM write/erase in progress.
     * Returns 0xFFFF on bus error.
     */
    uint16_t readFlags();

private:
    uint8_t _sda;
    uint8_t _scl;
    uint8_t _addr;

    /**
     * Performs an SMBus Read Word transaction.
     * Sends: START, slave address + W, command, repeated START,
     *        slave address + R, reads LSByte, MSByte, PEC, STOP.
     * Validates PEC (CRC-8, polynomial 0x07 = X8+X2+X1+1).
     *
     * @param reg     Register/command byte
     * @param out     16-bit result (raw RAM value)
     * @returns true on success, false on NACK or PEC mismatch
     */
    bool readRegister(uint8_t reg, uint16_t& out);

    /**
     * Converts a raw 16-bit RAM value to °C.
     * Checks error flag (MSb). Returns false if error bit is set.
     */
    bool rawTocelsius(uint16_t raw, float& tempC);

    /**
     * Computes CRC-8 per SMBus spec (polynomial 0x07).
     * Used to validate the PEC byte returned with each read.
     */
    uint8_t crc8(uint8_t* data, uint8_t length);
};