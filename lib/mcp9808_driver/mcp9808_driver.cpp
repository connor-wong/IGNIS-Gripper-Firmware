#include "mcp9808_driver.h"

// ---------------------------------------------------------------------------
// Construction
// ---------------------------------------------------------------------------

MCP9808Driver::MCP9808Driver(uint8_t sda, uint8_t scl, uint8_t address)
    : _sda(sda), _scl(scl), _addr(address)
{
}

// ---------------------------------------------------------------------------
// Public API
// ---------------------------------------------------------------------------

bool MCP9808Driver::begin()
{
    // I2C standard 400 kHz (datasheet spec)
    Wire.begin(_sda, _scl);
    Wire.setClock(400000);

    // Verify device presence by reading manufacturer ID (should be 0x0054)
    uint16_t manufId;
    if (!readRegister16(MCP9808_REG_MANUF_ID, manufId)) {
        Serial.println("[MCP9808] begin() failed – no I2C response");
        return false;
    }
    if (manufId != 0x0054) {
        Serial.print("[MCP9808] Unexpected manufacturer ID: 0x");
        Serial.println(manufId, HEX);
        return false;
    }

    // Default to highest resolution (0.0625°C)
    return setResolution(MCP9808_RES_0_0625);
}

bool MCP9808Driver::readTemperature(float& tempC)
{
    uint16_t raw;
    if (!readRegister16(MCP9808_REG_TEMP, raw)) return false;

    // Strip the three alert flag bits [15:13]; keep [12:0]
    uint16_t data = raw & 0x1FFF;

    // Bit 12 is the sign bit for negative temperatures
    if (data & MCP9808_TEMP_SIGN_BIT) {
        // Two's complement for negative values:
        // Clear sign bit, subtract 256
        data &= 0x0FFF;
        tempC = (float)data * MCP9808_TEMP_LSB - 256.0f;
    } else {
        tempC = (float)data * MCP9808_TEMP_LSB;
    }

    return true;
}

bool MCP9808Driver::setResolution(uint8_t resolution)
{
    return writeRegister8(MCP9808_REG_RESOLUTION, resolution & 0x03);
}

bool MCP9808Driver::setAlertLimits(float lowerC, float upperC, float criticalC)
{
    if (!writeRegister16(MCP9808_REG_ALERT_LOWER,  celsiusToRaw(lowerC)))    return false;
    if (!writeRegister16(MCP9808_REG_ALERT_UPPER,  celsiusToRaw(upperC)))    return false;
    if (!writeRegister16(MCP9808_REG_CRITICAL,      celsiusToRaw(criticalC))) return false;
    return true;
}

bool MCP9808Driver::enableAlert()
{
    uint16_t config;
    if (!readRegister16(MCP9808_REG_CONFIG, config)) return false;

    // Enable alert output in comparator mode, active-low (default polarity)
    config |= MCP9808_CONFIG_ALERT_CTRL;
    config &= ~MCP9808_CONFIG_ALERT_MODE;   // Comparator mode (not interrupt)
    config &= ~MCP9808_CONFIG_ALERT_POL;    // Active-low
    config &= ~MCP9808_CONFIG_ALERT_SEL;    // Trigger on upper + lower + critical

    return writeRegister16(MCP9808_REG_CONFIG, config);
}

bool MCP9808Driver::disableAlert()
{
    uint16_t config;
    if (!readRegister16(MCP9808_REG_CONFIG, config)) return false;
    config &= ~MCP9808_CONFIG_ALERT_CTRL;
    return writeRegister16(MCP9808_REG_CONFIG, config);
}

bool MCP9808Driver::shutdown()
{
    uint16_t config;
    if (!readRegister16(MCP9808_REG_CONFIG, config)) return false;
    config |= MCP9808_CONFIG_SHUTDOWN;
    return writeRegister16(MCP9808_REG_CONFIG, config);
}

bool MCP9808Driver::wake()
{
    uint16_t config;
    if (!readRegister16(MCP9808_REG_CONFIG, config)) return false;
    config &= ~MCP9808_CONFIG_SHUTDOWN;
    return writeRegister16(MCP9808_REG_CONFIG, config);
}

bool MCP9808Driver::printJson(uint8_t decimals)
{
    float tempC;
    if (!readTemperature(tempC)) return false;

    Serial.print("{\"pcb\":");
    Serial.print(tempC, decimals);
    Serial.println("}");
    return true;
}

// ---------------------------------------------------------------------------
// Private helpers
// ---------------------------------------------------------------------------

bool MCP9808Driver::readRegister16(uint8_t reg, uint16_t& out)
{
    Wire.beginTransmission(_addr);
    Wire.write(reg);
    if (Wire.endTransmission() != 0) return false;

    if (Wire.requestFrom((uint8_t)_addr, (uint8_t)2) != 2) return false;

    // MCP9808 sends MSByte first (big-endian)
    uint8_t msb = Wire.read();
    uint8_t lsb = Wire.read();
    out = (uint16_t)(msb << 8) | lsb;
    return true;
}

bool MCP9808Driver::writeRegister16(uint8_t reg, uint16_t value)
{
    Wire.beginTransmission(_addr);
    Wire.write(reg);
    Wire.write((uint8_t)(value >> 8));      // MSByte first
    Wire.write((uint8_t)(value & 0xFF));
    return Wire.endTransmission() == 0;
}

bool MCP9808Driver::writeRegister8(uint8_t reg, uint8_t value)
{
    Wire.beginTransmission(_addr);
    Wire.write(reg);
    Wire.write(value);
    return Wire.endTransmission() == 0;
}

uint16_t MCP9808Driver::celsiusToRaw(float tempC)
{
    // Alert limit register format (MCP9808 datasheet section 5.1.3):
    // Bits [11:4] = integer part, bits [3:2] = 0.25°C steps
    // Bit 12 = sign bit for negative values
    // Resolution is fixed at 0.25°C for limit registers (not configurable)

    uint16_t raw = 0;

    if (tempC < 0.0f) {
        raw |= MCP9808_TEMP_SIGN_BIT;      // Set sign bit
        tempC = -tempC;
        // Encode as two's complement: (256 - |temp|) / 0.0625
        raw |= (uint16_t)((256.0f - tempC) / MCP9808_TEMP_LSB) & 0x0FFF;
    } else {
        raw |= (uint16_t)(tempC / MCP9808_TEMP_LSB) & 0x0FFF;
    }

    return raw;
}