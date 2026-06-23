#include "mlx90614_driver.h"

// ---------------------------------------------------------------------------
// Construction
// ---------------------------------------------------------------------------

MLX90614Driver::MLX90614Driver(uint8_t sda, uint8_t scl, uint8_t address)
    : _sda(sda), _scl(scl), _addr(address)
{
}

// ---------------------------------------------------------------------------
// Public API
// ---------------------------------------------------------------------------

bool MLX90614Driver::begin()
{
    // SMBus max clock is 100 kHz (datasheet section 4.1.4.3.12)
    Wire.begin(_sda, _scl);
    Wire.setClock(100000);

    // Check device is present by probing with a zero-byte transmission
    Wire.beginTransmission(_addr);
    uint8_t err = Wire.endTransmission();
    if (err != 0) {
        Serial.print("[MLX90614] begin() failed – I2C error ");
        Serial.println(err);
        return false;
    }

    // Wait for POR settling time before first valid read (Tvalid = 250ms typ)
    delay(MLX90614_POR_SETTLE_MS);
    return true;
}

bool MLX90614Driver::readAmbient(float& tempC)
{
    uint16_t raw;
    if (!readRegister(MLX90614_REG_TA, raw)) return false;
    return rawTocelsius(raw, tempC);
}

bool MLX90614Driver::readObject(float& tempC)
{
    uint16_t raw;
    if (!readRegister(MLX90614_REG_TOBJ1, raw)) return false;
    return rawTocelsius(raw, tempC);
}

bool MLX90614Driver::read(float& ambientC, float& objectC)
{
    return readAmbient(ambientC) && readObject(objectC);
}

bool MLX90614Driver::printJson(uint8_t decimals)
{
    float ta, to;
    if (!read(ta, to)) return false;

    Serial.print("{");
    Serial.print("\"ta\":"); Serial.print(ta, decimals);
    Serial.print(",\"to\":"); Serial.print(to, decimals);
    Serial.println("}");
    return true;
}

uint16_t MLX90614Driver::readFlags()
{
    uint16_t flags;
    if (!readRegister(MLX90614_CMD_FLAGS, flags)) return 0xFFFF;
    return flags;
}

// ---------------------------------------------------------------------------
// Private helpers
// ---------------------------------------------------------------------------

bool MLX90614Driver::readRegister(uint8_t reg, uint16_t& out)
{
    // SMBus Read Word (section 4.1.4.3.7):
    // Write phase: START | addr+W | command | (no STOP yet)
    // Read phase:  repeated START | addr+R | LSByte | MSByte | PEC | STOP
    //
    // Wire.endTransmission(false) issues a repeated START, keeping the bus.
    Wire.beginTransmission(_addr);
    Wire.write(reg);
    if (Wire.endTransmission(false) != 0) return false;

    // Request 3 bytes: data LSB, data MSB, PEC
    if (Wire.requestFrom((uint8_t)_addr, (uint8_t)3) != 3) return false;

    uint8_t lsb = Wire.read();
    uint8_t msb = Wire.read();
    uint8_t pec = Wire.read();

    // Validate PEC (CRC-8 over the full SMBus transaction bytes):
    // addr+W, command, addr+R, LSB, MSB — then compare against received PEC
    uint8_t crcData[5] = {
        (uint8_t)((_addr << 1) | 0),   // slave addr + Write bit
        reg,                             // command byte
        (uint8_t)((_addr << 1) | 1),   // slave addr + Read bit
        lsb,
        msb
    };
    if (crc8(crcData, 5) != pec) {
        Serial.println("[MLX90614] PEC mismatch");
        return false;
    }

    out = (uint16_t)(msb << 8) | lsb;
    return true;
}

bool MLX90614Driver::rawTocelsius(uint16_t raw, float& tempC)
{
    // MSb is an error flag for linearised temperatures (section 4.1.8.2)
    if (raw & MLX90614_ERROR_FLAG) {
        Serial.println("[MLX90614] Error flag set in RAM read");
        return false;
    }

    // Ta[K] = raw * 0.02, then convert to °C (section 4.1.8.1 & 4.1.8.2)
    tempC = ((float)raw * MLX90614_LSB_K) - MLX90614_KELVIN_OFFSET;
    return true;
}

uint8_t MLX90614Driver::crc8(uint8_t* data, uint8_t length)
{
    // CRC-8 with polynomial X8+X2+X1+1 (0x07), as specified in
    // SMBus spec and referenced in MLX90614 datasheet section 4.1.4.3.1
    uint8_t crc = 0x00;
    for (uint8_t i = 0; i < length; i++) {
        crc ^= data[i];
        for (uint8_t bit = 0; bit < 8; bit++) {
            crc = (crc & 0x80) ? ((crc << 1) ^ 0x07) : (crc << 1);
        }
    }
    return crc;
}