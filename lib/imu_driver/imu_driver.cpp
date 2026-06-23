#include "imu_driver.h"

// ---------------------------------------------------------------------------
// Construction
// ---------------------------------------------------------------------------

ImuDriver::ImuDriver(uint8_t sda, uint8_t scl, uint8_t address)
    : _sda(sda), _scl(scl), _addr(address),
      _ax(0), _ay(0), _az(0), _gx(0), _gy(0), _gz(0),
      _axOff(0.0f), _ayOff(0.0f), _azOff(0.0f),
      _gxOff(0.0f), _gyOff(0.0f), _gzOff(0.0f)
{
}

// ---------------------------------------------------------------------------
// Public API
// ---------------------------------------------------------------------------

bool ImuDriver::begin()
{
    Wire.begin(_sda, _scl);

    // Wake the MPU (clear sleep bit in PWR_MGMT_1)
    Wire.beginTransmission(_addr);
    Wire.write(MPU_REG_PWR_MGMT_1);
    Wire.write(0x00);
    uint8_t err = Wire.endTransmission();

    if (err != 0) {
        Serial.print("[ImuDriver] begin() failed – I2C error ");
        Serial.println(err);
        return false;
    }

    delay(100); // Allow sensor to stabilise after wake-up
    return true;
}

void ImuDriver::calibrate()
{
    Serial.println("[ImuDriver] Calibrating – keep sensor flat and still...");

    long axSum = 0, aySum = 0, azSum = 0;
    long gxSum = 0, gySum = 0, gzSum = 0;

    for (int i = 0; i < MPU_CALIBRATION_SAMPLES; i++) {
        readRaw();
        axSum += _ax;
        aySum += _ay;
        azSum += _az;
        gxSum += _gx;
        gySum += _gy;
        gzSum += _gz;
        delay(MPU_CALIBRATION_DELAY_MS);
    }

    _axOff = (float)axSum / MPU_CALIBRATION_SAMPLES;
    _ayOff = (float)aySum / MPU_CALIBRATION_SAMPLES;
    // Subtract 1 g worth of raw counts so Z reads ~0 when flat
    _azOff = ((float)azSum / MPU_CALIBRATION_SAMPLES) - ACCEL_SCALE_FACTOR;
    _gxOff = (float)gxSum / MPU_CALIBRATION_SAMPLES;
    _gyOff = (float)gySum / MPU_CALIBRATION_SAMPLES;
    _gzOff = (float)gzSum / MPU_CALIBRATION_SAMPLES;

    Serial.println("[ImuDriver] Calibration complete.");
}

void ImuDriver::read(ImuData& data)
{
    readRaw();

    data.accelX = (_ax - _axOff) / ACCEL_SCALE_FACTOR;
    data.accelY = (_ay - _ayOff) / ACCEL_SCALE_FACTOR;
    data.accelZ = (_az - _azOff) / ACCEL_SCALE_FACTOR;

    data.gyroX  = (_gx - _gxOff) / GYRO_SCALE_FACTOR;
    data.gyroY  = (_gy - _gyOff) / GYRO_SCALE_FACTOR;
    data.gyroZ  = (_gz - _gzOff) / GYRO_SCALE_FACTOR;
}

void ImuDriver::printJson(const ImuData& data, uint8_t decimals)
{
    Serial.print("{");
    Serial.print("\"ax\":"); Serial.print(data.accelX, decimals);
    Serial.print(",\"ay\":"); Serial.print(data.accelY, decimals);
    Serial.print(",\"az\":"); Serial.print(data.accelZ, decimals);
    Serial.print(",\"gx\":"); Serial.print(data.gyroX,  decimals);
    Serial.print(",\"gy\":"); Serial.print(data.gyroY,  decimals);
    Serial.print(",\"gz\":"); Serial.print(data.gyroZ,  decimals);
    Serial.println("}");
}

// ---------------------------------------------------------------------------
// Private helpers
// ---------------------------------------------------------------------------

void ImuDriver::readRaw()
{
    Wire.beginTransmission(_addr);
    Wire.write(MPU_REG_ACCEL_XOUT);
    Wire.endTransmission(false);           // Repeated-start for burst read
    Wire.requestFrom(_addr, (uint8_t)14, (uint8_t)true);

    _ax = Wire.read() << 8 | Wire.read();
    _ay = Wire.read() << 8 | Wire.read();
    _az = Wire.read() << 8 | Wire.read();
    Wire.read(); Wire.read();              // Skip temperature registers
    _gx = Wire.read() << 8 | Wire.read();
    _gy = Wire.read() << 8 | Wire.read();
    _gz = Wire.read() << 8 | Wire.read();
}