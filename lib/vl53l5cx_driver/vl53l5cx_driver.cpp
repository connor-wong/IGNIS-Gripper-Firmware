#include "vl53l5cx_driver.h"

// ---------------------------------------------------------------------------
// Construction
// ---------------------------------------------------------------------------

VL53L5CXDriver::VL53L5CXDriver(uint8_t sda, uint8_t scl,
                                uint8_t lpnPin,
                                uint8_t resolution,
                                uint8_t freqHz)
    : _sda(sda), _scl(scl),
      _lpnPin(lpnPin),
      _resolution(resolution),
      _freqHz(freqHz),
      _ranging(false),
      _dataReady(false)
{
    memset(&_data, 0, sizeof(_data));
}

// ---------------------------------------------------------------------------
// Public API
// ---------------------------------------------------------------------------

bool VL53L5CXDriver::begin()
{
    // LPn must be driven HIGH before I2C is initialised.
    // The sensor stays in hardware standby while LPn is LOW.
    pinMode(_lpnPin, OUTPUT);
    digitalWrite(_lpnPin, HIGH);
    delay(10);  // Allow sensor to exit standby

    // 1MHz I2C — required for 8x8 @ 15Hz to fit within bus bandwidth.
    Wire.begin(_sda, _scl);
    Wire.setClock(VL53L5CX_I2C_CLOCK);

    // SparkFun begin() handles firmware upload (~84KB) and ULD init.
    // This can take up to ~1 second.
    if (!_sensor.begin()) {
        Serial.println("[VL53L5CX] begin() failed – sensor not found or firmware upload failed");
        return false;
    }

    // Apply resolution and frequency before starting ranging
    _sensor.setResolution(_resolution);
    _sensor.setRangingFrequency(_freqHz);
    _sensor.startRanging();
    _ranging = true;

    return true;
}

bool VL53L5CXDriver::update()
{
    if (!_ranging) return false;

    _dataReady = false;

    if (_sensor.isDataReady()) {
        if (_sensor.getRangingData(&_data)) {
            _dataReady = true;
        }
    }

    return _dataReady;
}

int16_t VL53L5CXDriver::getDistance(uint8_t zone) const
{
    if (zone >= _resolution) return -1;
    if (_data.target_status[zone] != VL53L5CX_STATUS_VALID) return -1;
    return _data.distance_mm[zone];
}

uint8_t VL53L5CXDriver::getStatus(uint8_t zone) const
{
    if (zone >= _resolution) return 255;
    return _data.target_status[zone];
}

bool VL53L5CXDriver::isValid(uint8_t zone) const
{
    if (zone >= _resolution) return false;
    return _data.target_status[zone] == VL53L5CX_STATUS_VALID;
}

uint8_t VL53L5CXDriver::getResolution() const { return _resolution; }
uint8_t VL53L5CXDriver::getFrequency()  const { return _freqHz; }

bool VL53L5CXDriver::setResolution(uint8_t resolution)
{
    if (resolution != VL53L5CX_RES_4X4 && resolution != VL53L5CX_RES_8X8) return false;

    _sensor.stopRanging();
    _resolution = resolution;
    _sensor.setResolution(_resolution);
    _sensor.startRanging();
    return true;
}

bool VL53L5CXDriver::setFrequency(uint8_t freqHz)
{
    _sensor.stopRanging();
    _freqHz = freqHz;
    _sensor.setRangingFrequency(_freqHz);
    _sensor.startRanging();
    return true;
}

bool VL53L5CXDriver::stopRanging()
{
    _sensor.stopRanging();
    _ranging = false;
    return true;
}

bool VL53L5CXDriver::startRanging()
{
    _sensor.startRanging();
    _ranging = true;
    return true;
}

bool VL53L5CXDriver::printJson()
{
    if (!update()) return false;

    Serial.print("{\"d\":[");
    for (uint8_t i = 0; i < _resolution; i++) {
        Serial.print(getDistance(i));   // -1 for invalid zones
        if (i < _resolution - 1) Serial.print(",");
    }

    Serial.print("],\"s\":[");
    for (uint8_t i = 0; i < _resolution; i++) {
        Serial.print(_data.target_status[i]);
        if (i < _resolution - 1) Serial.print(",");
    }

    Serial.println("]}");
    return true;
}