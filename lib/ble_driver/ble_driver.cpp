#include "ble_driver.h"

// ---------------------------------------------------------------------------
// NimBLE callback shims
//
// NOTE: NimBLE-Arduino defines:
//   #define BLEServerCallbacks         NimBLEServerCallbacks
//   #define BLECharacteristicCallbacks NimBLECharacteristicCallbacks
// Using those names directly would cause a redefinition error. The shims
// are prefixed "Ignis" to stay outside NimBLE's macro namespace.
// ---------------------------------------------------------------------------

class IgnisServerCallbacks : public NimBLEServerCallbacks {
public:
    explicit IgnisServerCallbacks(BLEDriver* driver) : _driver(driver) {}

    // 2.x signature — single declaration with NimBLEConnInfo& (required)
    void onConnect(NimBLEServer* pServer, NimBLEConnInfo& connInfo) override {
        _driver->_onConnect(pServer, connInfo);
    }

    // 2.x signature — NimBLEConnInfo& + int reason (both required)
    void onDisconnect(NimBLEServer* pServer, NimBLEConnInfo& connInfo, int reason) override {
        _driver->_onDisconnect(pServer, connInfo, reason);
    }

    // 2.x: onMTUChanged renamed to onMTUChange
    void onMTUChange(uint16_t mtu, NimBLEConnInfo& connInfo) override {
        _driver->_onMTUChange(mtu, connInfo);
    }

private:
    BLEDriver* _driver;
};

class IgnisCharCallbacks : public NimBLECharacteristicCallbacks {
public:
    explicit IgnisCharCallbacks(BLEDriver* driver) : _driver(driver) {}

    // 2.x: onWrite takes NimBLEConnInfo& as additional required parameter
    void onWrite(NimBLECharacteristic* pChar, NimBLEConnInfo& connInfo) override {
        _driver->_onWrite(pChar);
    }

    // 2.x: onStatus no longer takes a 'status' parameter — only return code
    void onStatus(NimBLECharacteristic* pChar, int code) override {
        if (code != 0 && code != BLE_HS_EDONE) {
            Serial.print("[BLEDriver] Indicate failed, code: ");
            Serial.println(code);
        }
    }

private:
    BLEDriver* _driver;
};

// ---------------------------------------------------------------------------
// Construction
// ---------------------------------------------------------------------------

BLEDriver::BLEDriver(const char* deviceName,
                     const char* serviceUUID,
                     const char* charUUID,
                     int8_t      txPower,
                     uint16_t    mtu)
    : _deviceName(deviceName),
      _serviceUUID(serviceUUID),
      _charUUID(charUUID),
      _txPower(txPower),
      _mtu(mtu),
      _writeCallback(nullptr),
      _pServer(nullptr),
      _pCharacteristic(nullptr),
      _negotiatedMTU(0)
{
}

// ---------------------------------------------------------------------------
// Public API
// ---------------------------------------------------------------------------

void BLEDriver::onWrite(WriteCallback cb)
{
    _writeCallback = cb;
}

void BLEDriver::begin()
{
    NimBLEDevice::init(_deviceName);
    NimBLEDevice::setPower(_txPower);  // 2.x: takes int8_t dBm directly (no esp_power_level_t)
    NimBLEDevice::setMTU(_mtu);

    // Server
    _pServer = NimBLEDevice::createServer();
    _pServer->setCallbacks(new IgnisServerCallbacks(this));

    // 2.x: advertising is NOT auto-restarted on disconnect by default.
    // advertiseOnDisconnect(true) re-enables this behaviour.
    _pServer->advertiseOnDisconnect(true);

    // Service + characteristic
    // 2.x: pService->start() has been removed — services start with the server.
    NimBLEService* pService = _pServer->createService(_serviceUUID);
    _pCharacteristic = pService->createCharacteristic(
        _charUUID,
        NIMBLE_PROPERTY::WRITE    |
        NIMBLE_PROPERTY::WRITE_NR |
        NIMBLE_PROPERTY::INDICATE,
        _mtu * 2
    );
    _pCharacteristic->setCallbacks(new IgnisCharCallbacks(this));

    // Advertising
    // 2.x: setAdvertisementData replaces all other ad data, so everything
    // must be set inside NimBLEAdvertisementData structs.
    // 2.x: setScanResponse → enableScanResponse, setMinPreferred/setMaxPreferred
    //      → setPreferredParams.
    // 2.x: device name and TX power are NOT advertised by default.
    NimBLEAdvertising* pAdvertising = NimBLEDevice::getAdvertising();

    NimBLEAdvertisementData advData;
    advData.setFlags(BLE_HS_ADV_F_DISC_GEN | BLE_HS_ADV_F_BREDR_UNSUP);
    advData.setCompleteServices(NimBLEUUID(_serviceUUID));

    NimBLEAdvertisementData scanResponse;
    scanResponse.setName(_deviceName);

    pAdvertising->setAdvertisementData(advData);
    pAdvertising->setScanResponseData(scanResponse);
    pAdvertising->enableScanResponse(true);
    pAdvertising->setPreferredParams(BLE_ADV_INTERVAL_MIN, BLE_ADV_INTERVAL_MAX);

    // 2.x: NimBLEAdvertising::start() no longer takes a callback pointer.
    // Use setAdvertisingCompleteCallback() separately if needed.
    pAdvertising->start();

    Serial.print("[BLEDriver] Advertising started — device name: ");
    Serial.println(_deviceName);
}

void BLEDriver::indicate(const uint8_t* data, size_t length)
{
    if (!isConnected() || !_pCharacteristic || !data || length == 0) return;
    _pCharacteristic->setValue(data, length);
    _pCharacteristic->indicate();
}

void BLEDriver::indicate(const char* str)
{
    if (!str) return;
    indicate((const uint8_t*)str, strlen(str));
}

bool BLEDriver::isConnected() const
{
    // 2.x: use getConnectedCount() rather than tracking a bool manually.
    // This correctly handles multiple simultaneous connections.
    if (!_pServer) return false;
    return _pServer->getConnectedCount() > 0;
}

uint16_t BLEDriver::getMTU() const
{
    // MTU is captured in _onMTUChange and stored in _negotiatedMTU.
    // Returns 0 if no MTU exchange has occurred yet.
    return _negotiatedMTU;
}

// ---------------------------------------------------------------------------
// Internal callbacks (forwarded from NimBLE shim classes)
// ---------------------------------------------------------------------------

void BLEDriver::_onConnect(NimBLEServer* pServer, NimBLEConnInfo& connInfo)
{
    Serial.print("[BLEDriver] Client connected — address: ");
    Serial.println(connInfo.getAddress().toString().c_str());
}

void BLEDriver::_onDisconnect(NimBLEServer* pServer, NimBLEConnInfo& connInfo, int reason)
{
    _negotiatedMTU = 0;
    Serial.print("[BLEDriver] Client disconnected — reason: ");
    Serial.println(reason);
    // advertiseOnDisconnect(true) set in begin() handles restart automatically.
}

void BLEDriver::_onMTUChange(uint16_t mtu, NimBLEConnInfo& connInfo)
{
    _negotiatedMTU = mtu;
    Serial.print("[BLEDriver] MTU updated: ");
    Serial.println(mtu);
}

void BLEDriver::_onWrite(NimBLECharacteristic* pChar)
{
    if (!_writeCallback) return;
    // 2.x: getValue() returns NimBLEAttValue; use data()/length() or cast to std::string
    std::string value = pChar->getValue();
    if (value.empty()) return;
    _writeCallback((uint8_t*)value.data(), value.length());
}