#include "ble_driver.h"

// ====== Inner Callback: Characteristic Write ======
class BLEDriver::CharCallbacks : public NimBLECharacteristicCallbacks {
public:
    explicit CharCallbacks(BLEDriver* driver) : _driver(driver) {}

    void onWrite(NimBLECharacteristic* pChar, NimBLEConnInfo& connInfo) override {
        if (_driver->_writeCb) {
            std::string value = pChar->getValue();
            _driver->_writeCb(
                reinterpret_cast<uint8_t*>(const_cast<char*>(value.data())),
                value.length()
            );
        }
    }

private:
    BLEDriver* _driver;
};

// ====== Inner Callback: Server Connect / Disconnect ======
class BLEDriver::ServerCallbacks : public NimBLEServerCallbacks {
public:
    explicit ServerCallbacks(BLEDriver* driver) : _driver(driver) {}

    void onConnect(NimBLEServer* pServer, NimBLEConnInfo& connInfo) override {
        _driver->_connected = true;
        if (_driver->_connectCb) _driver->_connectCb();
    }

    void onDisconnect(NimBLEServer* pServer, NimBLEConnInfo& connInfo, int reason) override {
        _driver->_connected = false;
        if (_driver->_disconnectCb) _driver->_disconnectCb();
        NimBLEDevice::startAdvertising();
    }

private:
    BLEDriver* _driver;
};

// ====== Constructor ======
BLEDriver::BLEDriver(const char* deviceName,
                     const char* serviceUUID,
                     const char* charUUID,
                     uint16_t mtuSize)
    : _deviceName(deviceName)
    , _serviceUUID(serviceUUID)
    , _charUUID(charUUID)
    , _mtuSize(mtuSize)
{}

// ====== Callback Setters ======
void BLEDriver::onWrite(WriteCallback cb)           { _writeCb      = cb; }
void BLEDriver::onConnect(ConnectionCallback cb)    { _connectCb    = cb; }
void BLEDriver::onDisconnect(ConnectionCallback cb) { _disconnectCb = cb; }

// ====== begin() ======
void BLEDriver::begin() {
    NimBLEDevice::init(_deviceName);
    NimBLEDevice::setPower(9);
    NimBLEDevice::setMTU(_mtuSize);

    NimBLEServer* pServer = NimBLEDevice::createServer();
    pServer->setCallbacks(new ServerCallbacks(this));

    NimBLEService* pService = pServer->createService(_serviceUUID);

    NimBLECharacteristic* pChar = pService->createCharacteristic(
        _charUUID,
        NIMBLE_PROPERTY::WRITE | NIMBLE_PROPERTY::WRITE_NR | NIMBLE_PROPERTY::NOTIFY | NIMBLE_PROPERTY::INDICATE,
        _mtuSize * 2
    );
    pChar->setCallbacks(new CharCallbacks(this));
    _pChar = pChar;     // Store for notify()/indicate()

    // ====== Advertising ======
    NimBLEAdvertising* pAdv = NimBLEDevice::getAdvertising();

    NimBLEAdvertisementData advData;
    advData.setFlags(BLE_HS_ADV_F_DISC_GEN | BLE_HS_ADV_F_BREDR_UNSUP);
    advData.setCompleteServices(NimBLEUUID(_serviceUUID));

    NimBLEAdvertisementData scanResp;
    scanResp.setName(_deviceName);

    pAdv->setAdvertisementData(advData);
    pAdv->setScanResponseData(scanResp);
    pAdv->enableScanResponse(true);
    pAdv->setPreferredParams(0x06, 0x12);
    pAdv->start();

    Serial.printf("\n✓ BLE started — advertising as \"%s\"\n", _deviceName);
}

// ====== notify() / indicate() ======
void BLEDriver::notify(const uint8_t* data, size_t length) {
    if (!_connected || !_pChar || !data || length == 0) return;
    _pChar->setValue(data, length);
    _pChar->notify();
}

void BLEDriver::notify(const char* str) {
    if (!str) return;
    notify(reinterpret_cast<const uint8_t*>(str), strlen(str));
}

void BLEDriver::indicate(const uint8_t* data, size_t length) {
    if (!_connected || !_pChar || !data || length == 0) return;
    _pChar->setValue(data, length);
    _pChar->indicate();
}

void BLEDriver::indicate(const char* str) {
    if (!str) return;
    indicate(reinterpret_cast<const uint8_t*>(str), strlen(str));
}

// ====== printJson() ======
void BLEDriver::printJson() const {
    Serial.printf(
        "{\"ble\":{\"name\":\"%s\",\"service\":\"%s\",\"characteristic\":\"%s\",\"mtu\":%u,\"connected\":%s}}\n",
        _deviceName,
        _serviceUUID,
        _charUUID,
        _mtuSize,
        _connected ? "true" : "false"
    );
}

// ====== isConnected() ======
bool BLEDriver::isConnected() const {
    return _connected;
}