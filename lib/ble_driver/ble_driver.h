#pragma once

#include <Arduino.h>
#include <NimBLEDevice.h>
#include <functional>

// ====== BLE Driver ======
// Wraps NimBLE server/characteristic setup with injectable callbacks.
// Usage:
//   BLEDriver ble("DEVICE_NAME", SERVICE_UUID, CHAR_UUID);
//   ble.onWrite([](uint8_t* data, size_t len) { ... });
//   ble.onConnect([]() { ... });
//   ble.onDisconnect([]() { ... });
//   ble.begin();

class BLEDriver {
public:
    using WriteCallback      = std::function<void(uint8_t*, size_t)>;
    using ConnectionCallback = std::function<void()>;

    // Constructor
    // deviceName   : BLE advertised name
    // serviceUUID  : 128-bit service UUID string
    // charUUID     : 128-bit characteristic UUID string
    // mtuSize      : negotiated MTU (default 512)
    BLEDriver(const char* deviceName,
              const char* serviceUUID,
              const char* charUUID,
              uint16_t mtuSize = 512);

    // Register callbacks before calling begin()
    void onWrite(WriteCallback cb);
    void onConnect(ConnectionCallback cb);
    void onDisconnect(ConnectionCallback cb);

    // Initialise and start advertising
    void begin();

    // Send data to the connected client via NOTIFY or INDICATE.
    // Prefer notify() for high-rate telemetry; use indicate() when ACK reliability matters.
    // No-op if not connected or data is null/empty.
    void notify(const uint8_t* data, size_t length);
    void notify(const char* str);
    void indicate(const uint8_t* data, size_t length);
    void indicate(const char* str);

    // Print a summary of BLE config to Serial
    void printJson() const;

    bool isConnected() const;

private:
    const char*           _deviceName;
    const char*           _serviceUUID;
    const char*           _charUUID;
    uint16_t              _mtuSize;
    bool                  _connected = false;
    NimBLECharacteristic* _pChar     = nullptr;  // Stored in begin() for notify()/indicate()

    WriteCallback      _writeCb      = nullptr;
    ConnectionCallback _connectCb    = nullptr;
    ConnectionCallback _disconnectCb = nullptr;

    // NimBLE internal callback classes — forward declared here,
    // defined in .cpp and granted friendship for private member access.
    class CharCallbacks;
    class ServerCallbacks;

    friend class CharCallbacks;
    friend class ServerCallbacks;
};