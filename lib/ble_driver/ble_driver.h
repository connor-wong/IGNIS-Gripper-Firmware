#pragma once

#include <Arduino.h>
#include <NimBLEDevice.h>

// Default BLE configuration
#define BLE_DEFAULT_DEVICE_NAME     "IGNIS"
#define BLE_DEFAULT_TX_POWER        9           // dBm (NimBLE range: -12 to 9)
#define BLE_DEFAULT_MTU             512
#define BLE_DEFAULT_SERVICE_UUID    "12345678-1234-5678-1234-56789abcdef0"
#define BLE_DEFAULT_CHAR_UUID       "12345678-1234-5678-1234-56789abcdef1"

// Advertising interval range (units of 0.625ms)
// 0x06 = 3.75ms (min), 0x12 = 11.25ms (max) — matches original config
#define BLE_ADV_INTERVAL_MIN        0x06
#define BLE_ADV_INTERVAL_MAX        0x12

/**
 * BLEDriver
 *
 * Wraps NimBLE into a single class that manages:
 *   - Server + advertising lifecycle
 *   - One service with one WRITE + INDICATE characteristic
 *   - Connection/disconnection callbacks (auto-restarts advertising on disconnect)
 *   - Inbound write handler via user-supplied callback
 *   - Outbound indicate() for sending data back to connected client
 *
 * Usage:
 *   1. Implement a write handler:
 *        void onReceive(uint8_t* data, size_t len) { ... }
 *
 *   2. Instantiate and call begin():
 *        BLEDriver ble;
 *        ble.onWrite(onReceive);
 *        ble.begin();
 *
 *   3. Send data to client:
 *        ble.indicate((uint8_t*)json, strlen(json));
 *
 * Notes:
 *   - INDICATE (not NOTIFY) is used — the client must ACK each packet.
 *     This provides reliability at the cost of slightly lower throughput.
 *   - MTU negotiation happens automatically; effective payload per packet
 *     is MTU - 3 bytes (ATT header overhead).
 *   - If no client is connected, indicate() is a no-op.
 */
class BLEDriver {
public:
    // Type alias for the inbound write callback
    using WriteCallback = void(*)(uint8_t* data, size_t length);

    /**
     * @param deviceName  BLE advertised device name
     * @param serviceUUID Service UUID string
     * @param charUUID    Characteristic UUID string
     * @param txPower     Transmit power in dBm (-12 to 9)
     * @param mtu         ATT MTU size (max packet size including header)
     */
    BLEDriver(const char* deviceName = BLE_DEFAULT_DEVICE_NAME,
              const char* serviceUUID = BLE_DEFAULT_SERVICE_UUID,
              const char* charUUID    = BLE_DEFAULT_CHAR_UUID,
              int8_t      txPower     = BLE_DEFAULT_TX_POWER,
              uint16_t    mtu         = BLE_DEFAULT_MTU);

    /**
     * Registers a callback invoked whenever the client writes to the
     * characteristic. Must be called before begin().
     *
     * @param cb  Function pointer: void cb(uint8_t* data, size_t length)
     */
    void onWrite(WriteCallback cb);

    /**
     * Initialises NimBLE, creates the server/service/characteristic,
     * configures advertising, and starts advertising.
     * Call once in setup().
     */
    void begin();

    /**
     * Sends data to the connected client via INDICATE.
     * No-op if no client is connected or data is null/empty.
     * For payloads larger than (MTU - 3) bytes, the NimBLE stack
     * handles fragmentation automatically.
     *
     * @param data    Byte buffer to send
     * @param length  Number of bytes
     */
    void indicate(const uint8_t* data, size_t length);

    /**
     * Convenience overload — sends a null-terminated C string.
     * Useful for JSON packets.
     *
     * @param str  Null-terminated string to send
     */
    void indicate(const char* str);

    /** Returns true if a BLE client is currently connected. */
    bool isConnected() const;

    /** Returns the negotiated MTU for the current connection (0 if not connected). */
    uint16_t getMTU() const;

    // --- Internal use by callback classes (must be public for NimBLE shims) ---
    void _onConnect(NimBLEServer* pServer, NimBLEConnInfo& connInfo);
    void _onDisconnect(NimBLEServer* pServer, NimBLEConnInfo& connInfo, int reason);
    void _onMTUChange(uint16_t mtu, NimBLEConnInfo& connInfo);  // 2.x: renamed from onMTUChanged
    void _onWrite(NimBLECharacteristic* pChar);

private:
    const char*   _deviceName;
    const char*   _serviceUUID;
    const char*   _charUUID;
    int8_t        _txPower;
    uint16_t      _mtu;

    WriteCallback _writeCallback;

    NimBLEServer*         _pServer;
    NimBLECharacteristic* _pCharacteristic;

    // 2.x: connection state is queried via _pServer->getConnectedCount() rather
    // than a manually tracked bool, which correctly handles multiple connections.
    // MTU is captured in _onMTUChange and stored here for getMTU() queries.
    uint16_t _negotiatedMTU;
};