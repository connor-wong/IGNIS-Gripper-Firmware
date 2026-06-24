#include <Arduino.h>
#include <NimBLEDevice.h>

/* Custom Libraries */
#include "imu_driver.h"
#include "motor_driver.h"
#include "encoder_driver.h"
// #include "mlx90614_driver.h"
#include "mcp9808_driver.h"
#include "vl53l5cx_driver.h"

// ====== Forward Declarations ======
void handleCharacteristicWrite(uint8_t *data, size_t length);
void setupBLE();

// ====== BLE Configuration ======
#define SERVICE_UUID        "12345678-1234-5678-1234-56789abcdef0"
#define CHARACTERISTIC_UUID "12345678-1234-5678-1234-56789abcdef1"
#define MTU_SIZE            512

// ====== Callback Classes ======
class MyCharacteristicCallback : public NimBLECharacteristicCallbacks {
public:
    void onWrite(NimBLECharacteristic *pCharacteristic, NimBLEConnInfo &connInfo) override {
        std::string value = pCharacteristic->getValue();
        handleCharacteristicWrite((uint8_t*)value.data(), value.length());
    }
};

class MyServerCallbacks : public NimBLEServerCallbacks {
public:
    void onConnect(NimBLEServer* pServer, NimBLEConnInfo& connInfo) override {
        Serial.println("\n📡 BLE Client Connected!");
    }

    void onDisconnect(NimBLEServer* pServer, NimBLEConnInfo& connInfo, int reason) override {
        Serial.println("\n📴 BLE Client Disconnected!");
        NimBLEDevice::startAdvertising();
    }
};

// ====== Data Handler ======
void handleCharacteristicWrite(uint8_t *data, size_t length) {
    if (length == 0) return;
}

// ====== BLE Setup ======
void setupBLE() {
    NimBLEDevice::init("IGNIS");
    NimBLEDevice::setPower(9);
    NimBLEDevice::setMTU(MTU_SIZE);

    NimBLEServer *pServer = NimBLEDevice::createServer();
    pServer->setCallbacks(new MyServerCallbacks());

    NimBLEService *pService = pServer->createService(SERVICE_UUID);

    NimBLECharacteristic *pCharacteristic = pService->createCharacteristic(
        CHARACTERISTIC_UUID,
        NIMBLE_PROPERTY::WRITE | NIMBLE_PROPERTY::WRITE_NR | NIMBLE_PROPERTY::INDICATE,
        MTU_SIZE * 2);

    pCharacteristic->setCallbacks(new MyCharacteristicCallback());

    NimBLEAdvertising *pAdvertising = NimBLEDevice::getAdvertising();
    NimBLEAdvertisementData advData;
    advData.setFlags(BLE_HS_ADV_F_DISC_GEN | BLE_HS_ADV_F_BREDR_UNSUP);
    advData.setCompleteServices(NimBLEUUID(SERVICE_UUID));

    NimBLEAdvertisementData scanResponse;
    scanResponse.setName("IGNIS");

    pAdvertising->setAdvertisementData(advData);
    pAdvertising->setScanResponseData(scanResponse);
    pAdvertising->enableScanResponse(true);
    pAdvertising->setPreferredParams(0x06, 0x12);
    pAdvertising->start();

    Serial.println("\n✓ BLE Advertising started");
    Serial.println("  Device Name: IGNIS");
}

// ---------------------------------------------------------------------------
// Motor and Encoder
// ---------------------------------------------------------------------------

// Motor Driver
MotorDriver motor(4, 5, 2, 3); // IN1=GPIO4, IN2=GPIO5, LEDC channels 2 & 3

// Encoder Driver
EncoderDriver encoder(18, 19);   // yellow → 18, green → 19

// ---------------------------------------------------------------------------
// I2C Drivers (shared SDA=GPIO8, SCL=GPIO9)
// ---------------------------------------------------------------------------

#define SDA_PIN 8
#define SCL_PIN 9

// IMU Driver | I2C address 0x68
ImuDriver imu(SDA_PIN, SCL_PIN, MPU_DEFAULT_ADDR);

// MLX90614 Driver | I2C address 0x5A
// MLX90614Driver objTemp(SDA_PIN, SCL_PIN, MLX90614_DEFAULT_ADDR);

// MCP9808 Driver | I2C address 0x18
MCP9808Driver pcbTemp(SDA_PIN, SCL_PIN, MCP9808_DEFAULT_ADDR);

// VL53L5CX Driver | I2C address 0x52 (8-bit) or 0x29 (7-bit)
VL53L5CXDriver tof(SDA_PIN, SCL_PIN, 5, VL53L5CX_RES_8X8, 15);

const unsigned long LOOP_INTERVAL_MS = 10;
unsigned long previousMillis = 0;

void setup() {
    Serial.begin(115200);

    // ---------------------------------------------------------------------------
    // Sensors Initialization
    // ---------------------------------------------------------------------------

    if (!imu.begin()) {
        Serial.println("IMU init failed");
        while (true) {}   // Halt; nothing useful to do without the sensor
    }

    Serial.println("MPU initialized.");
    imu.calibrate();

    if (!tof.begin()) {
        Serial.println("VL53L5CX init failed");
        while (true) {}   // Halt; nothing useful to do without the sensor
    }

    // objTemp.begin();
    // pcbTemp.begin();
    // pcbTemp.setResolution(MCP9808_RES_0_0625);

    // ---------------------------------------------------------------------------
    // Motor and Encoder Initialization
    // ---------------------------------------------------------------------------

    // motor.begin();
    // encoder.begin();
    // motor.setSpeed(180);
}

void loop() {
    unsigned long now = millis();

    if (now - previousMillis >= LOOP_INTERVAL_MS) {
        previousMillis = now;

        ImuData data;
        imu.read(data);
        ImuDriver::printJson(data);
    }

    // encoder.update();
    // Serial.println(encoder.getRPM());
    
    // objTemp.printJson();
    // pcbTemp.printJson();
    // tof.printJson();
}