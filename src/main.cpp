#include <Arduino.h>

/* Custom Drivers */
#include "ble_driver.h"
#include "imu_driver.h"
#include "motor_driver.h"
#include "encoder_driver.h"
// #include "mlx90614_driver.h"
#include "mcp9808_driver.h"
#include "vl53l5cx_driver.h"

// ====== BLE Configuration ======
#define SERVICE_UUID        "12345678-1234-5678-1234-56789abcdef0"
#define CHARACTERISTIC_UUID "12345678-1234-5678-1234-56789abcdef1"
#define MTU_SIZE            512

BLEDriver ble("IGNIS", SERVICE_UUID, CHARACTERISTIC_UUID, MTU_SIZE);

// ---------------------------------------------------------------------------
// Motor and Encoder
// ---------------------------------------------------------------------------

MotorDriver   motor(4, 5, 2, 3);    // IN1=GPIO4, IN2=GPIO5, LEDC ch 2 & 3
EncoderDriver encoder(18, 19);       // yellow → GPIO18, green → GPIO19

// ---------------------------------------------------------------------------
// I2C Drivers (shared SDA=GPIO8, SCL=GPIO9)
// ---------------------------------------------------------------------------

#define SDA_PIN 8
#define SCL_PIN 9

ImuDriver      imu(SDA_PIN, SCL_PIN, MPU_DEFAULT_ADDR);
// MLX90614Driver objTemp(SDA_PIN, SCL_PIN, MLX90614_DEFAULT_ADDR);
MCP9808Driver  pcbTemp(SDA_PIN, SCL_PIN, MCP9808_DEFAULT_ADDR);
VL53L5CXDriver tof(SDA_PIN, SCL_PIN, 5, VL53L5CX_RES_8X8, 15);

// ---------------------------------------------------------------------------
// BLE write handler — forward declared so the lambda can reference it
// ---------------------------------------------------------------------------

void sendCommandResponse(const char* command, bool ok, const char* state, const char* error, bool streaming);
void handleBLEWrite(uint8_t* data, size_t length);
void handleSerialCommands();
void handleCommand(const char* rawCommand);

// ---------------------------------------------------------------------------
// Timing
// ---------------------------------------------------------------------------

const unsigned long LOOP_INTERVAL_MS = 33;  // ~30 Hz BLE/Serial IMU stream
unsigned long previousMillis = 0;
uint16_t imuPacketSequence = 0;
bool imuStreamingEnabled = false;
String serialCommandBuffer;

struct __attribute__((packed)) CompactImuPacket {
    uint8_t  magic0;       // 'I'
    uint8_t  magic1;       // 'M'
    uint8_t  version;      // packet format version
    uint8_t  flags;        // reserved for future sensors/status bits
    uint16_t sequence;     // increments every IMU packet
    int16_t  axMg;         // accelerometer X in milli-g
    int16_t  ayMg;         // accelerometer Y in milli-g
    int16_t  azMg;         // accelerometer Z in milli-g
    int16_t  gxDps10;      // gyro X in deg/s * 10
    int16_t  gyDps10;      // gyro Y in deg/s * 10
    int16_t  gzDps10;      // gyro Z in deg/s * 10
};

static_assert(sizeof(CompactImuPacket) == 18, "Compact IMU packet must stay 18 bytes");

int16_t clampToInt16(float value)
{
    if (value > 32767.0f) return 32767;
    if (value < -32768.0f) return -32768;
    return static_cast<int16_t>(lroundf(value));
}

CompactImuPacket buildCompactImuPacket(const ImuData& data)
{
    CompactImuPacket packet{};
    packet.magic0 = 'I';
    packet.magic1 = 'M';
    packet.version = 1;
    packet.flags = 0;
    packet.sequence = imuPacketSequence++;
    packet.axMg = clampToInt16(data.accelX * 1000.0f);
    packet.ayMg = clampToInt16(data.accelY * 1000.0f);
    packet.azMg = clampToInt16(data.accelZ * 1000.0f);
    packet.gxDps10 = clampToInt16(data.gyroX * 10.0f);
    packet.gyDps10 = clampToInt16(data.gyroY * 10.0f);
    packet.gzDps10 = clampToInt16(data.gyroZ * 10.0f);
    return packet;
}

// ---------------------------------------------------------------------------
// Setup
// ---------------------------------------------------------------------------

void setup()
{
    Serial.begin(115200);

    // IMU
    if (!imu.begin()) {
        Serial.println("IMU init failed");
        while (true) {}
    }
    Serial.println("MPU initialized.");
    imu.calibrate();

    // BLE
    ble.onWrite(   [](uint8_t* data, size_t len) { handleBLEWrite(data, len); });
    ble.onConnect(    []() { Serial.println("\n📡 BLE Client Connected!");    });
    ble.onDisconnect( []() { Serial.println("\n📴 BLE Client Disconnected!"); });
    ble.begin();

    Serial.println("✓ System initialized and ready\n");

    // ToF
    // if (!tof.begin()) {
    //     Serial.println("VL53L5CX init failed");
    //     while (true) {}
    // }

    // Temperature sensors
    // objTemp.begin();
    // pcbTemp.begin();
    // pcbTemp.setResolution(MCP9808_RES_0_0625);

    // Motor and encoder
    // motor.begin();
    // encoder.begin();
    // motor.setSpeed(180);
}

// ---------------------------------------------------------------------------
// Loop
// ---------------------------------------------------------------------------

void loop()
{
    handleSerialCommands();

    unsigned long now = millis();

    if (imuStreamingEnabled && now - previousMillis >= LOOP_INTERVAL_MS) {
        previousMillis = now;

        ImuData data;
        imu.read(data);

        char buf[128];
        snprintf(buf, sizeof(buf),
            "{\"ax\":%.3f,\"ay\":%.3f,\"az\":%.3f,"
            "\"gx\":%.3f,\"gy\":%.3f,\"gz\":%.3f}",
            data.accelX, data.accelY, data.accelZ,
            data.gyroX,  data.gyroY,  data.gyroZ);

        if (ble.isConnected()) {
            CompactImuPacket packet = buildCompactImuPacket(data);
            ble.notify(reinterpret_cast<const uint8_t*>(&packet), sizeof(packet));
        } else {
            Serial.println(buf);    // Keep USB Serial JSON fallback for debugging
        }
    }

    // encoder.update();
    // objTemp.printJson();
    // pcbTemp.printJson();
    // tof.printJson();
}

// ---------------------------------------------------------------------------
// BLE write handler — receives commands from connected client
// ---------------------------------------------------------------------------

void sendCommandResponse(const char* command, bool ok, const char* state, const char* error, bool streaming)
{
    char response[160];

    if (error) {
        snprintf(response, sizeof(response),
            "{\"command\":\"%s\",\"ok\":false,\"error\":\"%s\",\"streaming\":%s}",
            command, error, streaming ? "true" : "false");
    } else if (state) {
        snprintf(response, sizeof(response),
            "{\"command\":\"%s\",\"ok\":%s,\"state\":\"%s\",\"streaming\":%s}",
            command, ok ? "true" : "false", state, streaming ? "true" : "false");
    } else {
        snprintf(response, sizeof(response),
            "{\"command\":\"%s\",\"ok\":%s,\"streaming\":%s}",
            command, ok ? "true" : "false", streaming ? "true" : "false");
    }

    Serial.println(response);
    if (ble.isConnected()) {
        ble.notify(response);
    }
}

void handleBLEWrite(uint8_t* data, size_t length)
{
    if (!data || length == 0) return;

    char command[32];
    size_t copyLength = min(length, sizeof(command) - 1);
    memcpy(command, data, copyLength);
    command[copyLength] = '\0';
    handleCommand(command);
}

void handleSerialCommands()
{
    while (Serial.available() > 0) {
        char c = static_cast<char>(Serial.read());
        if (c == '\n' || c == '\r') {
            if (serialCommandBuffer.length() > 0) {
                handleCommand(serialCommandBuffer.c_str());
                serialCommandBuffer = "";
            }
        } else if (serialCommandBuffer.length() < 31) {
            serialCommandBuffer += c;
        }
    }
}

void handleCommand(const char* rawCommand)
{
    if (!rawCommand) return;

    char command[32];
    size_t i = 0;
    while (rawCommand[i] != '\0' && i < sizeof(command) - 1) {
        char c = rawCommand[i];
        if (c >= 'a' && c <= 'z') c = c - 'a' + 'A';
        command[i] = c;
        i++;
    }
    command[i] = '\0';

    char* start = command;
    while (*start == ' ' || *start == '\t' || *start == '\r' || *start == '\n') start++;

    char* end = start + strlen(start);
    while (end > start && (end[-1] == ' ' || end[-1] == '\t' || end[-1] == '\r' || end[-1] == '\n')) {
        end--;
        *end = '\0';
    }

    if (strcmp(start, "START") == 0) {
        imuStreamingEnabled = true;
        previousMillis = millis();
        sendCommandResponse("START", true, nullptr, nullptr, true);
        return;
    }

    if (strcmp(start, "STOP") == 0) {
        imuStreamingEnabled = false;
        sendCommandResponse("STOP", true, nullptr, nullptr, false);
        return;
    }

    if (strcmp(start, "CALIBRATE") == 0) {
        bool wasStreaming = imuStreamingEnabled;
        imuStreamingEnabled = false;
        sendCommandResponse("CALIBRATE", true, "started", nullptr, false);
        imu.calibrate();
        imuStreamingEnabled = wasStreaming;
        sendCommandResponse("CALIBRATE", true, "complete", nullptr, imuStreamingEnabled);
        return;
    }

    sendCommandResponse(start, false, nullptr, "unknown_command", imuStreamingEnabled);
}