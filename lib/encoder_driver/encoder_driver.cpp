#include "encoder_driver.h"

// ---------------------------------------------------------------------------
// Static ISR trampoline
//
// ESP32 ISRs must be free functions (or static). A single static pointer lets
// the free-function ISRs forward to the correct instance method.
// Only one EncoderDriver instance is supported per program.
// ---------------------------------------------------------------------------
static EncoderDriver* _instance = nullptr;

static void IRAM_ATTR isrA() { if (_instance) _instance->handleA(); }
static void IRAM_ATTR isrB() { if (_instance) _instance->handleB(); }

// ---------------------------------------------------------------------------
// Construction
// ---------------------------------------------------------------------------

EncoderDriver::EncoderDriver(uint8_t pinA, uint8_t pinB,
                             uint16_t ppr, uint16_t gearRatio)
    : _pinA(pinA), _pinB(pinB),
      _ppr(ppr), _gearRatio(gearRatio),
      _cpr((int32_t)ppr * gearRatio * 4),
      _ticks(0),
      _lastTicks(0), _lastVelocityMs(0), _rpm(0.0f),
      _lastA(0), _lastB(0)
{
}

// ---------------------------------------------------------------------------
// Public API
// ---------------------------------------------------------------------------

void EncoderDriver::begin()
{
    _instance = this;

    pinMode(_pinA, INPUT);
    pinMode(_pinB, INPUT);

    // Sample initial pin states before enabling interrupts
    _lastA = digitalRead(_pinA);
    _lastB = digitalRead(_pinB);

    attachInterrupt(digitalPinToInterrupt(_pinA), isrA, CHANGE);
    attachInterrupt(digitalPinToInterrupt(_pinB), isrB, CHANGE);

    _lastVelocityMs = millis();
}

void EncoderDriver::update()
{
    uint32_t now = millis();
    uint32_t elapsed = now - _lastVelocityMs;

    if (elapsed < ENCODER_VELOCITY_WINDOW_MS) return;

    // Atomic snapshot of tick count
    noInterrupts();
    int32_t currentTicks = _ticks;
    interrupts();

    int32_t deltaTicks = currentTicks - _lastTicks;
    // Convert ticks/window → RPM
    // deltaTicks / _cpr = output shaft revolutions
    // elapsed / 1000.0  = window in seconds
    // × 60              = revolutions per minute
    _rpm = ((float)deltaTicks / (float)_cpr) * (60000.0f / (float)elapsed);

    _lastTicks      = currentTicks;
    _lastVelocityMs = now;
}

int32_t EncoderDriver::getTicks() const
{
    noInterrupts();
    int32_t t = _ticks;
    interrupts();
    return t;
}

float EncoderDriver::getRevolutions() const
{
    return (float)getTicks() / (float)_cpr;
}

float EncoderDriver::getDegrees() const
{
    float deg = fmod(getRevolutions() * 360.0f, 360.0f);
    if (deg < 0.0f) deg += 360.0f;
    return deg;
}

float EncoderDriver::getRPM() const
{
    return _rpm;
}

void EncoderDriver::reset()
{
    noInterrupts();
    _ticks = 0;
    interrupts();
    _lastTicks      = 0;
    _rpm            = 0.0f;
    _lastVelocityMs = millis();
}

// ---------------------------------------------------------------------------
// ISR handlers — quadrature x4 decoding
//
// State table for AB quadrature (x4):
//   lastA lastB | currA currB | direction
//     0     0   |   1     0   |   +1
//     0     0   |   0     1   |   -1
//     1     0   |   1     1   |   +1
//     1     0   |   0     0   |   -1
//     1     1   |   0     1   |   +1
//     1     1   |   1     0   |   -1
//     0     1   |   0     0   |   +1
//     0     1   |   1     1   |   -1
// ---------------------------------------------------------------------------

void IRAM_ATTR EncoderDriver::handleA()
{
    uint8_t currA = digitalRead(_pinA);
    // Direction: A changed — forward if A != B, reverse if A == B
    if (currA != _lastB) {
        _ticks++;
    } else {
        _ticks--;
    }
    _lastA = currA;
}

void IRAM_ATTR EncoderDriver::handleB()
{
    uint8_t currB = digitalRead(_pinB);
    // Direction: B changed — forward if A == B, reverse if A != B
    if (_lastA == currB) {
        _ticks++;
    } else {
        _ticks--;
    }
    _lastB = currB;
}