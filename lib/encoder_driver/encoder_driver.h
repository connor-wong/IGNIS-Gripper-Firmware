#pragma once

#include <Arduino.h>

// JGB37-520 encoder constants
#define ENCODER_PPR             11      // Pulses per motor shaft revolution
#define ENCODER_DEFAULT_RATIO   158     // Approximate gear reduction for 38RPM/6V variant
#define ENCODER_CPR(ratio)      ((ENCODER_PPR) * (ratio) * 4)  // Counts per output shaft rev (quadrature x4)

// Velocity estimation window (ms) — shorter = more responsive, noisier
#define ENCODER_VELOCITY_WINDOW_MS  100

/**
 * EncoderDriver
 *
 * Reads the AB quadrature Hall encoder on the JGB37-520.
 *
 * Wiring:
 *   Blue   → 3.3V (encoder power +)
 *   Black  → GND  (encoder power -)
 *   Yellow → pinA (C1/A phase, interrupt-capable GPIO)
 *   Green  → pinB (C2/B phase, interrupt-capable GPIO)
 *   Red    → motor power +  (to DRV8833 AOUT1/BOUT1)
 *   White  → motor power -  (to DRV8833 AOUT2/BOUT2)
 *
 * Quadrature decoding:
 *   Both edges of both channels are captured (x4 decoding) for maximum
 *   resolution: 11 PPR × gear ratio × 4 = counts per output shaft revolution.
 *
 * Thread safety:
 *   _ticks is volatile and updated only in ISRs. Read via getTicks() which
 *   performs an atomic snapshot via noInterrupts()/interrupts().
 */
class EncoderDriver {
public:
    /**
     * @param pinA      GPIO for encoder channel A (yellow wire) — must support interrupts
     * @param pinB      GPIO for encoder channel B (green wire)  — must support interrupts
     * @param ppr       Pulses per motor revolution (11 for JGB37-520)
     * @param gearRatio Gear reduction ratio of your specific variant (e.g. 158 for 38RPM)
     */
    EncoderDriver(uint8_t pinA, uint8_t pinB,
                  uint16_t ppr = ENCODER_PPR,
                  uint16_t gearRatio = ENCODER_DEFAULT_RATIO);

    /**
     * Configures pins and attaches interrupts. Call once in setup().
     * NOTE: Only one EncoderDriver instance is supported per program due to
     * ISR linkage constraints on Arduino/ESP32.
     */
    void begin();

    /**
     * Must be called regularly in loop() to update the velocity estimate.
     * Recalculates RPM every ENCODER_VELOCITY_WINDOW_MS milliseconds.
     */
    void update();

    /** Returns cumulative tick count since begin() or last reset. Signed: + = forward. */
    int32_t getTicks() const;

    /** Returns output shaft position in fractional revolutions since reset. */
    float getRevolutions() const;

    /** Returns output shaft angle in degrees [0, 360). */
    float getDegrees() const;

    /** Returns smoothed output shaft speed in RPM. Positive = forward. */
    float getRPM() const;

    /** Resets tick count and velocity state to zero. */
    void reset();

    // --- ISR handlers (public so static trampolines can reach them) ---
    void handleA();
    void handleB();

private:
    uint8_t  _pinA;
    uint8_t  _pinB;
    uint16_t _ppr;
    uint16_t _gearRatio;
    int32_t  _cpr;          // Counts per output revolution (ppr × ratio × 4)

    volatile int32_t _ticks;        // Raw quadrature count

    // Velocity estimation state
    int32_t  _lastTicks;
    uint32_t _lastVelocityMs;
    float    _rpm;

    // Previous pin states for quadrature decoding
    volatile uint8_t _lastA;
    volatile uint8_t _lastB;
};