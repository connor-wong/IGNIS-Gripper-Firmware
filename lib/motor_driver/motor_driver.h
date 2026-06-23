#pragma once

#include <Arduino.h>

// PWM configuration
#define MOTOR_PWM_FREQ      20000   // 20 kHz – above audible range
#define MOTOR_PWM_RESOLUTION    8   // 8-bit: 0–255
#define MOTOR_PWM_MAX         255

/**
 * MotorDriver
 *
 * Controls a single DC motor via one DRV8833 H-bridge channel.
 *
 * Wiring (one channel):
 *   IN1 → forward PWM input
 *   IN2 → reverse PWM input
 *
 * Drive modes supported:
 *   - Coast (IN1=0, IN2=0)  – motor freewheels
 *   - Brake (IN1=1, IN2=1)  – motor short-brakes (fast stop)
 *   - Forward / Reverse      – PWM speed control
 *
 * Speed convention: -255 (full reverse) … 0 (coast) … +255 (full forward)
 */
class MotorDriver {
public:
    /**
     * @param in1Pin  GPIO connected to DRV8833 IN1 (forward PWM)
     * @param in2Pin  GPIO connected to DRV8833 IN2 (reverse PWM)
     * @param pwmCh1  LEDC channel for IN1 (ESP32: 0–15, pick two unused ones)
     * @param pwmCh2  LEDC channel for IN2
     */
    MotorDriver(uint8_t in1Pin, uint8_t in2Pin,
                uint8_t pwmCh1 = 0, uint8_t pwmCh2 = 1);

    /** Configures GPIO and LEDC PWM channels. Call once in setup(). */
    void begin();

    /**
     * Sets motor speed and direction.
     * @param speed  -255 (full reverse) … +255 (full forward); 0 = coast
     */
    void setSpeed(int16_t speed);

    /** Active short-brake: holds rotor. Faster stop than coast. */
    void brake();

    /** Coast: both inputs low, motor freewheels to a stop. */
    void coast();

    /** Returns the last speed value set via setSpeed(). */
    int16_t getSpeed() const;

private:
    uint8_t  _in1Pin;
    uint8_t  _in2Pin;
    uint8_t  _pwmCh1;
    uint8_t  _pwmCh2;
    int16_t  _speed;    // Last commanded speed (-255 … +255)

    /** Writes duty cycles to both LEDC channels. */
    void writePwm(uint8_t duty1, uint8_t duty2);
};