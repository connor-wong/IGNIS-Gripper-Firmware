#include "motor_driver.h"

// ---------------------------------------------------------------------------
// Construction
// ---------------------------------------------------------------------------

MotorDriver::MotorDriver(uint8_t in1Pin, uint8_t in2Pin,
                         uint8_t pwmCh1, uint8_t pwmCh2)
    : _in1Pin(in1Pin), _in2Pin(in2Pin),
      _pwmCh1(pwmCh1), _pwmCh2(pwmCh2),
      _speed(0)
{
}

// ---------------------------------------------------------------------------
// Public API
// ---------------------------------------------------------------------------

void MotorDriver::begin()
{
    // Attach LEDC channels before driving anything
    ledcSetup(_pwmCh1, MOTOR_PWM_FREQ, MOTOR_PWM_RESOLUTION);
    ledcSetup(_pwmCh2, MOTOR_PWM_FREQ, MOTOR_PWM_RESOLUTION);
    ledcAttachPin(_in1Pin, _pwmCh1);
    ledcAttachPin(_in2Pin, _pwmCh2);

    coast(); // Safe initial state
}

void MotorDriver::setSpeed(int16_t speed)
{
    // Clamp to valid range
    speed = constrain(speed, -MOTOR_PWM_MAX, MOTOR_PWM_MAX);
    _speed = speed;

    if (speed > 0) {
        // Forward: IN1=PWM, IN2=0
        writePwm((uint8_t)speed, 0);
    } else if (speed < 0) {
        // Reverse: IN1=0, IN2=PWM
        writePwm(0, (uint8_t)(-speed));
    } else {
        coast();
    }
}

void MotorDriver::brake()
{
    _speed = 0;
    writePwm(MOTOR_PWM_MAX, MOTOR_PWM_MAX); // IN1=1, IN2=1 → short-brake
}

void MotorDriver::coast()
{
    _speed = 0;
    writePwm(0, 0); // IN1=0, IN2=0 → freewheel
}

int16_t MotorDriver::getSpeed() const
{
    return _speed;
}

// ---------------------------------------------------------------------------
// Private helpers
// ---------------------------------------------------------------------------

void MotorDriver::writePwm(uint8_t duty1, uint8_t duty2)
{
    ledcWrite(_pwmCh1, duty1);
    ledcWrite(_pwmCh2, duty2);
}