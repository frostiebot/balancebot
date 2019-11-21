#ifndef SERVO_H
#define SERVO_H

#include <driver/mcpwm.h>

#define SERVO_MIN_DUTY_uS  (500)  // Minimum pulse width in microsecond
#define SERVO_MAX_DUTY_uS  (2500) // Maximum pulse width in microsecond
#define SERVO_MAX_ANGLE    (180)  // Maximum angle in degree upto which servo can rotate
#define SERVO_FREQUENCY    (50)

typedef struct {
  uint32_t min_duty_us;
  uint32_t max_duty_us;
  uint32_t max_angle;
  mcpwm_config_t mcpwm_config;
} servo_config_t;


class Servo {
  public:
    Servo();

    void attach(gpio_num_t pin);
    void disable();

    float getDuty();

    void setAngle(uint32_t angle);
    void setDuty(uint32_t duty);
    void setDutyPercent(float duty);

  private:
    servo_config_t config = {
      SERVO_MIN_DUTY_uS,
      SERVO_MAX_DUTY_uS,
      SERVO_MAX_ANGLE,
      {
        SERVO_FREQUENCY,
        0,
        0,
        MCPWM_DUTY_MODE_0,
        MCPWM_UP_COUNTER
      }
    };

    mcpwm_unit_t pwmUnit = MCPWM_UNIT_0;
    mcpwm_timer_t pwmTimer = MCPWM_TIMER_0;
    mcpwm_operator_t pwmOperator = MCPWM_OPR_A;
    mcpwm_io_signals_t pwmSignal = MCPWM0A;
};

#endif
