#pragma once
#include <Arduino.h>
#include <driver/ledc.h>

/*
  BTS7960 / IBT-2 dual-EN wiring (your working test):
    RPWM -> GPIO25  (LEDC PWM)
    LPWM -> GPIO26  (LEDC PWM)
    R_EN -> GPIO27
    L_EN -> GPIO14
*/

// ---------------- Pin map ----------------
#ifndef BTS_RPWM_PIN
#define BTS_RPWM_PIN  25
#endif
#ifndef BTS_LPWM_PIN
#define BTS_LPWM_PIN  26
#endif
#ifndef BTS_R_EN_PIN
#define BTS_R_EN_PIN  27
#endif
#ifndef BTS_L_EN_PIN
#define BTS_L_EN_PIN  14
#endif

// ---------------- LEDC config ----------------
#ifndef BTS_PWM_FREQ
#define BTS_PWM_FREQ       20000   // 20 kHz default
#endif
#ifndef BTS_PWM_RES_BITS
#define BTS_PWM_RES_BITS   8       // 0..255
#endif
#ifndef BTS_PWM_TIMER
#define BTS_PWM_TIMER      LEDC_TIMER_0
#endif
#ifndef BTS_PWM_MODE
#define BTS_PWM_MODE       LEDC_LOW_SPEED_MODE
#endif
#ifndef BTS_PWM_CH_R
#define BTS_PWM_CH_R       LEDC_CHANNEL_0   // RPWM
#endif
#ifndef BTS_PWM_CH_L
#define BTS_PWM_CH_L       LEDC_CHANNEL_1   // LPWM
#endif

#define BTS_PWM_MAX        ((1U << BTS_PWM_RES_BITS) - 1)

// Default duty (≈70%)
#ifndef BTS_DEFAULT_DUTY
#define BTS_DEFAULT_DUTY   (BTS_PWM_MAX * 7 / 10)
#endif

// -------- API --------
void motor_bts7960_begin();
void motor_bts7960_stop();                  // coast (RPWM=LPWM=0)
void motor_bts7960_brake();                 // both high (active brake)
void motor_bts7960_right(uint16_t duty);    // RPWM=duty, LPWM=0
void motor_bts7960_left(uint16_t duty);     // RPWM=0,    LPWM=duty

void motor_bts7960_set_right(bool en, uint16_t duty);
void motor_bts7960_set_left(bool en, uint16_t duty);

bool     motor_bts7960_set_pwm_freq(uint32_t freq_hz);
void     motor_bts7960_set_default_duty(uint16_t duty);
uint16_t motor_bts7960_get_default_duty();
