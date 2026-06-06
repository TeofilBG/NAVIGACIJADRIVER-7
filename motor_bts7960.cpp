#include "motor_bts7960.h"

static inline uint16_t _clipDuty(uint16_t d) {
  return (d > BTS_PWM_MAX) ? BTS_PWM_MAX : d;
}

static bool g_ledc_ready = false;
static uint16_t g_default_duty = BTS_DEFAULT_DUTY;

static void _ledc_init_if_needed() {
  if (g_ledc_ready) return;

  // Timer shared by both channels
  ledc_timer_config_t tcfg{};
  tcfg.speed_mode      = BTS_PWM_MODE;
  tcfg.timer_num       = BTS_PWM_TIMER;
  tcfg.duty_resolution = (ledc_timer_bit_t)BTS_PWM_RES_BITS;
  tcfg.freq_hz         = BTS_PWM_FREQ;
  tcfg.clk_cfg         = LEDC_AUTO_CLK;
  ESP_ERROR_CHECK(ledc_timer_config(&tcfg));

  // RPWM channel
  ledc_channel_config_t cr{};
  cr.gpio_num   = BTS_RPWM_PIN;
  cr.speed_mode = BTS_PWM_MODE;
  cr.channel    = BTS_PWM_CH_R;
  cr.intr_type  = LEDC_INTR_DISABLE;
  cr.timer_sel  = BTS_PWM_TIMER;
  cr.duty       = 0;
  cr.hpoint     = 0;
  ESP_ERROR_CHECK(ledc_channel_config(&cr));

  // LPWM channel
  ledc_channel_config_t cl{};
  cl.gpio_num   = BTS_LPWM_PIN;
  cl.speed_mode = BTS_PWM_MODE;
  cl.channel    = BTS_PWM_CH_L;
  cl.intr_type  = LEDC_INTR_DISABLE;
  cl.timer_sel  = BTS_PWM_TIMER;
  cl.duty       = 0;
  cl.hpoint     = 0;
  ESP_ERROR_CHECK(ledc_channel_config(&cl));

  g_ledc_ready = true;
}

void motor_bts7960_begin() {
#if (BTS_R_EN_PIN >= 0)
  pinMode(BTS_R_EN_PIN, OUTPUT);
  digitalWrite(BTS_R_EN_PIN, HIGH);
#endif
#if (BTS_L_EN_PIN >= 0)
  pinMode(BTS_L_EN_PIN, OUTPUT);
  digitalWrite(BTS_L_EN_PIN, HIGH);
#endif

  pinMode(BTS_RPWM_PIN, OUTPUT);
  pinMode(BTS_LPWM_PIN, OUTPUT);

  _ledc_init_if_needed();

  // Coast
  ledc_set_duty(BTS_PWM_MODE, BTS_PWM_CH_R, 0);
  ledc_update_duty(BTS_PWM_MODE, BTS_PWM_CH_R);
  ledc_set_duty(BTS_PWM_MODE, BTS_PWM_CH_L, 0);
  ledc_update_duty(BTS_PWM_MODE, BTS_PWM_CH_L);

  Serial.println("[MOTOR] BTS7960 (dual EN) motor driver initialized");
}

void motor_bts7960_stop() {
  _ledc_init_if_needed();
  ledc_set_duty(BTS_PWM_MODE, BTS_PWM_CH_R, 0);
  ledc_update_duty(BTS_PWM_MODE, BTS_PWM_CH_R);
  ledc_set_duty(BTS_PWM_MODE, BTS_PWM_CH_L, 0);
  ledc_update_duty(BTS_PWM_MODE, BTS_PWM_CH_L);
}

void motor_bts7960_brake() {
  _ledc_init_if_needed();
  ledc_set_duty(BTS_PWM_MODE, BTS_PWM_CH_R, BTS_PWM_MAX);
  ledc_update_duty(BTS_PWM_MODE, BTS_PWM_CH_R);
  ledc_set_duty(BTS_PWM_MODE, BTS_PWM_CH_L, BTS_PWM_MAX);
  ledc_update_duty(BTS_PWM_MODE, BTS_PWM_CH_L);
}

void motor_bts7960_right(uint16_t duty) {
  _ledc_init_if_needed();
  uint16_t d = _clipDuty(duty == 0 ? g_default_duty : duty);
  ledc_set_duty(BTS_PWM_MODE, BTS_PWM_CH_L, 0);
  ledc_update_duty(BTS_PWM_MODE, BTS_PWM_CH_L);
  ledc_set_duty(BTS_PWM_MODE, BTS_PWM_CH_R, d);
  ledc_update_duty(BTS_PWM_MODE, BTS_PWM_CH_R);
}

void motor_bts7960_left(uint16_t duty) {
  _ledc_init_if_needed();
  uint16_t d = _clipDuty(duty == 0 ? g_default_duty : duty);
  ledc_set_duty(BTS_PWM_MODE, BTS_PWM_CH_R, 0);
  ledc_update_duty(BTS_PWM_MODE, BTS_PWM_CH_R);
  ledc_set_duty(BTS_PWM_MODE, BTS_PWM_CH_L, d);
  ledc_update_duty(BTS_PWM_MODE, BTS_PWM_CH_L);
}

void motor_bts7960_set_right(bool en, uint16_t duty) {
  if (en) motor_bts7960_right(duty);
  else    motor_bts7960_stop();
}
void motor_bts7960_set_left(bool en, uint16_t duty) {
  if (en) motor_bts7960_left(duty);
  else    motor_bts7960_stop();
}

bool motor_bts7960_set_pwm_freq(uint32_t freq_hz) {
  _ledc_init_if_needed();
  uint32_t set = ledc_set_freq(BTS_PWM_MODE, BTS_PWM_TIMER, freq_hz);
  bool ok = (set == freq_hz);
  if (ok) Serial.printf("[MOTOR] PWM frequency set to %lu Hz\n", (unsigned long)freq_hz);
  else    Serial.printf("[MOTOR] PWM frequency set failed, got %lu Hz\n", (unsigned long)set);
  return ok;
}

void motor_bts7960_set_default_duty(uint16_t duty) {
  g_default_duty = _clipDuty(duty);
  Serial.printf("[MOTOR] Default duty set to %u (%.1f%%) [Raw: %u/%u]\n",
                g_default_duty, (g_default_duty * 100.0f) / BTS_PWM_MAX,
                g_default_duty, BTS_PWM_MAX);
}

uint16_t motor_bts7960_get_default_duty() {
  return g_default_duty;
}
