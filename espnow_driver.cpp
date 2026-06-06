
/*************** espnow_driver.cpp — Driver with CCAL + Heartbeat (FULL, scaling + AP clamp + EQ fix + smoothed compass) ***************/
#include "espnow_driver.h"
#include "motor_bts7960.h"
#include "mag_encoder.h"
#include "compass_mmc5603.h"
#include "navigation.h"
#include "gps.h"
#include "config_nvs.h"
#include "compass_calibration.h"

#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <driver/ledc.h>
#include <esp_idf_version.h>
#include <math.h>
#include <string.h>

// ========================= Debug helpers (optional) =========================
#ifdef DEBUG
  #define DPRINT(x)   Serial.print(x)
  #define DPRINTLN(x) Serial.println(x)
#else
  #define DPRINT(x)
  #define DPRINTLN(x)
#endif

// ========================= Autopilot turn sense (CONFIGURABLE) ==================
// Set to +1 if a positive heading error (target - heading) should steer RIGHT (CW).
// Set to -1 if it should steer LEFT (CCW). If your boat turned the wrong way, -1 fixes it.
static constexpr int AP_TURN_SIGN = +1;

// Forward declaration
void update_helm_limits(uint16_t min_deg, uint16_t max_deg);
// ========================= Helm scaling (CONFIGURABLE) ======================
// Physical helm travel (runtime-configurable; defaults 40..140)
static volatile int g_helm_min_deg = 40;
static volatile int g_helm_max_deg = 140;
// Define helm limits setter
void update_helm_limits(uint16_t min_deg, uint16_t max_deg) {
  // Clamp to 0..180 and ensure a valid interval
  if (min_deg > 180) min_deg = 180;
  if (max_deg > 180) max_deg = 180;
  if (min_deg >= max_deg) {
    Serial.println("[HELM] Ignored invalid limits (min >= max)");
    return;
  }
  g_helm_min_deg = (int)min_deg;
  g_helm_max_deg = (int)max_deg;
  Serial.printf("[HELM] Limits set to %d..%d deg\n", g_helm_min_deg, g_helm_max_deg);
}


// Controller knob range (Controller now sends 40..140)
static constexpr int KNOB_MIN_DEG = 40;
static constexpr int KNOB_MAX_DEG = 140;

// ========================= BTS7960 wiring =========================
static const int RPWM_PIN = 25;
static const int LPWM_PIN = 26;
static const int R_EN_PIN = 27;
static const int L_EN_PIN = 14;

// Set to 1 to swap directions if needed
#define INVERT_MOTOR_DIR 0

// ========================= PWM config =============================
static const ledc_mode_t   PWM_MODE   = LEDC_LOW_SPEED_MODE;
static const ledc_timer_t  PWM_TIMER  = LEDC_TIMER_0;
static const uint32_t      FREQ_HZ    = 20000;
static const uint8_t       RES_BITS   = 8;
static const ledc_channel_t CH_R      = LEDC_CHANNEL_0;
static const ledc_channel_t CH_L      = LEDC_CHANNEL_1;

// ========================= Small utils =============================
static inline int clampi(int v,int a,int b){ return v<a?a:(v>b?b:v); }
static inline float clampf(float v,float a,float b){ return v<a?a:(v>b?b:v); }
static inline float wrap180(float e){ while(e>180)e-=360; while(e<-180)e+=360; return e; }
static inline int helm_range_limit(int v){ return clampi(v, g_helm_min_deg, g_helm_max_deg); }

// Map Controller knob -> helm target (linear 40..140 -> HELM_MIN..HELM_MAX)
static inline int scale_controller_to_helm(uint16_t knob_deg) {
  int k = clampi((int)knob_deg, KNOB_MIN_DEG, KNOB_MAX_DEG);
  float frac = (float)(k - KNOB_MIN_DEG) / (float)(KNOB_MAX_DEG - KNOB_MIN_DEG);
  return (int)roundf(g_helm_min_deg + frac * (g_helm_max_deg - g_helm_min_deg));
}

static inline void pwm_reclaim() {
  pinMode(R_EN_PIN, OUTPUT);
  pinMode(L_EN_PIN, OUTPUT);
  digitalWrite(R_EN_PIN, HIGH);
  digitalWrite(L_EN_PIN, HIGH);

  ledc_timer_config_t tcfg{};
  tcfg.speed_mode      = PWM_MODE;
  tcfg.timer_num       = PWM_TIMER;
  tcfg.duty_resolution = (ledc_timer_bit_t)RES_BITS;
  tcfg.freq_hz         = FREQ_HZ;
  tcfg.clk_cfg         = LEDC_AUTO_CLK;
  ledc_timer_config(&tcfg);

  ledc_channel_config_t cr{};
  cr.gpio_num   = RPWM_PIN;
  cr.speed_mode = PWM_MODE;
  cr.channel    = CH_R;
  cr.intr_type  = LEDC_INTR_DISABLE;
  cr.timer_sel  = PWM_TIMER;
  cr.duty       = 0;
  cr.hpoint     = 0;
  ledc_channel_config(&cr);

  ledc_channel_config_t cl{};
  cl.gpio_num   = LPWM_PIN;
  cl.speed_mode = PWM_MODE;
  cl.channel    = CH_L;
  cl.intr_type  = LEDC_INTR_DISABLE;
  cl.timer_sel  = PWM_TIMER;
  cl.duty       = 0;
  cl.hpoint     = 0;
  ledc_channel_config(&cl);
}

static inline uint32_t _duty_or_default(uint32_t d) {
  if (d == 0) return motor_bts7960_get_default_duty();
  return d;
}
static inline void _drive_stop_raw() {
  pwm_reclaim();
  ledc_set_duty(PWM_MODE, CH_R, 0); ledc_update_duty(PWM_MODE, CH_R);
  ledc_set_duty(PWM_MODE, CH_L, 0); ledc_update_duty(PWM_MODE, CH_L);
}
static inline void _drive_right_raw(uint32_t duty) {
  pwm_reclaim();
  duty = _duty_or_default(duty);
  if (INVERT_MOTOR_DIR) {
    ledc_set_duty(PWM_MODE, CH_R, 0);    ledc_update_duty(PWM_MODE, CH_R);
    ledc_set_duty(PWM_MODE, CH_L, duty); ledc_update_duty(PWM_MODE, CH_L);
  } else {
    ledc_set_duty(PWM_MODE, CH_L, 0);    ledc_update_duty(PWM_MODE, CH_L);
    ledc_set_duty(PWM_MODE, CH_R, duty); ledc_update_duty(PWM_MODE, CH_R);
  }
}
static inline void _drive_left_raw(uint32_t duty) {
  pwm_reclaim();
  duty = _duty_or_default(duty);
  if (INVERT_MOTOR_DIR) {
    ledc_set_duty(PWM_MODE, CH_L, 0);    ledc_update_duty(PWM_MODE, CH_L);
    ledc_set_duty(PWM_MODE, CH_R, duty); ledc_update_duty(PWM_MODE, CH_R);
  } else {
    ledc_set_duty(PWM_MODE, CH_R, 0);    ledc_update_duty(PWM_MODE, CH_R);
    ledc_set_duty(PWM_MODE, CH_L, duty); ledc_update_duty(PWM_MODE, CH_L);
  }
}

// ========================= ESPNOW peer / state =====================
static uint8_t g_peer_mac[6] = {0};
static bool    g_peer_added  = false;

static volatile int  g_target_position      = 90;
static volatile int  g_current_position     = 90;
static volatile bool g_equalization_active  = false;

static bool   g_autopilot_enabled   = false;
static bool   g_autopilot_centering = false;
static float  g_auto_heading_target = 0.0f;

bool DriverNOW_isAutopilotEnabled() { return g_autopilot_enabled; }

// ========================= AP/NAV params ===========================
static float   AP_KP = 2.0f;
static float   AP_KI = 0.015f;
static float   AP_KD = 3.0f;
static int     AP_HEADING_DEADBAND_DEG = 3;
static int     AP_HELM_MAX_STEP        = 40;
static int     AP_HELM_DEADBAND_STEPS  = 3;
static int     AP_HELM_RATE_LIMIT      = 12;
static uint32_t NAV_UPDATE_PERIOD_MS   = 500;

static const uint32_t AP_UPDATE_PERIOD_MS = 100;
static const uint32_t AP_STALL_TIMEOUT_MS = 4000;
static uint32_t g_last_nav_update_ms = 0;

// ========================= Config bridging functions =======================
// Implemented here to satisfy config_nvs.cpp's calls and keep AP/NAV globals in sync.
void update_autopilot_params(uint16_t max_step, uint16_t deadband, uint16_t rate_limit,
                             float kp, float ki, float kd, uint16_t heading_deadband) {
  AP_HELM_MAX_STEP        = (int)max_step;
  AP_HELM_DEADBAND_STEPS  = (int)deadband;
  AP_HELM_RATE_LIMIT      = (int)rate_limit;
  AP_KP                   = kp;
  AP_KI                   = ki;
  AP_KD                   = kd;
  AP_HEADING_DEADBAND_DEG = (int)heading_deadband;
}

void update_navigation_period(uint16_t period_ms) {
  NAV_UPDATE_PERIOD_MS = (uint32_t)period_ms;
}

// ========================= Config deferral ==========================
static uint32_t        g_config_seq         = 0;
static ConfigUpdateMsg g_pending_config     = {};
static bool            g_has_pending_config = false;

// ========================= Equalization constants =====================
static const int HELM_DEADBAND = 3;
static const int ENDSTOP_MARGIN_DEG = 2;
static const uint32_t STALE_CUTOFF_MS  = 800;

enum MoveDir { MD_STOP=0, MD_RIGHT=1, MD_LEFT=2 };
static MoveDir   g_move_dir            = MD_STOP;
static uint32_t  g_move_began_ms       = 0;
static uint32_t  g_last_switch_ms      = 0;
static int       g_helm_at_move_begin  = 90;
static uint32_t  g_last_helm_sample_ms = 0;
static int       g_last_helm_sample    = 90;

static int       g_last_helm_for_fallback = 90;
static uint32_t  g_helm_last_change_ms    = 0;

static const uint32_t REVERSE_COAST_MS = 120;
static const uint32_t MIN_RUN_MS       = 140;
static const uint32_t RAMP_FULL_MS     = 800;
static const float    BOOST_FACTOR     = 1.30f;
static const uint32_t JAM_CHECK_MS     = 450;

// STOP latch (manual only)
static const uint32_t STOP_HOLD_MS      = 450;
static const int      HOLD_STICK_BAND   = 4;
static const int      HOLD_EXIT_BAND    = 6;
static const uint32_t HOLD_EXIT_TIME_MS = 300;

static bool     g_stop_hold_active     = false;
static uint32_t g_stop_hold_until_ms   = 0;
static int      g_stop_hold_center     = 90;
static uint32_t g_hold_exit_start_ms   = 0;

// ========================= Telemetry heartbeat =====================
static uint32_t g_last_tele_tx_ms      = 0;
static const uint32_t TELE_PERIOD_MS   = 200;  // 5 Hz

// ========================= CCAL state (non-blocking) ===============
enum CCalState { CCAL_IDLE=0, CCAL_SETTLE, CCAL_RUN };
static volatile bool g_ccal_request = false;
static uint16_t g_ccal_req_duration_s = 15;
static uint16_t g_ccal_req_settle_ms  = 0;

static CCalState g_ccal_state = CCAL_IDLE;
static uint32_t  g_ccal_t0     = 0;
static uint32_t  g_ccal_last_print = 0;
static int16_t   g_ccal_min_x, g_ccal_min_y, g_ccal_min_z;
static int16_t   g_ccal_max_x, g_ccal_max_y, g_ccal_max_z;

// ========================= Helpers / sensors =============================
static inline bool is_manual_mode(){
  return (!g_autopilot_enabled) && (Navigation_get_mode() == NAV_MODE_MANUAL);
}

static inline int median3(int a,int b,int c){
  int lo = min(a,b), hi = max(a,b);
  if (c < lo) return lo; if (c > hi) return hi; return c;
}
static int s_fbuf[3] = {90,90,90};
static uint8_t s_fidx = 0; static bool s_filled = false;
static int helm_filtered(int raw){
  s_fbuf[s_fidx] = raw; s_fidx = (s_fidx + 1) % 3;
  if (!s_filled && s_fidx == 0) s_filled = true;
  return s_filled ? median3(s_fbuf[0], s_fbuf[1], s_fbuf[2]) : raw;
}

static float get_heading_safe() {
  // Circular EMA smoothing of compass heading with outlier rejection and rate limit
  static float  last_deg    = 0.0f;
  static float  sX = 0.0f, sY = 0.0f; // unit vector of smoothed heading
  static bool   inited      = false;
  static uint32_t last_ms   = 0;
  static uint32_t last_poll = 0;

  const uint32_t now = millis();

  // Poll at most every 100 ms (10 Hz). Increase if you want even smoother.
  if (now - last_poll < 100) return last_deg;
  last_poll = now;

  if (!compass_is_online()) {
    return last_deg;
  }

  // Use raw instantaneous heading (0..360). We'll smooth it here.
  float h = compass_get_heading_deg();
  if (!isfinite(h)) return last_deg;

  // Compute short-way delta from our current smoothed value
  float d = wrap180(h - last_deg);

  // Outlier reject: huge jumps are likely spurious. Allow rate-limited change.
  float dt = (last_ms == 0) ? 0.1f : (now - last_ms) / 1000.0f;
  if (dt <= 0.0f) dt = 0.1f;

  // Limits (tunable)
  const float MAX_ABS_JUMP_DEG = 35.0f;   // immediate hard gate
  const float MAX_RATE_DEG_S   = 90.0f;   // deg/sec
  const float ALLOWED = max(MAX_ABS_JUMP_DEG, MAX_RATE_DEG_S * dt + 5.0f);

  if (fabsf(d) > ALLOWED) {
    // Ignore this sample, keep last
    return last_deg;
  }

  // Circular EMA update. alpha small => more smoothing.
  const float alpha = 0.15f; // try 0.10..0.25 on water
  float cx = cosf(h * DEG_TO_RAD);
  float cy = sinf(h * DEG_TO_RAD);

  if (!inited) {
    sX = cx; sY = cy;
    inited = true;
  } else {
    sX = (1.0f - alpha) * sX + alpha * cx;
    sY = (1.0f - alpha) * sY + alpha * cy;
    float n = sqrtf(sX*sX + sY*sY);
    if (n > 1e-6f) { sX /= n; sY /= n; }
  }

  float hs = atan2f(sY, sX) * RAD_TO_DEG;
  if (hs < 0) hs += 360.0f;

  last_deg = hs;
  last_ms  = now;
  return last_deg;
}

static int get_helm_safe() {
  static int last = 90;
  static uint32_t last_ms = 0;
  static uint32_t last_poll = 0;
  const uint32_t now = millis();

  if (now - last_poll < 20) return last;
  last_poll = now;

  int helm = mag_get_encoder_position();
  if (helm < 0 || helm > 180) return last;
  if (abs(helm - last) > 50 && (now - last_ms) < 100) return last;

  if (helm != last) g_helm_last_change_ms = now;

  last = helm; last_ms = now;
  g_last_helm_for_fallback = last;
  return helm;
}

static void motion_start(MoveDir dir){
  g_move_dir = dir;
  g_move_began_ms = millis();
  g_helm_at_move_begin = mag_get_encoder_position();
  g_last_helm_sample = g_helm_at_move_begin;
  g_last_helm_sample_ms = g_move_began_ms;
}
static void motion_stop(){
  _drive_stop_raw();
  g_move_dir = MD_STOP;
  g_last_switch_ms = millis();
}

// ramp/boost helpers
static uint32_t ramped_duty(){
  uint32_t base = motor_bts7960_get_default_duty();
  uint32_t t = millis() - g_move_began_ms;
  if (t >= RAMP_FULL_MS) return base;
  float f = 0.45f + 0.55f * (float)t / (float)RAMP_FULL_MS;  // 45%→100%
  uint32_t d = (uint32_t)(base * f);
  if (d < 60) d = 60;
  return d;
}
static uint32_t maybe_boost(uint32_t duty){
  const uint32_t now = millis();
  if (now - g_last_helm_sample_ms < JAM_CHECK_MS) return duty;
  int helm = mag_get_encoder_position();
  int dHelm = abs(helm - g_last_helm_sample);
  g_last_helm_sample = helm;
  g_last_helm_sample_ms = now;
  if (dHelm <= 1) {
    uint32_t boosted = (uint32_t)(duty * BOOST_FACTOR);
    uint32_t maxRaw  = (1u<<RES_BITS) - 1;
    if (boosted > maxRaw) boosted = maxRaw;
    return boosted;
  }
  return duty;
}

// ========================= Equalization core =======================
static void handle_encoder_equalization(){
  g_current_position = get_helm_safe();
  int filt = helm_filtered(g_current_position);
  const uint32_t now = millis();

  // Safety: never chase a target outside physical helm range
  if (g_target_position < g_helm_min_deg) g_target_position = g_helm_min_deg;
  if (g_target_position > g_helm_max_deg) g_target_position = g_helm_max_deg;

  const bool at_low_end  = (filt <= ENDSTOP_MARGIN_DEG);
  const bool at_high_end = (filt >= (180 - ENDSTOP_MARGIN_DEG));
  const bool helm_stale  = (now - g_helm_last_change_ms) > STALE_CUTOFF_MS;

  const bool manual = is_manual_mode();
  const int DB_HELM = manual ? 0 : AP_HELM_DEADBAND_STEPS;

  // disable STOP latch if not manual
  if (!manual || g_autopilot_centering) {
    g_stop_hold_active   = false;
    g_hold_exit_start_ms = 0;
  }

  // STOP latch (manual only)
  if (manual && g_stop_hold_active) {
    int err_hold = abs(filt - g_stop_hold_center);
    if (err_hold <= HOLD_STICK_BAND && now < g_stop_hold_until_ms) {
      if (g_move_dir != MD_STOP) { _drive_stop_raw(); g_move_dir = MD_STOP; }
      return;
    }
    if (err_hold >= HOLD_EXIT_BAND) {
      if (g_hold_exit_start_ms == 0) g_hold_exit_start_ms = now;
      if (now - g_hold_exit_start_ms >= HOLD_EXIT_TIME_MS) {
        g_stop_hold_active   = false;
        g_hold_exit_start_ms = 0;
      } else {
        if (g_move_dir != MD_STOP) { _drive_stop_raw(); g_move_dir = MD_STOP; }
        return;
      }
    } else {
      g_hold_exit_start_ms = 0;
      if (g_move_dir != MD_STOP) { _drive_stop_raw(); g_move_dir = MD_STOP; }
      return;
    }
  }

  // close enough → STOP
  if (!helm_stale && abs(filt - g_target_position) <= DB_HELM){
    if (g_move_dir != MD_STOP) { _drive_stop_raw(); g_move_dir = MD_STOP; DPRINTLN("STOP"); }
    g_equalization_active  = false;

    /* manual stop-hold disabled */
    if (false){
      g_stop_hold_active     = true;
      g_stop_hold_center     = filt;
      g_stop_hold_until_ms   = now + STOP_HOLD_MS;
      g_hold_exit_start_ms   = 0;
    }
    if (g_autopilot_centering && g_target_position == 90){
      g_autopilot_centering = false;
      DPRINTLN("AUTOPILOT: Helm centered at 90");
    }
    return;
  }

  // desired direction
  MoveDir want = MD_STOP;
  if (!helm_stale){
    want = (filt < g_target_position - DB_HELM) ? MD_RIGHT : MD_LEFT;
  } else {
    int delta = g_target_position - g_last_helm_for_fallback;
    if      (delta > 0) want = MD_RIGHT;
    else if (delta < 0) want = MD_LEFT;
  }

  // endstops
  if ((want == MD_RIGHT && at_high_end) || (want == MD_LEFT && at_low_end)){
    if (g_move_dir != MD_STOP){ _drive_stop_raw(); g_move_dir = MD_STOP; DPRINTLN("[SAFE STOP] endstop"); }
    /* manual stop-hold disabled */
    if (false){
      g_stop_hold_active   = true;
      g_stop_hold_center   = filt;
      g_stop_hold_until_ms = now + STOP_HOLD_MS;
      g_hold_exit_start_ms = 0;
    }
    return;
  }

  // reverse guard
  if (g_move_dir != MD_STOP && want != g_move_dir){
    if (now - g_last_switch_ms < REVERSE_COAST_MS){ _drive_stop_raw(); return; }
    if (now - g_move_began_ms < MIN_RUN_MS) return;
    _drive_stop_raw(); g_last_switch_ms = now; g_move_dir = MD_STOP; return;
  }

  // start motion
  if (g_move_dir == MD_STOP && want != MD_STOP){
    if (now - g_last_switch_ms < REVERSE_COAST_MS) return;
    motion_start(want);
    g_equalization_active = true;
    DPRINTLN(want==MD_RIGHT? "RIGHT":"LEFT");
  }

  // ramp/boost
  if (g_move_dir == MD_RIGHT){
    uint32_t d = maybe_boost(ramped_duty());
    _drive_right_raw(d);
  } else if (g_move_dir == MD_LEFT){
    uint32_t d = maybe_boost(ramped_duty());
    _drive_left_raw(d);
  }
}

// ========================= PID Autopilot ===========================
static float    ap_err_prev_deg = 0.0f;
static float    ap_integral     = 0.0f;
static uint32_t ap_prev_ms      = 0;
static int      ap_cmd_prev     = 90;
static uint32_t ap_move_timer   = 0;

static void ap_on(float current_heading_deg){
  g_auto_heading_target = current_heading_deg;
  ap_err_prev_deg = 0.0f;
  ap_integral     = 0.0f;
  ap_prev_ms      = millis();
  ap_cmd_prev     = 90;
  ap_move_timer   = millis();

  // leaving manual → clear any manual hold
  g_stop_hold_active   = false;
  g_hold_exit_start_ms = 0;
}
static int rate_limit(int cmd,int prev,int limit){
  int d = cmd - prev; if (d>limit) d=limit; if (d<-limit) d=-limit; return prev + d;
}
static void ap_update(){
  static uint32_t last = 0;
  uint32_t now = millis();
  if (now - last < AP_UPDATE_PERIOD_MS) return;
  last = now;

  NavigationMode nav_mode = Navigation_get_mode();
  bool autopilot_active = (g_autopilot_enabled || nav_mode == NAV_MODE_NAVIGATION);
  if (!autopilot_active) return;
  if (g_autopilot_centering) return;

  if (nav_mode == NAV_MODE_NAVIGATION && now - g_last_nav_update_ms >= NAV_UPDATE_PERIOD_MS){
    g_last_nav_update_ms = now;
    GpsFix gps_fix; GPS_getFix(gps_fix);
    if (gps_fix.valid){
      g_auto_heading_target = Navigation_update(gps_fix);
    }
  }

  const float heading = get_heading_safe();
  const int   helm    = get_helm_safe();

  float err = AP_TURN_SIGN * wrap180(g_auto_heading_target - heading);
  if (fabsf(err) < AP_HEADING_DEADBAND_DEG) err = 0.0f;

  if (ap_prev_ms == 0) ap_prev_ms = now;
  float dt = (now - ap_prev_ms) / 1000.0f;
  if (dt <= 0.0f) dt = AP_UPDATE_PERIOD_MS/1000.0f;
  ap_prev_ms = now;

  float P = AP_KP * err;
  float Dterm = AP_KD * ((err - ap_err_prev_deg) / dt);
  float cmd_f = 90.0f + P + (AP_KI * ap_integral) + Dterm;

  // Respect physical helm range in AP
  int maxOffsetPhys = min(AP_HELM_MAX_STEP, min(g_helm_max_deg - 90, 90 - g_helm_min_deg));
  int cmd = (int)roundf(cmd_f);
  int offset = clampi(cmd - 90, -maxOffsetPhys, maxOffsetPhys);
  cmd = 90 + offset;
  cmd = rate_limit(cmd, ap_cmd_prev, AP_HELM_RATE_LIMIT);
  cmd = helm_range_limit(cmd); // final clamp to 60..120
  ap_cmd_prev = cmd;

  // anti-windup
  bool not_saturated = (abs((int)roundf(90.0f + P + (AP_KI * ap_integral) + Dterm) - 90) < maxOffsetPhys - 2);
  if (not_saturated && err != 0.0f){
    ap_integral += err * dt;
    ap_integral = clampf(ap_integral, -800.0f, 800.0f);
  }

  if (abs(cmd - helm) <= AP_HELM_DEADBAND_STEPS){
    g_target_position = helm;
#ifdef DEBUG
    Serial.printf("[AP] hold cmd=%d helm=%d err=%.1f\n", cmd, helm, err);
#endif
    _drive_stop_raw();
    g_equalization_active = false;
  } else {
    g_target_position = cmd;
#ifdef DEBUG
    Serial.printf("[AP] drive cmd=%d helm=%d err=%.1f\n", cmd, helm, err);
#endif
    g_equalization_active = true; // <-- ensure equalizer runs
  }

  if (abs(helm - g_target_position) > (AP_HELM_DEADBAND_STEPS + 1)){
    if (now - ap_move_timer > AP_STALL_TIMEOUT_MS){
      _drive_stop_raw(); ap_move_timer = now;
      DPRINTLN("[AP] Motor stall timeout, stopping");
    }
  } else {
    ap_move_timer = now;
  }
  ap_err_prev_deg = err;
}

// ========================= CCAL (non-blocking) =====================
static void ccal_begin(uint16_t dur_s, uint16_t settle_ms) {
  // stop control safely
  if (g_autopilot_enabled) g_autopilot_enabled = false;
  if (Navigation_get_mode() == NAV_MODE_NAVIGATION) Navigation_stop();
  _drive_stop_raw();

  g_ccal_min_x = g_ccal_min_y = g_ccal_min_z =  32767;
  g_ccal_max_x = g_ccal_max_y = g_ccal_max_z = -32768;
  g_ccal_last_print = 0;

  if (settle_ms > 0) {
    g_ccal_state = CCAL_SETTLE;
    g_ccal_t0    = millis();
    Serial.printf("[CAL] Settling for %u ms...\n", (unsigned)settle_ms);
  } else {
    g_ccal_state = CCAL_RUN;
    g_ccal_t0    = millis();
    Serial.printf("[CAL] Starting live compass calibration for %u s\n", (unsigned)dur_s);
    Serial.println("[CAL] Rotate the boat/device slowly in ALL orientations...");
  }
}

static void ccal_process() {
  static uint16_t s_duration = 0;
  static uint16_t s_settle   = 0;

  if (g_ccal_request) {
    g_ccal_request = false;
    s_duration = g_ccal_req_duration_s;
    s_settle   = g_ccal_req_settle_ms;
    ccal_begin(s_duration, s_settle);
    return;
  }

  switch (g_ccal_state) {
    case CCAL_IDLE: return;

    case CCAL_SETTLE:
      if (millis() - g_ccal_t0 >= (uint32_t)s_settle) {
        g_ccal_state = CCAL_RUN;
        g_ccal_t0    = millis();
        Serial.printf("[CAL] Starting live compass calibration for %u s\n", (unsigned)s_duration);
        Serial.println("[CAL] Rotate the boat/device slowly in ALL orientations...");
      }
      break;

    case CCAL_RUN: {
      int16_t x,y,z;
      if (compass_sample_raw(x,y,z)) {
        if (x < g_ccal_min_x) g_ccal_min_x = x; if (x > g_ccal_max_x) g_ccal_max_x = x;
        if (y < g_ccal_min_y) g_ccal_min_y = y; if (y > g_ccal_max_y) g_ccal_max_y = y;
        if (z < g_ccal_min_z) g_ccal_min_z = z; if (z > g_ccal_max_z) g_ccal_max_z = z;
      }
      if (millis() - g_ccal_last_print >= 500) {
        g_ccal_last_print = millis();
        Serial.printf("[CAL] min{%d,%d,%d}  max{%d,%d,%d}\n",
                      g_ccal_min_x, g_ccal_min_y, g_ccal_min_z,
                      g_ccal_max_x, g_ccal_max_y, g_ccal_max_z);
      }
      if (millis() - g_ccal_t0 >= (uint32_t)s_duration * 1000UL) {
        bool ok_span = ((g_ccal_max_x - g_ccal_min_x) >= 100) &&
                       ((g_ccal_max_y - g_ccal_min_y) >= 100) &&
                       ((g_ccal_max_z - g_ccal_min_z) >= 100);

        if (!ok_span) {
          Serial.println("[CAL] FAILED: not enough motion; try again.");
          if (g_peer_added) {
            const char *msg = "CCAL-FAIL";
            esp_now_send(g_peer_mac, (const uint8_t*)msg, strlen(msg)+1);
          }
          g_ccal_state = CCAL_IDLE;
          break;
        }

        // Apply and store to NVS (direct)
        bool ok = CompassCalib_store_direct(g_ccal_min_x, g_ccal_min_y, g_ccal_min_z,
                                            g_ccal_max_x, g_ccal_max_y, g_ccal_max_z);

        Serial.println(ok ? "[CAL] Calibration applied and saved to NVS"
                          : "[CAL] Applied, but save failed");

        if (g_peer_added) {
          char ack[64];
          snprintf(ack, sizeof(ack),
                   ok ? "CCAL-OK min{%d,%d,%d} max{%d,%d,%d}"
                      : "CCAL-APPLIED-NVSERR",
                   g_ccal_min_x, g_ccal_min_y, g_ccal_min_z,
                   g_ccal_max_x, g_ccal_max_y, g_ccal_max_z);
          esp_now_send(g_peer_mac, (const uint8_t*)ack, strlen(ack)+1);
        }
        g_ccal_state = CCAL_IDLE;
      }
    } break;
  }
}

// ========================= Telemetry heartbeat =====================
static void send_telemetry_heartbeat() {
  if (!g_peer_added) return;

  TelemetryMsg t{};
  static uint32_t seq = 0;
  t.seq = ++seq;

  t.heading_deg = get_heading_safe();
  t.servo_deg   = (float)g_target_position;          // commanded target
  t.helm_deg    = (float)mag_get_encoder_position(); // actual helm
  t.link_ok     = 1;

  NavigationStatus s = Navigation_get_status();
  t.nav_mode          = (uint8_t)s.mode;             // 0=MANUAL,1=AP,2=NAV
  t.current_wp        = s.current_waypoint;
  t.dist_to_wp_m      = s.distance_to_waypoint_m;
  t.bearing_to_wp     = s.bearing_to_waypoint_deg;
  t.cross_track_err_m = s.cross_track_error_m;
  t.gps_valid         = s.gps_valid ? 1 : 0;

  esp_now_send(g_peer_mac, (uint8_t*)&t, sizeof(t));
}

// ========================= ESPNOW callbacks =======================
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5,0,0)
static void onDataSent(const wifi_tx_info_t *tx, esp_now_send_status_t status){ (void)tx; (void)status; }
static void onDataRecv(const esp_now_recv_info_t *rx, const uint8_t *data, int len){
  (void)rx;

  // Manual
  if (len == (int)sizeof(CtrlMsg)) {
    CtrlMsg m; memcpy(&m, data, sizeof(m));
    NavigationMode nav_mode = Navigation_get_mode();
    if (!g_autopilot_enabled && nav_mode != NAV_MODE_NAVIGATION) {
      int scaled = scale_controller_to_helm(m.encoder_deg);
      g_target_position = helm_range_limit(scaled);
      g_equalization_active = true;

      // manual: follow stick center if holding
      if (g_stop_hold_active) {
        int delta = abs(scaled - g_stop_hold_center);
        if (delta >= HOLD_EXIT_BAND) {
          g_stop_hold_active = false;
          g_hold_exit_start_ms = 0;
        } else {
          g_stop_hold_center = scaled;
        }
      }
      handle_encoder_equalization();
      Serial.printf("[MANUAL] RX knob=%u  scaled=%d  HELM=%d  TARGET=%d\n",
                    m.encoder_deg, scaled, mag_get_encoder_position(), g_target_position);
    }

    if (g_peer_added){
      TelemetryMsg t{};
      t.seq = m.seq;
      t.heading_deg = get_heading_safe();
      t.servo_deg   = (float)g_target_position;
      t.helm_deg    = (float)mag_get_encoder_position();
      t.link_ok     = 1;
      NavigationStatus s = Navigation_get_status();
      t.nav_mode          = (uint8_t)s.mode;
      t.current_wp        = s.current_waypoint;
      t.dist_to_wp_m      = s.distance_to_waypoint_m;
      t.bearing_to_wp     = s.bearing_to_waypoint_deg;
      t.cross_track_err_m = s.cross_track_error_m;
      t.gps_valid         = s.gps_valid ? 1 : 0;
      esp_now_send(g_peer_mac, (uint8_t*)&t, sizeof(t));
    }
    return;
  }

  // Autopilot
  if (len == (int)sizeof(AutopilotCmd)) {
    AutopilotCmd c; memcpy(&c, data, sizeof(c));
    if (c.autopilot_on && !g_autopilot_enabled){
      if (Navigation_get_mode() == NAV_MODE_NAVIGATION){
        Navigation_stop(); Serial.println("NAVIGATION OFF - Switching to autopilot mode");
      }
      g_stop_hold_active   = false; g_hold_exit_start_ms = 0;
      g_autopilot_enabled   = true;
      g_autopilot_centering = false;
      g_auto_heading_target = c.direction_deg;
      ap_on(get_heading_safe());

      int current_helm = get_helm_safe();
      if (abs(current_helm - 90) > AP_HELM_DEADBAND_STEPS){
        g_autopilot_centering = true;
        g_target_position = 90; // center is within range
        g_equalization_active = true; // <-- ensure EQ runs during centering
        Serial.printf("AUTOPILOT ON - Target heading: %.1f° (centering from %d)\n",
                      g_auto_heading_target, current_helm);
      } else {
        Serial.printf("AUTOPILOT ON - Target heading: %.1f° (centered)\n", g_auto_heading_target);
      }
    } else if (!c.autopilot_on && g_autopilot_enabled){
      g_autopilot_enabled   = false;
      g_autopilot_centering = false;
      g_equalization_active = false;
      g_target_position = 90;
      motion_stop();
      Serial.println("AUTOPILOT OFF - Manual control");
    }
    return;
  }

  // Navigation
  if (len == (int)sizeof(NavigationCmd)) {
    NavigationCmd c; memcpy(&c, data, sizeof(c));
    if (c.navigation_on){
      g_stop_hold_active   = false; g_hold_exit_start_ms = 0;
      if (g_autopilot_enabled){ g_autopilot_enabled = false; Serial.println("AUTOPILOT OFF - Navigation mode"); }

      NavigationConfig cfg = Navigation_get_config();
      cfg.waypoint_radius_m   = c.waypoint_radius;
      cfg.cross_track_limit_m = c.cross_track_limit;
      Navigation_set_config(cfg);

      if (Navigation_start(c.waypoint_index)){
        g_autopilot_centering = false;
        ap_on(get_heading_safe());
        int current_helm = get_helm_safe();
        if (abs(current_helm - 90) > AP_HELM_DEADBAND_STEPS){
          g_autopilot_centering = true; g_target_position = 90;
          g_equalization_active = true; // <-- ensure EQ runs during centering
          Serial.printf("NAVIGATION ON - WP %u (centering from %d)\n", c.waypoint_index, current_helm);
        } else {
          Serial.printf("NAVIGATION ON - WP %u (centered)\n", c.waypoint_index);
        }
      } else {
        Serial.println("NAVIGATION FAILED - no waypoints?");
      }
    } else {
      if (Navigation_get_mode() == NAV_MODE_NAVIGATION){
        Navigation_stop();
        g_autopilot_enabled   = false;
        g_autopilot_centering = false;
        g_equalization_active = false;
        g_target_position = 90;
        motion_stop();
        Serial.println("NAVIGATION OFF - Manual control");
      }
    }
    return;
  }

  // CCAL (queue; do not block here)
  if (len >= (int)sizeof(CompassCalibCmd)) {
    const CompassCalibCmd* c = reinterpret_cast<const CompassCalibCmd*>(data);
    if (memcmp(c->tag, "CCAL", 4) == 0) {
      if (c->duration_s == 0xFFFF) { // CANCEL
        g_ccal_state   = CCAL_IDLE;
        g_ccal_request = false;
        Serial.println("[CCAL] Cancel request");
        return;
      }
      if (c->duration_s == 0) {      // SAVE (unused; we auto-save at end)
        Serial.println("[CCAL] Save request (ignored; auto-save at end)");
        return;
      }
      if (g_ccal_state != CCAL_IDLE || g_ccal_request) {
        Serial.println("[CCAL] Busy, ignoring duplicate START");
        return;
      }
      g_ccal_req_duration_s = c->duration_s;
      g_ccal_req_settle_ms  = c->settle_ms;
      g_ccal_request        = true;
      Serial.printf("[CCAL] Request queued: duration=%us settle=%ums\n",
                    (unsigned)g_ccal_req_duration_s, (unsigned)g_ccal_req_settle_ms);
      return;
    }
  }

  // Config update
  if (len == (int)sizeof(ConfigUpdateMsg)) {
    ConfigUpdateMsg c; memcpy(&c, data, sizeof(c));
    ConfigParam param = (ConfigParam)c.param_id;

    Serial.printf("[CFG] Received: %s = %.2f (save=%d, safe=%d)\n",
                  Config_get_param_name(param), c.value, c.save_to_nvs, c.apply_when_safe);

    if (!Config_validate_param(param, c.value)) {
      ConfigAckMsg ack{c.seq, (uint8_t)param, c.value, 0, 1};
      if (g_peer_added) esp_now_send(g_peer_mac, (uint8_t*)&ack, sizeof(ack));
      Serial.printf("[CFG] Rejected: out of range for %s\n", Config_get_param_name(param));
      return;
    }

    if (c.apply_when_safe){
      g_pending_config = c; g_has_pending_config = true;
      Serial.printf("[CFG] Deferred until safe: %s\n", Config_get_param_name(param));
    } else {
      bool success = Config_update_param(param, c.value, c.save_to_nvs);
      uint8_t reason = success ? 0 : 2;
      if (success){
        DriverConfig cur = Config_get_current();
        switch(param){
          case CFG_ENCODER_COUNTS:  mag_set_counts_per_180(cur.encoder_counts_180); break;
case CFG_PWM_FREQUENCY:   motor_bts7960_set_pwm_freq(cur.pwm_frequency_hz); break;
          case CFG_PID_KP: AP_KP = cur.pid_kp; break;
          case CFG_PID_KI: AP_KI = cur.pid_ki; break;
          case CFG_PID_KD: AP_KD = cur.pid_kd; break;
          case CFG_AP_MAX_STEP: AP_HELM_MAX_STEP = cur.ap_max_step_deg; break;
          case CFG_AP_DEADBAND: AP_HELM_DEADBAND_STEPS = cur.ap_deadband_deg; break;
          case CFG_AP_RATE_LIMIT: AP_HELM_RATE_LIMIT = cur.ap_rate_limit; break;
          case CFG_HEADING_DEADBAND: AP_HEADING_DEADBAND_DEG = cur.heading_deadband_deg; break;
          case CFG_NAV_UPDATE_PERIOD: NAV_UPDATE_PERIOD_MS = cur.nav_update_period_ms; break;
          case CFG_HELM_MIN_DEG:
          case CFG_HELM_MAX_DEG: update_helm_limits(cur.helm_min_deg, cur.helm_max_deg); break;

          default:                  Config_apply_to_system(); break;
        }
      }
      ConfigAckMsg ack{c.seq, (uint8_t)param, c.value, (uint8_t)(success?1:0), reason};
      if (g_peer_added) esp_now_send(g_peer_mac, (uint8_t*)&ack, sizeof(ack));
    }
    return;
  }
}
#else
static void onDataSent(const uint8_t *mac, esp_now_send_status_t status){ (void)mac; (void)status; }
static void onDataRecv_legacy(const uint8_t *mac, const uint8_t *data, int len){
  esp_now_recv_info_t dummy{}; onDataRecv(&dummy, data, len);
}
#endif

// ========================= ESPNOW init/helpers =====================
static bool setLocalMAC(const uint8_t mac[6]){
  if (mac[0] & 0x01){ Serial.println("ERR: multicast MAC not allowed"); return false; }
  esp_err_t err = esp_wifi_set_mac(WIFI_IF_STA, mac);
  if (err != ESP_OK){ Serial.printf("esp_wifi_set_mac failed: %d\n", (int)err); return false; }
  return true;
}

static bool addPeer(const uint8_t peer[6]){
  if (!peer) return false;
  esp_now_peer_info_t p{};
  memcpy(p.peer_addr, peer, 6);
  p.channel = 0;
  p.encrypt = false;
  esp_err_t st = esp_now_add_peer(&p);
  if (st == ESP_ERR_ESPNOW_EXIST) st = ESP_OK;
  return st == ESP_OK;
}

void DriverNOW_begin(const uint8_t local_mac[6], const uint8_t controller_mac[6]){
  WiFi.mode(WIFI_STA);
  if (local_mac) setLocalMAC(local_mac);

  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW init failed!");
    return;
  }

#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5,0,0)
  esp_now_register_recv_cb(onDataRecv);
  esp_now_register_send_cb(onDataSent);
#else
  esp_now_register_recv_cb(onDataRecv_legacy);
  esp_now_register_send_cb(onDataSent);
#endif

  if (controller_mac){
    memcpy(g_peer_mac, controller_mac, 6);
    g_peer_added = addPeer(g_peer_mac);
  } else {
    // peer must be added later
    memset(g_peer_mac, 0, sizeof(g_peer_mac));
    g_peer_added = false;
  }



  Serial.println("[ESPNOW] DriverNOW ready");
}

void DriverNOW_loop(){
  // Apply deferred config when safe
  if (g_has_pending_config){
    bool safe = (!g_autopilot_enabled) && (Navigation_get_mode() == NAV_MODE_MANUAL) && (g_move_dir == MD_STOP);
    if (safe){
      ConfigUpdateMsg c = g_pending_config;
      g_has_pending_config = false;
      bool success = Config_update_param((ConfigParam)c.param_id, c.value, c.save_to_nvs);
      uint8_t reason = success ? 0 : 2;
      if (success){
        DriverConfig cur = Config_get_current();
        switch((ConfigParam)c.param_id){
          case CFG_ENCODER_COUNTS:  mag_set_counts_per_180(cur.encoder_counts_180); break;
          case CFG_PWM_FREQUENCY:   motor_bts7960_set_pwm_freq(cur.pwm_frequency_hz); break;
          case CFG_PID_KP: AP_KP = cur.pid_kp; break;
          case CFG_PID_KI: AP_KI = cur.pid_ki; break;
          case CFG_PID_KD: AP_KD = cur.pid_kd; break;
          case CFG_AP_MAX_STEP: AP_HELM_MAX_STEP = cur.ap_max_step_deg; break;
          case CFG_AP_DEADBAND: AP_HELM_DEADBAND_STEPS = cur.ap_deadband_deg; break;
          case CFG_AP_RATE_LIMIT: AP_HELM_RATE_LIMIT = cur.ap_rate_limit; break;
          case CFG_HEADING_DEADBAND: AP_HEADING_DEADBAND_DEG = cur.heading_deadband_deg; break;
          case CFG_NAV_UPDATE_PERIOD: NAV_UPDATE_PERIOD_MS = cur.nav_update_period_ms; break;
          default: Config_apply_to_system(); break;
        }
      }
      ConfigAckMsg ack{c.seq, c.param_id, c.value, (uint8_t)(success?1:0), reason};
      if (g_peer_added) esp_now_send(g_peer_mac, (uint8_t*)&ack, sizeof(ack));
    }
  }

  // Autopilot update
  ap_update();

  // Equalizer: run unconditionally so AP/manual can move as needed
  handle_encoder_equalization();

  // Compass calibration processing
  ccal_process();

  // Telemetry heartbeat
  uint32_t now = millis();
  if (now - g_last_tele_tx_ms >= TELE_PERIOD_MS){
    g_last_tele_tx_ms = now;
    send_telemetry_heartbeat();
  }
}
