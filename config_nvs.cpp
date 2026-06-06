#include "config_nvs.h"
#include <nvs_flash.h>
#include <nvs.h>
#include <string.h>

static const char* NVS_NAMESPACE = "driver_cfg";
static const char* NVS_KEY       = "v1";

static DriverConfig g_current_config{};
static bool         g_initialized = false;

static uint32_t calc_checksum(const DriverConfig &cfg) {
  const uint8_t* p = reinterpret_cast<const uint8_t*>(&cfg);
  size_t len = sizeof(DriverConfig) - sizeof(uint32_t);
  uint32_t sum = 0;
  for (size_t i=0;i<len;i++) sum += p[i];
  return sum;
}

static void load_defaults(DriverConfig &cfg) {
  cfg.motor_duty_percent    = CFG_DEFAULT_MOTOR_DUTY;
  cfg.ap_max_step_deg       = CFG_DEFAULT_AP_MAX_STEP;
  cfg.ap_deadband_deg       = CFG_DEFAULT_AP_DEADBAND;
  cfg.ap_rate_limit         = CFG_DEFAULT_AP_RATE_LIMIT;
  cfg.encoder_counts_180    = CFG_DEFAULT_ENCODER_COUNTS;
  cfg.pid_kp                = CFG_DEFAULT_PID_KP;
  cfg.pid_ki                = CFG_DEFAULT_PID_KI;
  cfg.pid_kd                = CFG_DEFAULT_PID_KD;
  cfg.heading_deadband_deg  = CFG_DEFAULT_HEADING_DEADBAND;
  cfg.nav_update_period_ms  = CFG_DEFAULT_NAV_UPDATE_PERIOD;
  cfg.pwm_frequency_hz      = CFG_DEFAULT_PWM_FREQUENCY;
  cfg.helm_min_deg          = CFG_DEFAULT_HELM_MIN_DEG;
  cfg.helm_max_deg          = CFG_DEFAULT_HELM_MAX_DEG;
  cfg.checksum              = calc_checksum(cfg);
}

static const char* param_names[CFG_COUNT] = {
  "MOTOR_DUTY",
  "AP_MAX_STEP",
  "AP_DEADBAND",
  "AP_RATE_LIMIT",
  "ENCODER_COUNTS",
  "PID_KP",
  "PID_KI",
  "PID_KD",
  "HEADING_DEADBAND",
  "NAV_UPDATE_PERIOD",
  "PWM_FREQUENCY",
  "HELM_MIN_DEG",
  "HELM_MAX_DEG"
};

bool Config_begin() {
  esp_err_t err = nvs_flash_init();
  if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
    nvs_flash_erase();
    err = nvs_flash_init();
  }
  if (err != ESP_OK) {
    Serial.printf("[CFG] nvs init failed: %s\n", esp_err_to_name(err));
    return false;
  }

  if (!Config_load(g_current_config)) {
    Serial.println("[CFG] No valid config, using defaults");
    load_defaults(g_current_config);
    Config_save(g_current_config);
  } else {
    Serial.println("[CFG] Loaded config from NVS");
  }

  g_initialized = true;
  Config_apply_to_system();
  return true;
}

bool Config_load(DriverConfig &out) {
  nvs_handle_t h;
  esp_err_t err = nvs_open(NVS_NAMESPACE, NVS_READONLY, &h);
  if (err != ESP_OK) return false;

  DriverConfig tmp{};
  size_t sz = sizeof(DriverConfig);
  err = nvs_get_blob(h, NVS_KEY, &tmp, &sz);
  nvs_close(h);
  if (err != ESP_OK || sz != sizeof(DriverConfig)) return false;

  if (tmp.checksum != calc_checksum(tmp)) return false;
  out = tmp;
  return true;
}

bool Config_save(const DriverConfig &cfg) {
  nvs_handle_t h;
  esp_err_t err = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &h);
  if (err != ESP_OK) return false;

  DriverConfig copy = cfg;
  copy.checksum = calc_checksum(copy);
  err = nvs_set_blob(h, NVS_KEY, &copy, sizeof(copy));
  if (err == ESP_OK) err = nvs_commit(h);
  nvs_close(h);

  if (err == ESP_OK) Serial.println("[CFG] NVS saved");
  else Serial.printf("[CFG] NVS save failed: %s\n", esp_err_to_name(err));
  return err == ESP_OK;
}

bool Config_reset_defaults() {
  load_defaults(g_current_config);
  bool ok = Config_save(g_current_config);
  if (ok) Config_apply_to_system();
  return ok;
}

DriverConfig Config_get_current() { return g_current_config; }

bool Config_validate_param(ConfigParam p, float v) {
  switch (p) {
    case CFG_MOTOR_DUTY:        return v >= 30 && v <= 100;
    case CFG_AP_MAX_STEP:       return v >= 20 && v <= 50;
    case CFG_AP_DEADBAND:       return v >= 2  && v <= 6;
    case CFG_AP_RATE_LIMIT:     return v >= 4  && v <= 16;
    case CFG_ENCODER_COUNTS:    return v >= 180 && v <= 720;
    case CFG_PID_KP:            return v >= 0.5f   && v <= 5.0f;
    case CFG_PID_KI:            return v >= 0.001f && v <= 0.1f;
    case CFG_PID_KD:            return v >= 0.5f   && v <= 10.0f;
    case CFG_HEADING_DEADBAND:  return v >= 1 && v <= 10;
    case CFG_NAV_UPDATE_PERIOD: return v >= 200 && v <= 1000;
    case CFG_PWM_FREQUENCY:     return v >= 10000 && v <= 100000;
    case CFG_HELM_MIN_DEG:      return v >= 20 && v <= 100;
    case CFG_HELM_MAX_DEG:      return v >= 100 && v <= 160;
    default: return false;
  }
}

bool Config_update_param(ConfigParam p, float v, bool save_immediately) {
  if (!g_initialized || !Config_validate_param(p, v)) {
    Serial.printf("[CFG] Reject %s=%.2f (range)\n", Config_get_param_name(p), v);
    return false;
  }

  // Cross-constraints for helm limits:
  if (p == CFG_HELM_MIN_DEG) {
    if (v >= g_current_config.helm_max_deg) { Serial.println("[CFG] helm_min >= helm_max"); return false; }
    g_current_config.helm_min_deg = (uint16_t)v;
  } else if (p == CFG_HELM_MAX_DEG) {
    if (v <= g_current_config.helm_min_deg) { Serial.println("[CFG] helm_max <= helm_min"); return false; }
    g_current_config.helm_max_deg = (uint16_t)v;
  } else {
    switch (p) {
      case CFG_MOTOR_DUTY:        g_current_config.motor_duty_percent = (uint16_t)v; break;
      case CFG_AP_MAX_STEP:       g_current_config.ap_max_step_deg     = (uint16_t)v; break;
      case CFG_AP_DEADBAND:       g_current_config.ap_deadband_deg     = (uint16_t)v; break;
      case CFG_AP_RATE_LIMIT:     g_current_config.ap_rate_limit       = (uint16_t)v; break;
      case CFG_ENCODER_COUNTS:    g_current_config.encoder_counts_180  = (uint16_t)v; break;
      case CFG_PID_KP:            g_current_config.pid_kp              = v; break;
      case CFG_PID_KI:            g_current_config.pid_ki              = v; break;
      case CFG_PID_KD:            g_current_config.pid_kd              = v; break;
      case CFG_HEADING_DEADBAND:  g_current_config.heading_deadband_deg= (uint16_t)v; break;
      case CFG_NAV_UPDATE_PERIOD: g_current_config.nav_update_period_ms= (uint16_t)v; break;
      case CFG_PWM_FREQUENCY:     g_current_config.pwm_frequency_hz    = (uint32_t)v; break;
      default: return false;
    }
  }

  if (save_immediately) return Config_save(g_current_config);
  return true;
}

const char* Config_get_param_name(ConfigParam p) {
  if (p >= 0 && p < CFG_COUNT) return param_names[p];
  return "UNKNOWN";
}

bool Config_get_param_range(ConfigParam p, float &mn, float &mx) {
  switch (p) {
    case CFG_MOTOR_DUTY:        mn=30; mx=100; return true;
    case CFG_AP_MAX_STEP:       mn=20; mx=50;  return true;
    case CFG_AP_DEADBAND:       mn=2;  mx=6;   return true;
    case CFG_AP_RATE_LIMIT:     mn=4;  mx=16;  return true;
    case CFG_ENCODER_COUNTS:    mn=180;mx=720; return true;
    case CFG_PID_KP:            mn=0.5f; mx=5.0f; return true;
    case CFG_PID_KI:            mn=0.001f; mx=0.1f; return true;
    case CFG_PID_KD:            mn=0.5f; mx=10.0f; return true;
    case CFG_HEADING_DEADBAND:  mn=1; mx=10; return true;
    case CFG_NAV_UPDATE_PERIOD: mn=200; mx=1000; return true;
    case CFG_PWM_FREQUENCY:     mn=10000; mx=100000; return true;
    case CFG_HELM_MIN_DEG:      mn=20; mx=100; return true;
    case CFG_HELM_MAX_DEG:      mn=100; mx=160; return true;
    default: return false;
  }
}

void Config_apply_to_system() {
  extern void     motor_bts7960_set_default_duty(uint16_t duty);
  extern bool     motor_bts7960_set_pwm_freq(uint32_t freq_hz);
  extern void     update_autopilot_params(uint16_t max_step, uint16_t deadband, uint16_t rate_limit,
                                          float kp, float ki, float kd, uint16_t heading_deadband);
  extern void     update_navigation_period(uint16_t period_ms);
  extern void     mag_set_counts_per_180(int counts);
  extern void     update_helm_limits(uint16_t min_deg, uint16_t max_deg);

  // Motor defaults (percent -> raw 0..255)
  uint16_t duty_raw = (g_current_config.motor_duty_percent * 255) / 100;
  motor_bts7960_set_default_duty(duty_raw);

  // Encoder resolution (slider)
  mag_set_counts_per_180(g_current_config.encoder_counts_180);

  // Helm min/max (NEW)
  update_helm_limits(g_current_config.helm_min_deg, g_current_config.helm_max_deg);

  // AP/PID/heading params
  update_autopilot_params(
    g_current_config.ap_max_step_deg,
    g_current_config.ap_deadband_deg,
    g_current_config.ap_rate_limit,
    g_current_config.pid_kp,
    g_current_config.pid_ki,
    g_current_config.pid_kd,
    g_current_config.heading_deadband_deg
  );

  // Navigation update period
  update_navigation_period(g_current_config.nav_update_period_ms);

  // PWM freq
  motor_bts7960_set_pwm_freq(g_current_config.pwm_frequency_hz);

  Serial.printf("[CFG] Applied: duty=%u%% enc180=%u helm{%u..%u} ap(max=%u,db=%u,rate=%u,kp=%.3f,ki=%.3f,kd=%.3f,hd_db=%u) nav=%ums pwm=%luHz\n",
    (unsigned)g_current_config.motor_duty_percent,
    (unsigned)g_current_config.encoder_counts_180,
    (unsigned)g_current_config.helm_min_deg,
    (unsigned)g_current_config.helm_max_deg,
    (unsigned)g_current_config.ap_max_step_deg,
    (unsigned)g_current_config.ap_deadband_deg,
    (unsigned)g_current_config.ap_rate_limit,
    g_current_config.pid_kp,
    g_current_config.pid_ki,
    g_current_config.pid_kd,
    (unsigned)g_current_config.heading_deadband_deg,
    (unsigned)g_current_config.nav_update_period_ms,
    (unsigned long)g_current_config.pwm_frequency_hz
  );
}
