#pragma once
#include <Arduino.h>

// Configuration parameter IDs (must match controller)
typedef enum {
  CFG_MOTOR_DUTY = 0,         // Motor duty cycle percent (30-100)
  CFG_AP_MAX_STEP = 1,        // Autopilot max helm step (20-50 deg)
  CFG_AP_DEADBAND = 2,        // Autopilot helm deadband (2-6 deg)
  CFG_AP_RATE_LIMIT = 3,      // Autopilot rate limit per 100ms (4-16)
  CFG_ENCODER_COUNTS = 4,     // Encoder counts per 180 deg (180-720)
  CFG_PID_KP = 5,             // PID Kp (0.5-5.0)
  CFG_PID_KI = 6,             // PID Ki (0.001-0.1)
  CFG_PID_KD = 7,             // PID Kd (0.5-10.0)
  CFG_HEADING_DEADBAND = 8,   // Heading deadband (1-10 deg)
  CFG_NAV_UPDATE_PERIOD = 9,  // Navigation update period ms (200-1000)
  CFG_PWM_FREQUENCY = 10,     // Motor PWM frequency Hz (10000-100000)
  CFG_HELM_MIN_DEG = 11,      // NEW: Helm min target deg (default 40)
  CFG_HELM_MAX_DEG = 12,      // NEW: Helm max target deg (default 140)
  CFG_COUNT = 13
} ConfigParam;

// Persistent configuration blob stored in NVS
typedef struct {
  uint16_t motor_duty_percent;    // 30-100
  uint16_t ap_max_step_deg;       // 20-50
  uint16_t ap_deadband_deg;       // 2-6
  uint16_t ap_rate_limit;         // 4-16 per 100ms
  uint16_t encoder_counts_180;    // 180-720
  float    pid_kp;                // 0.5-5.0
  float    pid_ki;                // 0.001-0.1
  float    pid_kd;                // 0.5-10.0
  uint16_t heading_deadband_deg;  // 1-10
  uint16_t nav_update_period_ms;  // 200-1000
  uint32_t pwm_frequency_hz;      // 10000-100000
  uint16_t helm_min_deg;          // 20-100 (default 40)
  uint16_t helm_max_deg;          // 100-160 (default 140)
  uint32_t checksum;              // integrity
} DriverConfig;

// Defaults
#define CFG_DEFAULT_MOTOR_DUTY         70
#define CFG_DEFAULT_AP_MAX_STEP        30
#define CFG_DEFAULT_AP_DEADBAND        3
#define CFG_DEFAULT_AP_RATE_LIMIT      12
#define CFG_DEFAULT_ENCODER_COUNTS     540
#define CFG_DEFAULT_PID_KP             2.0f
#define CFG_DEFAULT_PID_KI             0.015f
#define CFG_DEFAULT_PID_KD             3.0f
#define CFG_DEFAULT_HEADING_DEADBAND   3
#define CFG_DEFAULT_NAV_UPDATE_PERIOD  500
#define CFG_DEFAULT_PWM_FREQUENCY      20000
#define CFG_DEFAULT_HELM_MIN_DEG       40
#define CFG_DEFAULT_HELM_MAX_DEG       140

// API
bool Config_begin();
bool Config_load(DriverConfig &config);
bool Config_save(const DriverConfig &config);
bool Config_reset_defaults();
DriverConfig Config_get_current();
bool Config_update_param(ConfigParam param, float value, bool save_immediately = false);
bool Config_validate_param(ConfigParam param, float value);
const char* Config_get_param_name(ConfigParam param);
void Config_apply_to_system();
bool Config_get_param_range(ConfigParam param, float &min_val, float &max_val);
