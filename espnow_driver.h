#pragma once
#include <Arduino.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <WiFi.h>
#include "config_nvs.h"

bool DriverNOW_isAutopilotEnabled();

// ---- Message structs (must match Controller) ----

// Simple raw command to trigger compass calibration from Controller.
// We detect by the 4-byte tag "CCAL" so it won't clash with your other structs.
typedef struct __attribute__((packed)) {
  char     tag[4];       // "CCAL"
  uint16_t duration_s;
  uint16_t settle_ms;
} CompassCalibCmd;


typedef struct __attribute__((packed)) {
  uint32_t seq;
  uint16_t encoder_deg;  // 0..180 from Controller knob
} CtrlMsg;

typedef struct __attribute__((packed)) {
  uint32_t seq;
  float heading_deg;     // compass
  float servo_deg;       // echo of encoder
  float helm_deg;        // AS5600 helm angle
  uint8_t link_ok;
  // Navigation status fields
  uint8_t nav_mode;      // 0=manual, 1=autopilot, 2=navigation
  uint16_t current_wp;   // current waypoint index
  float dist_to_wp_m;    // distance to current waypoint (meters)
  float bearing_to_wp;   // bearing to current waypoint (degrees)
  float cross_track_err_m; // cross-track error (meters, + = right of track)
  uint8_t gps_valid;     // GPS fix status
} TelemetryMsg;

// Autopilot command (Controller -> Driver)
typedef struct __attribute__((packed)) {
  uint32_t seq;
  uint8_t  autopilot_on;   // 1 = ON, 0 = OFF
  float    direction_deg;  // heading captured on ON
} AutopilotCmd;

// Navigation command (Controller -> Driver)
typedef struct __attribute__((packed)) {
  uint32_t seq;
  uint8_t  navigation_on;     // 1 = ON, 0 = OFF
  uint8_t  waypoint_index;    // starting waypoint (0 = first)
  float    cross_track_limit; // max cross-track error before correction (meters)
  float    waypoint_radius;   // distance to waypoint before advancing (meters)
} NavigationCmd;

// Configuration update command (Controller -> Driver)
typedef struct __attribute__((packed)) {
  uint32_t seq;
  uint8_t  param_id;          // ConfigParam enum value
  float    value;             // New parameter value
  uint8_t  save_to_nvs;       // 1 = save immediately, 0 = just update RAM
  uint8_t  apply_when_safe;   // 1 = apply when safe, 0 = apply immediately
} ConfigUpdateMsg;

// Configuration acknowledgment (Driver -> Controller)
typedef struct __attribute__((packed)) {
  uint32_t seq;
  uint8_t  param_id;          // ConfigParam that was updated
  float    actual_value;      // Value that was actually applied
  uint8_t  success;           // 1 = success, 0 = failed/rejected
  uint8_t  reason;            // Failure reason: 0=OK, 1=out_of_range, 2=unsafe_conditions, 3=unknown_param
} ConfigAckMsg;

// Function declarations for configuration system
void update_autopilot_params(uint16_t max_step, uint16_t deadband, uint16_t rate_limit, 
                             float kp, float ki, float kd, uint16_t heading_deadband);
void update_navigation_period(uint16_t period_ms);

// Initialize ESP-NOW on Driver, set local & peer MACs, register callbacks
void DriverNOW_begin(const uint8_t local_mac[6], const uint8_t controller_mac[6]);

// Background service (optional; callbacks do most work)
void DriverNOW_loop();