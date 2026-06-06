#pragma once
#include <Arduino.h>
#include "gps.h"
#include "geojson_ble.h"

// Navigation state
typedef enum {
  NAV_MODE_MANUAL = 0,
  NAV_MODE_AUTOPILOT = 1,
  NAV_MODE_NAVIGATION = 2
} NavigationMode;

// Navigation status structure
typedef struct {
  NavigationMode mode;
  bool gps_valid;
  uint16_t current_waypoint;
  uint16_t total_waypoints;
  float distance_to_waypoint_m;
  float bearing_to_waypoint_deg;
  float cross_track_error_m;
  float target_heading_deg;
  bool route_complete;
} NavigationStatus;

// Configuration
typedef struct {
  float waypoint_radius_m;      // Distance to waypoint before advancing (default: 20m)
  float cross_track_limit_m;    // Max cross-track error before correction (default: 50m)
  float cross_track_gain;       // Cross-track correction gain (default: 1.0)
  float min_speed_mps;          // Minimum speed for navigation (default: 0.5 m/s)
  uint32_t gps_timeout_ms;      // GPS timeout before fallback (default: 10000ms)
} NavigationConfig;

// Initialize navigation system
void Navigation_begin();

// Main navigation update (call at 10Hz or higher)
// Returns target heading in degrees (0-360) for autopilot
float Navigation_update(const GpsFix &gps_fix);

// Control functions
bool Navigation_start(uint16_t starting_waypoint = 0);
void Navigation_stop();
bool Navigation_is_active();

// Configuration
void Navigation_set_config(const NavigationConfig &config);
NavigationConfig Navigation_get_config();

// Status queries
NavigationStatus Navigation_get_status();
NavigationMode Navigation_get_mode();
void Navigation_set_mode(NavigationMode mode);

// Route management
bool Navigation_load_route_from_ble();  // Load from GeoJSON BLE storage
bool Navigation_has_valid_route();
void Navigation_clear_route();

// Waypoint control
bool Navigation_set_target_waypoint(uint16_t waypoint_index);
uint16_t Navigation_get_current_waypoint();
bool Navigation_advance_waypoint();

// Utility functions
float Navigation_calculate_bearing(double lat1, double lon1, double lat2, double lon2);
float Navigation_calculate_distance(double lat1, double lon1, double lat2, double lon2);
float Navigation_calculate_cross_track_error(double lat1, double lon1, double lat2, double lon2, 
                                            double current_lat, double current_lon);