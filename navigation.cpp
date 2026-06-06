#define NAV_DEBUG 0  // Set to 1 to enable debug, 0 to disable

#include "navigation.h"
#include <math.h>

// Constants
static const float EARTH_RADIUS_M = 6371000.0f;  // Earth radius in meters
// Note: DEG_TO_RAD and RAD_TO_DEG are already defined in Arduino.h

// Navigation state
static NavigationMode g_nav_mode = NAV_MODE_MANUAL;
static NavigationConfig g_config = {
  .waypoint_radius_m = 20.0f,     // 20m radius to advance waypoint
  .cross_track_limit_m = 50.0f,   // 50m cross-track limit
  .cross_track_gain = 1.0f,       // 1.0 gain for cross-track correction
  .min_speed_mps = 0.5f,          // 0.5 m/s minimum speed
  .gps_timeout_ms = 10000         // 10 second GPS timeout
};

static NavigationStatus g_status = {};
static uint32_t g_last_gps_valid_ms = 0;

// Internal route storage (copied from BLE on start)
static std::vector<Waypoint> g_nav_route;

// State management flags
static bool g_navigation_manually_stopped = false;
static bool g_navigation_active = false;  // NEW: Clear active state tracking

// Helper function prototypes
static void update_navigation_status(const GpsFix &gps_fix);
static float calculate_target_heading(const GpsFix &gps_fix);
static bool advance_to_next_waypoint();
static void sync_navigation_state();  // NEW: State synchronization

void Navigation_begin() {
  g_nav_mode = NAV_MODE_MANUAL;
  g_status = {};
  g_nav_route.clear();
  g_last_gps_valid_ms = 0;
  g_navigation_manually_stopped = false;
  g_navigation_active = false;
  
  // Initialize status structure
  g_status.mode = NAV_MODE_MANUAL;
  g_status.gps_valid = false;
  g_status.current_waypoint = 0;
  g_status.total_waypoints = 0;
  g_status.distance_to_waypoint_m = 0.0f;
  g_status.bearing_to_waypoint_deg = 0.0f;
  g_status.cross_track_error_m = 0.0f;
  g_status.target_heading_deg = 0.0f;
  g_status.route_complete = false;
  
  Serial.println("[NAV] Navigation system initialized");
}

// NEW: State synchronization helper
static void sync_navigation_state() {
  g_status.mode = g_nav_mode;
  g_navigation_active = (g_nav_mode == NAV_MODE_NAVIGATION && !g_navigation_manually_stopped);
  
  #if NAV_DEBUG
  Serial.printf("[NAV DEBUG] State sync - mode=%d, active=%d, stopped=%d\n",
                (int)g_nav_mode, g_navigation_active, g_navigation_manually_stopped);
  #endif
}

float Navigation_update(const GpsFix &gps_fix) {
  // Always update status regardless of mode
  update_navigation_status(gps_fix);
  sync_navigation_state();
  
  #if NAV_DEBUG
  Serial.printf("[NAV DEBUG] Update called - mode=%d, gps_valid=%d, route_empty=%d, route_complete=%d, manually_stopped=%d, active=%d\n",
                (int)g_nav_mode, gps_fix.valid, g_nav_route.empty(), g_status.route_complete, g_navigation_manually_stopped, g_navigation_active);
  #endif
  
  // Only calculate new heading if in navigation mode and active
  if (!g_navigation_active) {
    return g_status.target_heading_deg;  // Return last calculated heading
  }
  
  // Check if navigation should auto-stop due to errors
  if (!g_status.gps_valid) {
    Serial.println("[NAV] GPS invalid - auto-stopping navigation");
    Navigation_stop();
    return g_status.target_heading_deg;
  }
  
  if (g_nav_route.empty()) {
    Serial.println("[NAV] No waypoints - auto-stopping navigation");
    Navigation_stop();
    return g_status.target_heading_deg;
  }
  
  if (g_status.route_complete) {
    Serial.println("[NAV] Route complete - auto-stopping navigation");
    Navigation_stop();
    return g_status.target_heading_deg;
  }
  
  // Calculate new target heading
  float new_heading = calculate_target_heading(gps_fix);
  g_status.target_heading_deg = new_heading;
  
  return new_heading;
}

bool Navigation_start(uint16_t starting_waypoint) {
  // Clear manual stop flag when starting
  g_navigation_manually_stopped = false;
  
  // Load route from BLE storage
  if (!Navigation_load_route_from_ble()) {
    Serial.println("[NAV] Failed to load route from BLE");
    sync_navigation_state();
    return false;
  }
  
  if (starting_waypoint >= g_nav_route.size()) {
    Serial.printf("[NAV] Invalid starting waypoint %u (max %u)\n", 
                  starting_waypoint, (unsigned)(g_nav_route.size() - 1));
    sync_navigation_state();
    return false;
  }
  
  g_nav_mode = NAV_MODE_NAVIGATION;
  g_status.current_waypoint = starting_waypoint;
  g_status.total_waypoints = g_nav_route.size();
  g_status.route_complete = false;
  
  // Initialize target heading to 0 - will be calculated on first update
  g_status.target_heading_deg = 0.0f;
  
  sync_navigation_state();
  
  Serial.printf("[NAV] Navigation started - waypoint %u/%u\n", 
                starting_waypoint + 1, (unsigned)g_nav_route.size());
  
  return true;
}

void Navigation_stop() {
  bool was_active = g_navigation_active;
  
  if (g_nav_mode == NAV_MODE_NAVIGATION) {
    g_navigation_manually_stopped = true;  // Mark as manually stopped
    if (was_active) {
      Serial.println("[NAV] Navigation manually stopped");
    }
  }
  
  g_nav_mode = NAV_MODE_MANUAL;  // Will be set to autopilot by caller if needed
  sync_navigation_state();
}

bool Navigation_is_active() {
  sync_navigation_state();
  return g_navigation_active;
}

void Navigation_set_config(const NavigationConfig &config) {
  g_config = config;
  Serial.printf("[NAV] Config updated - wp_radius=%.1fm, xt_limit=%.1fm\n",
                config.waypoint_radius_m, config.cross_track_limit_m);
}

NavigationConfig Navigation_get_config() {
  return g_config;
}

NavigationStatus Navigation_get_status() {
  sync_navigation_state();
  return g_status;
}

NavigationMode Navigation_get_mode() {
  return g_nav_mode;
}

void Navigation_set_mode(NavigationMode mode) {
  NavigationMode old_mode = g_nav_mode;
  
  // Handle mode transition logic
  if (mode == NAV_MODE_NAVIGATION) {
    g_navigation_manually_stopped = false;
  } else if (old_mode == NAV_MODE_NAVIGATION && mode != NAV_MODE_NAVIGATION) {
    g_navigation_manually_stopped = true;
  }
  
  g_nav_mode = mode;
  sync_navigation_state();
  
  Serial.printf("[NAV] Mode changed from %d to %d\n", (int)old_mode, (int)mode);
}

bool Navigation_load_route_from_ble() {
  const std::vector<Waypoint>& ble_route = GeoRoute_all();
  
  if (ble_route.empty()) {
    Serial.println("[NAV] No waypoints in BLE storage");
    g_status.total_waypoints = 0;
    return false;
  }
  
  // Copy route to internal storage
  g_nav_route = ble_route;
  g_status.total_waypoints = g_nav_route.size();
  
  Serial.printf("[NAV] Loaded %u waypoints from BLE\n", (unsigned)g_nav_route.size());
  
  #if NAV_DEBUG
  // Debug: Print all waypoints
  for (size_t i = 0; i < g_nav_route.size(); i++) {
    Serial.printf("[NAV] WP %u: lat=%.6f, lon=%.6f\n", 
                  (unsigned)i, g_nav_route[i].lat, g_nav_route[i].lon);
  }
  #endif
  
  return true;
}

bool Navigation_has_valid_route() {
  return !g_nav_route.empty();
}

void Navigation_clear_route() {
  g_nav_route.clear();
  g_status.total_waypoints = 0;
  sync_navigation_state();
  Serial.println("[NAV] Route cleared");
}

bool Navigation_set_target_waypoint(uint16_t waypoint_index) {
  if (waypoint_index >= g_nav_route.size()) {
    return false;
  }
  
  g_status.current_waypoint = waypoint_index;
  g_status.route_complete = false;
  sync_navigation_state();
  
  Serial.printf("[NAV] Target waypoint set to %u\n", waypoint_index);
  return true;
}

uint16_t Navigation_get_current_waypoint() {
  return g_status.current_waypoint;
}

bool Navigation_advance_waypoint() {
  if (g_status.current_waypoint >= g_nav_route.size() - 1) {
    g_status.route_complete = true;
    sync_navigation_state();
    Serial.println("[NAV] Route completed - no more waypoints");
    return false;
  }
  
  g_status.current_waypoint++;
  Serial.printf("[NAV] Advanced to waypoint %u/%u\n", 
                g_status.current_waypoint + 1, (unsigned)g_nav_route.size());
  return true;
}

// Utility functions using great circle calculations
float Navigation_calculate_bearing(double lat1, double lon1, double lat2, double lon2) {
  double lat1_rad = lat1 * DEG_TO_RAD;
  double lat2_rad = lat2 * DEG_TO_RAD;
  double dlon_rad = (lon2 - lon1) * DEG_TO_RAD;
  
  double y = sin(dlon_rad) * cos(lat2_rad);
  double x = cos(lat1_rad) * sin(lat2_rad) - sin(lat1_rad) * cos(lat2_rad) * cos(dlon_rad);
  
  double bearing_rad = atan2(y, x);
  double bearing_deg = bearing_rad * RAD_TO_DEG;
  
  // Normalize to 0-360
  while (bearing_deg < 0.0) bearing_deg += 360.0;
  while (bearing_deg >= 360.0) bearing_deg -= 360.0;
  
  return (float)bearing_deg;
}

float Navigation_calculate_distance(double lat1, double lon1, double lat2, double lon2) {
  double lat1_rad = lat1 * DEG_TO_RAD;
  double lat2_rad = lat2 * DEG_TO_RAD;
  double dlat_rad = (lat2 - lat1) * DEG_TO_RAD;
  double dlon_rad = (lon2 - lon1) * DEG_TO_RAD;
  
  double a = sin(dlat_rad/2) * sin(dlat_rad/2) + 
             cos(lat1_rad) * cos(lat2_rad) * 
             sin(dlon_rad/2) * sin(dlon_rad/2);
  double c = 2 * atan2(sqrt(a), sqrt(1-a));
  
  return (float)(EARTH_RADIUS_M * c);
}

float Navigation_calculate_cross_track_error(double lat1, double lon1, double lat2, double lon2, 
                                            double current_lat, double current_lon) {
  // Distance from start to current position
  float dist_ac = Navigation_calculate_distance(lat1, lon1, current_lat, current_lon);
  
  // Bearing from start to current position
  float bearing_ac = Navigation_calculate_bearing(lat1, lon1, current_lat, current_lon) * DEG_TO_RAD;
  
  // Bearing from start to end (desired track)
  float bearing_ab = Navigation_calculate_bearing(lat1, lon1, lat2, lon2) * DEG_TO_RAD;
  
  // Cross-track error (positive = right of track)
  float cross_track = asin(sin(dist_ac / EARTH_RADIUS_M) * sin(bearing_ac - bearing_ab)) * EARTH_RADIUS_M;
  
  return cross_track;
}

// Internal helper functions
static void update_navigation_status(const GpsFix &gps_fix) {
  g_status.mode = g_nav_mode;
  g_status.gps_valid = gps_fix.valid;
  g_status.total_waypoints = g_nav_route.size();
  
  if (gps_fix.valid) {
    g_last_gps_valid_ms = millis();
  } else {
    // Check GPS timeout
    if (millis() - g_last_gps_valid_ms > g_config.gps_timeout_ms) {
      if (g_navigation_active) {
        Serial.println("[NAV] GPS timeout - navigation will auto-stop");
      }
    }
  }
  
  // Update waypoint status if we have a valid route and position
  if (gps_fix.valid && g_status.current_waypoint < g_nav_route.size()) {
    const Waypoint& target_wp = g_nav_route[g_status.current_waypoint];
    
    // Calculate distance and bearing to current waypoint
    g_status.distance_to_waypoint_m = Navigation_calculate_distance(
      gps_fix.lat, gps_fix.lon, target_wp.lat, target_wp.lon);
    
    g_status.bearing_to_waypoint_deg = Navigation_calculate_bearing(
      gps_fix.lat, gps_fix.lon, target_wp.lat, target_wp.lon);
    
    #if NAV_DEBUG
    Serial.printf("[NAV DEBUG] Current: %.6f,%.6f -> WP%u: %.6f,%.6f = Bearing %.1f deg, Dist %.1fm\n",
                  gps_fix.lat, gps_fix.lon,
                  g_status.current_waypoint,
                  target_wp.lat, target_wp.lon,
                  g_status.bearing_to_waypoint_deg,
                  g_status.distance_to_waypoint_m);
    #endif
    
    // Calculate cross-track error if we have at least 2 waypoints
    if (g_status.current_waypoint > 0) {
      const Waypoint& prev_wp = g_nav_route[g_status.current_waypoint - 1];
      g_status.cross_track_error_m = Navigation_calculate_cross_track_error(
        prev_wp.lat, prev_wp.lon, target_wp.lat, target_wp.lon,
        gps_fix.lat, gps_fix.lon);
    } else {
      g_status.cross_track_error_m = 0.0f;  // First waypoint has no cross-track
    }
    
    // Check if we should advance waypoint (only if navigation is active)
    if (g_navigation_active && g_status.distance_to_waypoint_m <= g_config.waypoint_radius_m) {
      advance_to_next_waypoint();
    }
  } else {
    g_status.distance_to_waypoint_m = 0.0f;
    g_status.bearing_to_waypoint_deg = 0.0f;
    g_status.cross_track_error_m = 0.0f;
  }
}

static float calculate_target_heading(const GpsFix &gps_fix) {
  if (g_status.current_waypoint >= g_nav_route.size()) {
    #if NAV_DEBUG
    Serial.println("[NAV DEBUG] No valid waypoint - keeping last heading");
    #endif
    return g_status.target_heading_deg;  // Keep last heading
  }
  
  // Base heading is direct bearing to waypoint (calculated in update_navigation_status)
  float target_heading = g_status.bearing_to_waypoint_deg;
  
  #if NAV_DEBUG
  Serial.printf("[NAV DEBUG] Base bearing to WP%u: %.1f deg\n", 
                g_status.current_waypoint, target_heading);
  #endif
  
  // Apply cross-track error correction
  if (fabsf(g_status.cross_track_error_m) > 5.0f) {  // Only correct if error > 5m
    float cross_track_correction = g_status.cross_track_error_m * g_config.cross_track_gain;
    
    // Limit correction to reasonable values
    cross_track_correction = constrain(cross_track_correction, -30.0f, 30.0f);
    
    target_heading -= cross_track_correction;  // Negative because + error = right of track
    
    #if NAV_DEBUG
    Serial.printf("[NAV DEBUG] Cross-track correction: %.1fm error -> %.1f deg correction -> %.1f deg final\n",
                  g_status.cross_track_error_m, cross_track_correction, target_heading);
    #endif
    
    // Normalize heading
    while (target_heading < 0.0f) target_heading += 360.0f;
    while (target_heading >= 360.0f) target_heading -= 360.0f;
  }
  
  #if NAV_DEBUG
  Serial.printf("[NAV DEBUG] Final target heading: %.1f deg\n", target_heading);
  #endif
  
  return target_heading;
}

static bool advance_to_next_waypoint() {
  return Navigation_advance_waypoint();
}