#include <Arduino.h>
#include <WiFi.h>

#include "espnow_driver.h"
#include "mag_encoder.h"  // AiEsp32RotaryEncoder wrapper (0..180)
#include "compass_mmc5603.h"
#include "i2c_tools.h"
#include "motor_bts7960.h"
#include "geojson_ble.h"  // BLE GeoJSON receiver
#include "gps.h"          // GPS reader (NEO-6M via UART1)
#include "espnow_gps.h"   // GPS over ESP-NOW bridge
#include "navigation.h"   // Navigation system
#include "config_nvs.h"   // Configuration management
#include "compass_calibration.h"


// -------------------- MACs --------------------
static const uint8_t DRIVER_LOCAL_MAC[6] = { 0xE0, 0x5A, 0x1B, 0xA0, 0x1E, 0x70 };
static const uint8_t CONTROLLER_PEER_MAC[6] = { 0x7E, 0x12, 0x34, 0x56, 0x78, 0x9A };

// -------------------- PINS --------------------
static const int ENC_PIN_A = 2;  // rotary A
static const int ENC_PIN_B = 15;  // rotary B

// Compass I2C pins
static const int COMPASS_I2C_SDA = 21;
static const int COMPASS_I2C_SCL = 22;
static const float COMPASS_DECLINATION_DEG = 0.0f;

#define ENABLE_COMPASS 1

// Function declaration
uint32_t mag_debug_isr_ticks();

void setup() {
  Serial.begin(9600);
  delay(500);

  Serial.println("===========================================");
  Serial.println("DriverESP NAVIGATION System v0.4 Starting...");
  Serial.println("===========================================");

  // Initialize BLE for route uploads
  GeoBLE_begin("NAVIGACIJA");

  // Initialize GPS
  GPS_begin();
  delay(200);

  // Config system
  if (!Config_begin()) Serial.println("[CFG] Failed to init config");
  else Serial.println("[CFG] Config OK");

  // Motor controller
  motor_bts7960_begin();
  Serial.println("[MOTOR] Motor controller initialized");

#if ENABLE_COMPASS
  Serial.println("[COMPASS] Initializing compass...");
  if (compass_begin(COMPASS_I2C_SDA, COMPASS_I2C_SCL, 50000, COMPASS_DECLINATION_DEG)) {
    // Apply saved calibration if present; otherwise you can still fall back to old constants
if (!CompassCalib_load_from_nvs()) {
  // optional fallback (only if you want a default the very first time)
  compass_set_calibration_minmax(-270, -443, -451, +419, +268, +163);
}
    compass_set_yaw_offset_deg(0.0f);
    float initial_heading = compass_get_heading_deg();
    Serial.printf("[COMPASS] Initialized OK, heading=%.1f°\n", initial_heading);
  } else {
    Serial.println("[COMPASS] Failed, running without compass");
  }
#else
  Serial.println("[COMPASS] Temporarily disabled");
#endif  

  // -------------------- ENCODER (original style) --------------------
  Serial.println("[ENCODER] Initializing helm encoder...");
  if (!mag_begin(ENC_PIN_A, ENC_PIN_B, 0)) {
    Serial.println("[ENCODER] init FAILED, check wiring");
  } else {
    // Apply config from NVS or defaults
    DriverConfig cfg = Config_get_current();
    int cps180 = cfg.encoder_counts_180 > 0 ? cfg.encoder_counts_180 : 180;
    mag_set_counts_per_180(cps180);    // counts → 180°
    mag_set_hysteresis(1, 0);          // suppress 1° chatter
    mag_set_debounce_us(700);          // debounce (0.7ms like original)

    mag_encoder_init();                // attach ISRs, start at 90°

    // Optional: invert if helm reversed
    // mag_set_invert(true);

    delay(100);
    int pos = mag_get_encoder_position();
    Serial.printf("[ENCODER] Ready pos=%d° (0..180) counts/180=%d\n", pos, cps180);
    Serial.printf("[ENCODER] Pin states: A=%d B=%d\n",
                  digitalRead(ENC_PIN_A), digitalRead(ENC_PIN_B));
  }
  // -----------------------------------------------------------------

  // ESP-NOW communication
  Serial.println("[ESPNOW] Initializing...");
  DriverNOW_begin(DRIVER_LOCAL_MAC, CONTROLLER_PEER_MAC);
  Serial.println("[ESPNOW] OK");
  Serial.printf("[ESPNOW] Local MAC: %s\n", WiFi.macAddress().c_str());

  // GPS → ESP-NOW bridge
  GPSNOW_setPeer(CONTROLLER_PEER_MAC);
  GPSNOW_begin();
  Serial.println("[GPS-NOW] Bridge initialized");

  // Navigation defaults
  NavigationConfig nav_cfg = Navigation_get_config();
  nav_cfg.waypoint_radius_m   = 20.0f;
  nav_cfg.cross_track_limit_m = 50.0f;
  nav_cfg.cross_track_gain    = 1.5f;
  nav_cfg.min_speed_mps       = 0.5f;
  nav_cfg.gps_timeout_ms      = 10000;
  Navigation_set_config(nav_cfg);

  if (GeoRoute_count() > 0) {
    Navigation_load_route_from_ble();
    Serial.printf("[SETUP] Auto-loaded %u waypoints\n", GeoRoute_count());
  } else {
    Serial.println("[SETUP] No waypoints found");
  }

  Serial.println("===========================================");
  Serial.println("DriverESP NAVIGATION System Ready v0.4");
  Serial.println("===========================================");

  DriverConfig cfg = Config_get_current();
  Serial.printf("Motor Duty=%u%% | AP MaxStep=%u° | AP Deadband=%u°\n",
                cfg.motor_duty_percent, cfg.ap_max_step_deg, cfg.ap_deadband_deg);
  Serial.printf("PID: KP=%.2f KI=%.3f KD=%.2f | Encoder=%u counts/180\n",
                cfg.pid_kp, cfg.pid_ki, cfg.pid_kd, cfg.encoder_counts_180);
  Serial.printf("PWM Freq=%lu Hz | NavPeriod=%u ms\n",
                cfg.pwm_frequency_hz, cfg.nav_update_period_ms);
  Serial.println("===========================================");
}

void loop() {
  DriverNOW_loop();
  compass_loop();
  GeoBLE_loop();
  GPS_loop();
  GPSNOW_loop();

  static size_t last_wp = 0;
  size_t now_wp = GeoRoute_count();
  if (now_wp != last_wp) {
    if (now_wp > 0) {
      Navigation_load_route_from_ble();
      Serial.printf("[WAYPOINT] Loaded %u WPs\n", now_wp);
    } else {
      Navigation_clear_route();
      Serial.println("[WAYPOINT] Route cleared");
    }
    last_wp = now_wp;
  }

  static uint32_t dbg = 0;
  uint32_t now = millis();
  if (now - dbg >= 2000) {
    dbg = now;

    int helm = mag_get_encoder_position();
    float heading = compass_get_heading_deg_smooth();
    uint32_t isr_ticks = mag_debug_isr_ticks();

    NavigationMode nav_mode = Navigation_get_mode();
    NavigationStatus nav_stat = Navigation_get_status();
    GpsFix gps_fix;
    GPS_getFix(gps_fix);

    Serial.printf("[STATUS] HELM=%d° HDG=%.1f° MODE=", helm, heading);
    if (DriverNOW_isAutopilotEnabled()) Serial.print("AUTOPILOT");
    else {
      switch(nav_mode){
        case NAV_MODE_MANUAL: Serial.print("MANUAL"); break;
        case NAV_MODE_AUTOPILOT: Serial.print("AUTOPILOT"); break;
        case NAV_MODE_NAVIGATION:
          Serial.printf("NAV(WP %u/%u)", nav_stat.current_waypoint+1, nav_stat.total_waypoints);
          break;
      }
    }
    Serial.printf(" | GPS=%s COMP=%s",
                  gps_fix.valid?"OK":"NO", compass_is_online()?"OK":"ERR");
    if (gps_fix.valid) {
      Serial.printf(" LAT=%.6f LON=%.6f SAT=%u HDOP=%.1f AGE=%ums",
                    gps_fix.lat, gps_fix.lon, gps_fix.sats, gps_fix.hdop,
                    (unsigned)gps_fix.fix_age_ms);
    } else if (gps_fix.year) {
      Serial.printf(" UTC=%04u-%02u-%02u %02u:%02u:%02uZ",
                    gps_fix.year, gps_fix.month, gps_fix.day,
                    gps_fix.hour, gps_fix.minute, gps_fix.second);
    }
    Serial.println();

    DriverConfig cfg = Config_get_current();
    Serial.printf("[TECH] DUTY=%u%% A=%d B=%d ISR=%u\n",
                  cfg.motor_duty_percent,
                  digitalRead(ENC_PIN_A), digitalRead(ENC_PIN_B),
                  isr_ticks);
  }
  delay(5);
}
