#include "compass_mmc5603.h"

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_MMC56x3.h>
#include <Preferences.h>

// ================= Internal state =================
static Adafruit_MMC5603 g_mmc;
static bool   g_inited = false;
static bool   g_online = false;

// NVS storage namespace for cal (kept same keys you already use)
static const char* NVS_NS = "ccal";

// Hard-iron min/max saved by external CCAL (units: ~0.1 uT integer domain)
static int16_t g_min_x=0, g_min_y=0, g_min_z=0;
static int16_t g_max_x=0, g_max_y=0, g_max_z=0;
static bool    g_has_minmax = false;

// Derived offsets/scales (float), computed from min/max
static float   g_off_x=0, g_off_y=0, g_off_z=0;
static float   g_scl_x=1, g_scl_y=1, g_scl_z=1;

// Declination & yaw offset (deg)
static float   g_decl_deg = 0.0f;
static float   g_yaw_off_deg = 0.0f;

// Smoothing (unit-vector EMA)
static float   s_vx = 1.0f, s_vy = 0.0f;
static bool    s_has_smooth = false;
static const float SMOOTH_ALPHA = 0.15f;  // 0..1, higher = more responsive

// Cached heading (deg) and timestamp
static float    g_last_heading_deg = NAN;
static uint32_t g_last_sample_ms   = 0;

// Rate limit I2C reads in compass_loop to avoid starving other subsystems (20 Hz)
static const uint32_t READ_PERIOD_MS = 50;

// ---------------- Utilities ----------------
static inline float wrap360(float a){ while(a<0)a+=360; while(a>=360)a-=360; return a; }
static inline float deg2rad(float d){ return d*0.01745329252f; }
static inline float rad2deg(float r){ return r*57.2957795f; }

static void recompute_from_minmax(){
  int sx = (int)g_max_x - (int)g_min_x;
  int sy = (int)g_max_y - (int)g_min_y;
  int sz = (int)g_max_z - (int)g_min_z;
  g_has_minmax = (sx>10 && sy>10 && sz>10);
  if (g_has_minmax){
    g_off_x = 0.5f * (g_min_x + g_max_x);
    g_off_y = 0.5f * (g_min_y + g_max_y);
    g_off_z = 0.5f * (g_min_z + g_max_z);
    g_scl_x = sx ? (2.0f / (float)sx) : 1.0f;
    g_scl_y = sy ? (2.0f / (float)sy) : 1.0f;
    g_scl_z = sz ? (2.0f / (float)sz) : 1.0f;
  } else {
    g_off_x = g_off_y = g_off_z = 0.0f;
    g_scl_x = g_scl_y = g_scl_z = 1.0f;
  }
}

static void load_minmax_from_nvs(){
  Preferences p;
  if (!p.begin(NVS_NS, true)) { g_has_minmax=false; return; }
  g_min_x = p.getShort("min_x", 0);
  g_min_y = p.getShort("min_y", 0);
  g_min_z = p.getShort("min_z", 0);
  g_max_x = p.getShort("max_x", 0);
  g_max_y = p.getShort("max_y", 0);
  g_max_z = p.getShort("max_z", 0);
  p.end();
  recompute_from_minmax();
}

// compute instantaneous heading from the given sample (rx/ry/rz in ~0.1 uT)
static float compute_heading_from_sample(float rx, float ry, float rz){
  if (g_has_minmax){
    rx = (rx - g_off_x) * g_scl_x;
    ry = (ry - g_off_y) * g_scl_y;
    rz = (rz - g_off_z) * g_scl_z;
  }
  return wrap360(rad2deg(atan2f(ry, rx)) + g_decl_deg + g_yaw_off_deg);
}

// ---------------- Core ----------------
bool compass_begin(int sda_pin, int scl_pin, uint32_t i2c_hz, float declination_deg){
  if (g_inited) return g_online;
  g_decl_deg = declination_deg;

  Serial.println(F("[COMPASS] Initializing compass..."));
  Wire.begin(sda_pin, scl_pin);
  if (i2c_hz) Wire.setClock(i2c_hz);

  // Try both common MMC5603 addresses
  Serial.println(F("[COMPASS] MMC5603: probing 0x30..."));
  g_online = g_mmc.begin(0x30, &Wire);
  if (!g_online){
    Serial.println(F("[COMPASS] MMC5603: probing 0x31..."));
    g_online = g_mmc.begin(0x31, &Wire);
  }
  if (g_online){
    Serial.println(F("[COMPASS] MMC5603 online"));
  } else {
    Serial.println(F("[COMPASS] MMC5603 NOT FOUND"));
  }

  load_minmax_from_nvs();
  g_inited = true;
  g_last_sample_ms = 0;
  s_has_smooth = false;

  if (g_online){
    // Prime one sample
    sensors_event_t mag;
    if (g_mmc.getEvent(&mag)){
      float rx = mag.magnetic.x * 10.0f;
      float ry = mag.magnetic.y * 10.0f;
      float rz = mag.magnetic.z * 10.0f;
      g_last_heading_deg = compute_heading_from_sample(rx, ry, rz);
      s_vx = cosf(deg2rad(g_last_heading_deg));
      s_vy = sinf(deg2rad(g_last_heading_deg));
      s_has_smooth = true;
      Serial.printf("[COMPASS] Initialized OK, heading=%.1f°\n", g_last_heading_deg);
    }
  }
  return g_online;
}

void compass_loop(){
  if (!g_online) return;

  uint32_t now = millis();
  if (now - g_last_sample_ms < READ_PERIOD_MS) return; // rate limit I2C
  g_last_sample_ms = now;

  sensors_event_t mag;
  if (!g_mmc.getEvent(&mag)) return;

  float rx = mag.magnetic.x * 10.0f;
  float ry = mag.magnetic.y * 10.0f;
  float rz = mag.magnetic.z * 10.0f;

  float hdg = compute_heading_from_sample(rx, ry, rz);
  g_last_heading_deg = hdg;

  // Update smoothing
  float cx = cosf(deg2rad(hdg));
  float cy = sinf(deg2rad(hdg));
  if (!s_has_smooth){ s_vx=cx; s_vy=cy; s_has_smooth=true; }
  else {
    s_vx = (1.0f - SMOOTH_ALPHA)*s_vx + SMOOTH_ALPHA*cx;
    s_vy = (1.0f - SMOOTH_ALPHA)*s_vy + SMOOTH_ALPHA*cy;
    float n = sqrtf(s_vx*s_vx + s_vy*s_vy);
    if (n > 1e-6f){ s_vx/=n; s_vy/=n; }
  }
}

bool compass_is_online(){ return g_online; }

bool compass_sample_raw(int16_t& x, int16_t& y, int16_t& z){
  if (!g_online) return false;
  sensors_event_t mag;
  if (!g_mmc.getEvent(&mag)) return false;
  x = (int16_t)lrintf(mag.magnetic.x * 10.0f);
  y = (int16_t)lrintf(mag.magnetic.y * 10.0f);
  z = (int16_t)lrintf(mag.magnetic.z * 10.0f);
  return true;
}

float compass_get_heading_deg(){
  // Return last cached; caller should call compass_loop() periodically
  return g_last_heading_deg;
}

float compass_get_heading_deg_smooth(){
  if (!g_online) return NAN;

  // Always attempt a fresh sample so smoothing progresses even if compass_loop() isn't called.
  sensors_event_t mag;
  if (g_mmc.getEvent(&mag)){
    float rx = mag.magnetic.x * 10.0f;
    float ry = mag.magnetic.y * 10.0f;
    float rz = mag.magnetic.z * 10.0f;
    float hdg = compute_heading_from_sample(rx, ry, rz);

    float cx = cosf(deg2rad(hdg));
    float cy = sinf(deg2rad(hdg));
    if (!s_has_smooth){ s_vx=cx; s_vy=cy; s_has_smooth=true; }
    else {
      s_vx = (1.0f - SMOOTH_ALPHA)*s_vx + SMOOTH_ALPHA*cx;
      s_vy = (1.0f - SMOOTH_ALPHA)*s_vy + SMOOTH_ALPHA*cy;
      float n = sqrtf(s_vx*s_vx + s_vy*s_vy);
      if (n > 1e-6f){ s_vx/=n; s_vy/=n; }
    }
    g_last_heading_deg = hdg;
  } else if (!s_has_smooth) {
    // Initialize once if smoothing not ready
    if (!isnan(g_last_heading_deg)){
      s_vx = cosf(deg2rad(g_last_heading_deg));
      s_vy = sinf(deg2rad(g_last_heading_deg));
      s_has_smooth = true;
    } else {
      return NAN;
    }
  }

  return wrap360(rad2deg(atan2f(s_vy, s_vx)));
}

void compass_set_calibration_minmax(int16_t min_x,int16_t min_y,int16_t min_z,
                                    int16_t max_x,int16_t max_y,int16_t max_z){
  g_min_x=min_x; g_min_y=min_y; g_min_z=min_z;
  g_max_x=max_x; g_max_y=max_y; g_max_z=max_z;
  // Recompute and persist
  recompute_from_minmax();
  Preferences p; if (p.begin(NVS_NS, false)){
    p.putShort("min_x", g_min_x);
    p.putShort("min_y", g_min_y);
    p.putShort("min_z", g_min_z);
    p.putShort("max_x", g_max_x);
    p.putShort("max_y", g_max_y);
    p.putShort("max_z", g_max_z);
    p.end();
  }
}

void compass_set_yaw_offset_deg(float deg){
  g_yaw_off_deg = deg;
}

void compass_setreset_now(){
  // No-op for this library version (older Adafruit_MMC56x3 without setReset())
}
