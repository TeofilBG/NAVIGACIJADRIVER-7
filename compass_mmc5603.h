#pragma once
#include <Arduino.h>

// Drop-in MMC5603 replacement exposing the legacy compass_* API your sketch calls.
// Implements init, loop, raw sampling, min/max calibration, yaw offset, and smoothed heading.
// Compatible with Adafruit_MMC56x3 without setReset() API (older versions).

// ---- Legacy API expected by your sketch ----
bool  compass_begin(int sda_pin, int scl_pin, uint32_t i2c_hz, float declination_deg);
void  compass_loop();                                // call in loop() (optional but recommended)
bool  compass_is_online();                           // sensor present

// Raw sample used by your CCAL flow (units: ~0.1 uT integer)
bool  compass_sample_raw(int16_t& x, int16_t& y, int16_t& z);

// Instant heading (deg 0..360)
float compass_get_heading_deg();
// Smoothed heading (deg 0..360) used by your code
float compass_get_heading_deg_smooth();

// Calibration and offsets
void  compass_set_calibration_minmax(int16_t min_x,int16_t min_y,int16_t min_z,
                                     int16_t max_x,int16_t max_y,int16_t max_z);
void  compass_set_yaw_offset_deg(float deg);

// Optional: force set/reset if you want to hook it (no-op for this lib version)
void  compass_setreset_now();
