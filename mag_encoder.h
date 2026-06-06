#pragma once
#include <Arduino.h>

// ===== Public API (kept stable) =====

// Initialize backend (i2c_freq_hz kept for compatibility; ignored here).
// sda_pin_asA = encoder A pin, scl_pin_asB = encoder B pin.
bool  mag_begin(int sda_pin_asA, int scl_pin_asB, uint32_t i2c_freq_hz = 0);

// Synthetic 12-bit raw (0..4095) from current 0..180° position
uint16_t mag_get_raw12();

// Legacy angle (0..360) – mapped as pos*2.0
float mag_get_angle_deg();

// Kept for compatibility (no-op here)
void  mag_set_zero_to_current();

// Same as mag_get_angle_deg()
float mag_get_angle_deg_zeroed();

// Optional invert (swap CW/CCW sense)
void  mag_set_invert(bool invert);

// ---- RUNTIME TUNING (restored) ----

// How many encoder *counts* correspond to 0..180°.
// 180 ≈ 1°/count; 90 ≈ 2°/count; 60 ≈ 3°/count; 360 ≈ 0.5°/count.
void  mag_set_counts_per_180(int counts);
int   mag_get_counts_per_180();

// Schmitt hysteresis on reported degrees (to suppress 1–2° chatter).
// Example: out=1, in=0 (minimal stickiness). Set both 0 to disable.
void  mag_set_hysteresis(int out_deg, int in_deg);

// Debounce on the ISR in microseconds (typical 400–1200us).
void  mag_set_debounce_us(uint32_t us);

// ---- ENCODER MODE API (used by your app) ----
void  mag_encoder_init();                 // sets 0..180, start at 90
int   mag_get_encoder_position();         // returns 0..180 (filtered + hysteresis)
void  mag_reset_encoder();                // set to 90
void  mag_set_encoder_position(int pos);  // clamp 0..180

// Debug
extern "C" uint32_t mag_debug_isr_ticks();
