#pragma once
#include <Arduino.h>

// Run live compass calibration for `duration_ms`.
// Rotate the device slowly in all orientations while it runs.
// On success: applies calibration immediately and stores to NVS.
// Returns true on success; min/max are returned via refs.
bool CompassCalib_run_and_store(uint32_t duration_ms,
                                int16_t &min_x, int16_t &min_y, int16_t &min_z,
                                int16_t &max_x, int16_t &max_y, int16_t &max_z);

// Load calibration from NVS (if available) and apply it.
// Returns true if values were found and applied.
bool CompassCalib_load_from_nvs();

// NEW: just store/apply already-measured min/max to NVS (no timed run).
// Returns true on success.
bool CompassCalib_store_direct(int16_t min_x, int16_t min_y, int16_t min_z,
                               int16_t max_x, int16_t max_y, int16_t max_z);
