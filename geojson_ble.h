#pragma once
#include <Arduino.h>
#include <vector>

struct Waypoint {
  double lat;   // degrees
  double lon;   // degrees
  double alt;   // meters (optional; 0.0 if absent)
};

// ---- Public API ----
void GeoBLE_begin(const char* deviceName = "NAVIGACIJA");  // start BLE server (NimBLE)
void GeoBLE_loop();                                         // call every loop()

// Waypoint store helpers
void GeoRoute_clear();
size_t GeoRoute_count();
bool GeoRoute_get(size_t idx, Waypoint& out);
const std::vector<Waypoint>& GeoRoute_all();                // const ref for read-only
void GeoRoute_printToSerial();                               // pretty-print all WPs
