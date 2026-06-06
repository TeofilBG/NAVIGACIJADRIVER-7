#pragma once
#include <Arduino.h>

// ---------------- Config (override if you want) ----------------
// Default UART pins for classic ESP32 DevKit. If you’re on S3 or use other pins,
// just #define these BEFORE including gps.h, or pass pins to GPS_begin(...).
#ifndef GPS_RX_PIN   // GPS module TX -> this pin (ESP32 RX)
#define GPS_RX_PIN 16
#endif
#ifndef GPS_TX_PIN   // GPS module RX -> this pin (ESP32 TX) [often unused]
#define GPS_TX_PIN 17
#endif
#ifndef GPS_BAUD
#define GPS_BAUD   9600
#endif
#ifndef GPS_REPORT_MS
#define GPS_REPORT_MS 5000UL   // print to Serial every 5 seconds
#endif

struct GpsFix {
  bool   valid            = false;
  double lat              = 0.0;   // degrees
  double lon              = 0.0;   // degrees
  double alt_m            = 0.0;   // meters
  double speed_kmph       = 0.0;   // km/h
  double course_deg       = 0.0;   // degrees
  uint32_t fix_age_ms     = 0;     // how old the last fix is
  uint8_t  sats           = 0;
  double   hdop           = 0.0;   // 0 = unknown
  uint16_t year           = 0;     // UTC date/time if available
  uint8_t  month          = 0;
  uint8_t  day            = 0;
  uint8_t  hour           = 0;
  uint8_t  minute         = 0;
  uint8_t  second         = 0;
};

// ---- Public API ----
void GPS_begin(int rxPin = GPS_RX_PIN, int txPin = GPS_TX_PIN, uint32_t baud = GPS_BAUD);
void GPS_loop();                      // call in your loop()
bool GPS_getFix(GpsFix &out);         // copy the most recent fix (returns true if valid)
uint32_t GPS_lastReportAt();          // millis() when last 5s report was printed
