#include "gps.h"
#include <TinyGPSPlus.h>

#if !defined(ARDUINO_ARCH_ESP32)
  #error "This GPS module is written for ESP32/ESP32-S3."
#endif

// Use UART1 for GPS
HardwareSerial GPS_Serial(1);
static TinyGPSPlus gps;

static GpsFix s_lastFix;
static uint32_t s_lastFeedMs   = 0;   // last byte feed time
static uint32_t s_lastReportMs = 0;   // last time we printed to Serial

// Convert TinyGPS++ fields into our struct
static void snapshotFix(GpsFix &f) {
  f.valid  = gps.location.isValid();
  f.lat    = gps.location.isValid() ? gps.location.lat() : 0.0;
  f.lon    = gps.location.isValid() ? gps.location.lng() : 0.0;
  f.alt_m  = gps.altitude.isValid() ? gps.altitude.meters() : 0.0;
  f.speed_kmph = gps.speed.isValid() ? gps.speed.kmph() : 0.0;
  f.course_deg = gps.course.isValid() ? gps.course.deg() : 0.0;
  f.sats   = gps.satellites.isValid() ? (uint8_t)gps.satellites.value() : 0;
  f.hdop   = gps.hdop.isValid() ? gps.hdop.hdop() : 0.0;

  if (gps.date.isValid()) {
    f.year  = gps.date.year();
    f.month = gps.date.month();
    f.day   = gps.date.day();
  } else {
    f.year = f.month = f.day = 0;
  }
  if (gps.time.isValid()) {
    f.hour   = gps.time.hour();
    f.minute = gps.time.minute();
    f.second = gps.time.second();
  } else {
    f.hour = f.minute = f.second = 0;
  }

  // Age of the location fix in ms (0 if fresh)
  f.fix_age_ms = gps.location.age();
}

void GPS_begin(int rxPin, int txPin, uint32_t baud) {
  // Begin UART1 on chosen pins (works for ESP32 & ESP32-S3)
  GPS_Serial.begin(baud, SERIAL_8N1, rxPin, txPin);
  s_lastReportMs = millis();
  s_lastFeedMs   = millis();

  Serial.printf("[GPS] UART1 started @ %lu baud (RX=%d, TX=%d)\r\n",
                (unsigned long)baud, rxPin, txPin);
  Serial.println(F("[GPS] Expecting NMEA from NEO-6M / similar. Reporting every 5s."));
}

void GPS_loop() {
  // Feed parser with all available bytes
  while (GPS_Serial.available()) {
    gps.encode(GPS_Serial.read());
    s_lastFeedMs = millis();
  }

  // Keep the public fix cache fresh for status output and ESP-NOW users.
  // Time/date can be valid before latitude/longitude, so update it every loop.
  snapshotFix(s_lastFix);

  // Every GPS_REPORT_MS, print the latest snapshot
  const uint32_t now = millis();
  if (now - s_lastReportMs >= GPS_REPORT_MS) {
    s_lastReportMs = now;

    if (s_lastFix.valid) {
      // UTC date/time string (if available)
      if (s_lastFix.year) {
        Serial.printf("[GPS] %04u-%02u-%02u %02u:%02u:%02uZ  ",
                      s_lastFix.year, s_lastFix.month, s_lastFix.day,
                      s_lastFix.hour, s_lastFix.minute, s_lastFix.second);
      } else {
        Serial.print(F("[GPS] -- no UTC time --  "));
      }

      Serial.printf("lat=%.6f  lon=%.6f  alt=%.1f m  spd=%.2f km/h  crs=%.1f°  sats=%u  hdop=%.1f  age=%ums\r\n",
                    s_lastFix.lat, s_lastFix.lon, s_lastFix.alt_m,
                    s_lastFix.speed_kmph, s_lastFix.course_deg,
                    s_lastFix.sats, s_lastFix.hdop, (unsigned)s_lastFix.fix_age_ms);
    } else {
      // No valid fix yet — still report something so you know it’s alive
      const bool anyBytes = (now - s_lastFeedMs) < 2000;
      Serial.printf("[GPS] NO FIX  (bytes:%s  sats:%u  hdop:%s)\r\n",
                    anyBytes ? "OK" : "none",
                    gps.satellites.isValid() ? (unsigned)gps.satellites.value() : 0u,
                    gps.hdop.isValid() ? String(gps.hdop.hdop(), 1).c_str() : "n/a");
    }
  }
}

bool GPS_getFix(GpsFix &out) {
  out = s_lastFix;
  return s_lastFix.valid;
}

uint32_t GPS_lastReportAt() {
  return s_lastReportMs;
}
