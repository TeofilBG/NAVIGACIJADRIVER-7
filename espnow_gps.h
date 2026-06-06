#pragma once
#include <Arduino.h>
#include "gps.h"   // uses GpsFix + GPS_getFix()

// Packet sent from DRIVER -> CONTROLLER
// UTC epoch seconds (0 if unknown), current speed in km/h, and a valid flag.
typedef struct __attribute__((packed)) {
  uint32_t seq;         // monotonically increasing
  uint32_t epoch_sec;   // Unix time (UTC). 0 if GPS time not valid yet
  float    speed_kmh;   // TinyGPS++ speed.kmph()
  uint8_t  valid;       // 1 when GPS fix is valid (lat/lon), else 0
  uint8_t  _pad[3];     // keep 16-byte alignment
} GpsTimeSpeedMsg;

// Optional: set the peer MAC (controller). If not set, will use broadcast FF:FF:FF:FF:FF:FF.
void GPSNOW_setPeer(const uint8_t mac[6]);

// Begin the GPS→ESP-NOW bridge (adds peer if needed). Call AFTER your esp_now_init().
void GPSNOW_begin();

// Call in loop(); every ~5 s it reads GPS and sends a GpsTimeSpeedMsg.
void GPSNOW_loop();

// Manual send (if you want to trigger yourself).
bool GPSNOW_send(uint32_t epoch_sec, float speed_kmh, bool valid);
