#include "espnow_gps.h"
#include <WiFi.h>
#include <esp_now.h>
#include <string.h>

// ---------- Config ----------
static const uint32_t SEND_PERIOD_MS = 5000;

// ---------- State ----------
static uint8_t g_peer_mac[6] = {0xFF,0xFF,0xFF,0xFF,0xFF,0xFF}; // default broadcast
static bool    g_peer_added   = false;
static uint32_t g_last_send   = 0;
static uint32_t g_seq         = 0;

// Simple, robust civil date → Unix epoch (UTC). Returns 0 if date fields look invalid.
static uint32_t makeEpoch(uint16_t y, uint8_t m, uint8_t d,
                          uint8_t hh, uint8_t mm, uint8_t ss) {
  if (y < 1970 || m < 1 || m > 12 || d < 1 || d > 31) return 0;
  // Howard Hinnant days-from-civil algorithm (condensed)
  int32_t yy = (int32_t)y;
  int32_t mm2 = (int32_t)m;
  int32_t dd = (int32_t)d;
  yy -= (mm2 <= 2);
  const int32_t era = (yy >= 0 ? yy : yy - 399) / 400;
  const uint32_t yoe = (uint32_t)(yy - era * 400);
  const uint32_t doy = (153 * (mm2 + (mm2 > 2 ? -3 : 9)) + 2) / 5 + dd - 1;
  const uint32_t doe = yoe * 365 + yoe / 4 - yoe / 100 + doy;
  const int64_t days = (int64_t)era * 146097 + (int64_t)doe - 719468; // days since 1970-01-01
  if (days < 0) return 0;
  return (uint32_t)(days * 86400 + (int64_t)hh * 3600 + (int64_t)mm * 60 + ss);
}

void GPSNOW_setPeer(const uint8_t mac[6]) {
  if (!mac) return;
  memcpy(g_peer_mac, mac, 6);
  g_peer_added = false; // force re-add with new address
}

static void ensurePeerAdded() {
  if (g_peer_added) return;

  esp_now_peer_info_t p{};
  memcpy(p.peer_addr, g_peer_mac, 6);
  p.channel = 0;               // same channel as current WiFi STA
  p.encrypt = false;

  // Ignore "peer exists" as success; add or update both OK
  esp_err_t st = esp_now_add_peer(&p);
  if (st == ESP_ERR_ESPNOW_EXIST) st = ESP_OK;
  g_peer_added = (st == ESP_OK);
}

void GPSNOW_begin() {
  // Do NOT call esp_now_init() here; assume your main ESP-NOW module already did it.
  ensurePeerAdded();
}

bool GPSNOW_send(uint32_t epoch_sec, float speed_kmh, bool valid) {
  ensurePeerAdded();
  if (!g_peer_added) return false;

  GpsTimeSpeedMsg msg{};
  msg.seq        = ++g_seq;
  msg.epoch_sec  = epoch_sec;
  msg.speed_kmh  = speed_kmh;
  msg.valid      = valid ? 1 : 0;

  return (esp_now_send(g_peer_mac, reinterpret_cast<const uint8_t*>(&msg), sizeof(msg)) == ESP_OK);
}

void GPSNOW_loop() {
  const uint32_t now = millis();
  if (now - g_last_send < SEND_PERIOD_MS) return;
  g_last_send = now;

  GpsFix fix;
  GPS_getFix(fix); // copies most recent fix from gps.cpp

  uint32_t epoch = 0;
  if (fix.year) {
    epoch = makeEpoch(fix.year, fix.month, fix.day, fix.hour, fix.minute, fix.second);
  }

  (void)GPSNOW_send(epoch, fix.speed_kmph, fix.valid);
}
