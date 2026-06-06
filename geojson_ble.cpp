#include "geojson_ble.h"
#include <NimBLEDevice.h>
#include <string>
#include <cctype>
#include <cstdlib>  // strtod

// Prefer larger MTU for chunkier uploads from terminal apps
static const uint16_t PREFERRED_MTU  = 185;
static const uint32_t IDLE_PARSE_MS  = 800;   // idle gap to auto-parse
static const uint16_t MAX_WP         = 4096;  // soft cap

// Nordic UART (NUS) UUIDs (BLE terminal-friendly)
static const char* UUID_NUS_SERVICE = "6E400001-B5A3-F393-E0A9-E50E24DCCA9E";
static const char* UUID_NUS_RX      = "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"; // Write
static const char* UUID_NUS_TX      = "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"; // Notify

// Runtime state
static NimBLECharacteristic* s_txChar = nullptr; // optional notify
static std::string s_buf;                        // raw incoming GeoJSON text
static bool s_receiving = false;
static uint32_t s_last_rx_ms = 0;

// Waypoint storage
static std::vector<Waypoint> g_route;

// --- helpers ---
static inline void trim_space(const char*& p) {
  while (*p && isspace((unsigned char)*p)) ++p;
}
static bool parse_number(const char*& p, double& out) {
  trim_space(p);
  char* endp = nullptr;
  double v = strtod(p, &endp);
  if (endp == p) return false;
  out = v; p = endp; return true;
}

// Extract lon/lat/alt triples from any GeoJSON (LineString, MultiLineString, FeatureCollection…)
static void parse_geojson_coords(const std::string& src, std::vector<Waypoint>& out) {
  const char* p = src.c_str();
  while (*p) {
    if (*p == '[') {
      const char* q = p + 1;
      double a=0, b=0, c=0;
      if (!parse_number(q, a)) { ++p; continue; }
      trim_space(q);
      if (*q != ',') { ++p; continue; }
      ++q;
      if (!parse_number(q, b)) { ++p; continue; }
      trim_space(q);
      if (*q == ',') {
        ++q;
        if (!parse_number(q, c)) c = 0.0;
        trim_space(q);
      } else {
        c = 0.0;
      }
      if (*q == ']') {
        if (out.size() < MAX_WP) out.push_back(Waypoint{ b, a, c }); // [lon,lat,(alt)]
        p = q + 1;
        continue;
      }
    }
    ++p;
  }
}

static void finish_and_parse() {
  if (!s_receiving || s_buf.empty()) return;

  Serial.println(F("[BLE] Route input: IDLE -> parsing GeoJSON"));
  std::vector<Waypoint> parsed;
  parsed.reserve(128);
  parse_geojson_coords(s_buf, parsed);

  // Commit
  g_route = std::move(parsed);
  s_buf.clear();
  s_receiving = false;

  // Print to Serial upon upload
  GeoRoute_printToSerial();
}

// ---------- Callbacks (compatible with multiple NimBLE versions) ----------
#if __has_include(<NimBLEConnInfo.h>)
  #include <NimBLEConnInfo.h>
  #define GEO_HAVE_CONNINFO 1
#endif

class RxCallbacks : public NimBLECharacteristicCallbacks {
public:
  // Newer API:
#ifdef GEO_HAVE_CONNINFO
  void onWrite(NimBLECharacteristic* ch, NimBLEConnInfo&) {
    std::string value = ch->getValue();
    if (value.empty()) return;
    if (!s_receiving) {
      s_receiving = true;
      Serial.println(F("[BLE] Route input: ON (GeoJSON text)"));
    }
    s_last_rx_ms = millis();
    s_buf.append(value);
    if (s_buf.find("\n!END") != std::string::npos || s_buf.find("\r\n!END") != std::string::npos) {
      finish_and_parse();
    } else if (s_buf.size() > 512u * 1024u) {
      Serial.println(F("[BLE] Input too large, truncating and parsing…"));
      finish_and_parse();
    }
  }
#endif

  // Older API:
  void onWrite(NimBLECharacteristic* ch) {
#ifndef GEO_HAVE_CONNINFO
    std::string value = ch->getValue();
    if (value.empty()) return;
    if (!s_receiving) {
      s_receiving = true;
      Serial.println(F("[BLE] Route input: ON (GeoJSON text)"));
    }
    s_last_rx_ms = millis();
    s_buf.append(value);
    if (s_buf.find("\n!END") != std::string::npos || s_buf.find("\r\n!END") != std::string::npos) {
      finish_and_parse();
    } else if (s_buf.size() > 512u * 1024u) {
      Serial.println(F("[BLE] Input too large, truncating and parsing…"));
      finish_and_parse();
    }
#else
    // If both are present, keep logic only in the (char, ConnInfo&) overload.
    (void)ch;
#endif
  }
};

class ServerCallbacks : public NimBLEServerCallbacks {
public:
  void onConnect(NimBLEServer* pServer) {
    (void)pServer;
    Serial.println(F("[BLE] Central connected"));
  }
  void onDisconnect(NimBLEServer* pServer) {
    (void)pServer;
    Serial.println(F("[BLE] Central disconnected"));
    NimBLEDevice::startAdvertising();
  }
};

// ---------- Public API ----------
void GeoBLE_begin(const char* deviceName) {
  // Initialize NimBLE with device name
  NimBLEDevice::init(deviceName);
  NimBLEDevice::setPower(ESP_PWR_LVL_P9);
  NimBLEDevice::setSecurityAuth(false, false, false);

  NimBLEServer* server = NimBLEDevice::createServer();
  server->setCallbacks(new ServerCallbacks());

  NimBLEService* svc = server->createService(UUID_NUS_SERVICE);

  // TX (notify)
  s_txChar = svc->createCharacteristic(
    UUID_NUS_TX,
    NIMBLE_PROPERTY::NOTIFY
  );

  // RX (write)
  NimBLECharacteristic* rx = svc->createCharacteristic(
    UUID_NUS_RX,
    NIMBLE_PROPERTY::WRITE | NIMBLE_PROPERTY::WRITE_NR
  );
  rx->setCallbacks(new RxCallbacks());

  svc->start();

  // Try enhanced advertising setup (with error handling)
  NimBLEAdvertising* adv = NimBLEDevice::getAdvertising();
  adv->addServiceUUID(UUID_NUS_SERVICE);
  
  // Try to set advertising data with device name
  try {
    NimBLEAdvertisementData advData;
    advData.setShortName(deviceName);
    advData.setCompleteServices(BLEUUID(UUID_NUS_SERVICE));
    adv->setAdvertisementData(advData);
    
    // Try scan response data
    NimBLEAdvertisementData scanData;
    scanData.setName(deviceName);
    adv->setScanResponseData(scanData);
    
    Serial.printf("[BLE] Enhanced advertising configured for: '%s'\n", deviceName);
  } catch (...) {
    Serial.println("[BLE] Enhanced advertising failed, using basic mode");
  }
  
  // Start advertising
  adv->start();

  // Set MTU
  NimBLEDevice::setMTU(PREFERRED_MTU);

  Serial.printf("[BLE] NUS ready. Device name: '%s'\n", deviceName);
  Serial.println("[BLE] Connect with BLE terminal app and paste GeoJSON data");
  Serial.println("[BLE] If device shows as MAC address, this is a library limitation");
}

void GeoBLE_loop() {
  if (s_receiving && (millis() - s_last_rx_ms) > IDLE_PARSE_MS) {
    finish_and_parse();
  }
}

// ---- Route helpers ----
void GeoRoute_clear() { g_route.clear(); }
size_t GeoRoute_count() { return g_route.size(); }
bool GeoRoute_get(size_t idx, Waypoint& out) {
  if (idx >= g_route.size()) return false;
  out = g_route[idx]; return true;
}
const std::vector<Waypoint>& GeoRoute_all() { return g_route; }

void GeoRoute_printToSerial() {
  Serial.printf("[BLE] Parsed %u waypoint(s)\r\n", (unsigned)g_route.size());
  for (size_t i = 0; i < g_route.size(); ++i) {
    const auto& w = g_route[i];
    Serial.printf("WP %u: lat=%.8f, lon=%.8f, alt=%.2f\r\n",
                  (unsigned)i, w.lat, w.lon, w.alt);
  }
  if (g_route.empty()) {
    Serial.println(F("[BLE] No coordinates found in provided GeoJSON."));
  }
}