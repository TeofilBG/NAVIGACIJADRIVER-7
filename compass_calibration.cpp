#include "compass_calibration.h"
#include "compass_mmc5603.h"

#include <nvs_flash.h>
#include <nvs.h>

static const char* NVS_NS  = "compass";
static const char* NVS_KEY = "minmax";

typedef struct {
  int16_t min_x, min_y, min_z;
  int16_t max_x, max_y, max_z;
  uint32_t crc;
} CalBlob;

static uint32_t crc32_accum(uint32_t crc, const uint8_t* p, size_t n){
  crc = ~crc;
  for (size_t i=0;i<n;i++){
    uint8_t b = p[i];
    crc ^= b;
    for (int k=0;k<8;k++){
      uint32_t m = -(crc & 1u);
      crc = (crc >> 1) ^ (0xEDB88320u & m);
    }
  }
  return ~crc;
}

static bool nvs_ready() {
  // Make sure NVS is up (idempotent)
  esp_err_t e = nvs_flash_init();
  if (e == ESP_ERR_NVS_NO_FREE_PAGES || e == ESP_ERR_NVS_NEW_VERSION_FOUND){
    ESP_ERROR_CHECK(nvs_flash_erase());
    e = nvs_flash_init();
  }
  return (e == ESP_OK);
}

static bool nvs_save_minmax(const CalBlob& blob){
  if (!nvs_ready()) return false;
  nvs_handle_t h;
  esp_err_t e = nvs_open(NVS_NS, NVS_READWRITE, &h);
  if (e != ESP_OK) return false;
  e = nvs_set_blob(h, NVS_KEY, &blob, sizeof(blob));
  if (e == ESP_OK) e = nvs_commit(h);
  nvs_close(h);
  return e == ESP_OK;
}

static bool nvs_load_minmax(CalBlob& out){
  if (!nvs_ready()) return false;
  nvs_handle_t h;
  size_t sz = sizeof(out);
  esp_err_t e = nvs_open(NVS_NS, NVS_READONLY, &h);
  if (e != ESP_OK) return false;
  e = nvs_get_blob(h, NVS_KEY, &out, &sz);
  nvs_close(h);
  if (e != ESP_OK || sz != sizeof(out)) return false;

  uint32_t crc = out.crc; CalBlob tmp = out; tmp.crc = 0;
  uint32_t calc = crc32_accum(0, reinterpret_cast<const uint8_t*>(&tmp), sizeof(tmp));
  return crc == calc;
}

bool CompassCalib_load_from_nvs(){
  CalBlob b{};
  if (!nvs_load_minmax(b)) return false;
  compass_set_calibration_minmax(b.min_x,b.min_y,b.min_z,b.max_x,b.max_y,b.max_z);
  Serial.printf("[CAL] Loaded NVS minmax: min(%d,%d,%d) max(%d,%d,%d)\n",
                b.min_x,b.min_y,b.min_z,b.max_x,b.max_y,b.max_z);
  return true;
}

bool CompassCalib_store_direct(int16_t min_x, int16_t min_y, int16_t min_z,
                               int16_t max_x, int16_t max_y, int16_t max_z){
  // Apply now
  compass_set_calibration_minmax(min_x,min_y,min_z, max_x,max_y,max_z);
  // Save blob
  CalBlob blob{min_x,min_y,min_z, max_x,max_y,max_z, 0};
  blob.crc = crc32_accum(0, reinterpret_cast<const uint8_t*>(&blob), sizeof(blob));
  if (!nvs_save_minmax(blob)) return false;
  Serial.println("[CAL] Stored to NVS");
  return true;
}

bool CompassCalib_run_and_store(uint32_t duration_ms,
                                int16_t &min_x, int16_t &min_y, int16_t &min_z,
                                int16_t &max_x, int16_t &max_y, int16_t &max_z){
  // If duration==0, just store whatever mins/maxes were passed (or current)
  if (duration_ms == 0) {
    return CompassCalib_store_direct(min_x,min_y,min_z, max_x,max_y,max_z);
  }

  // init ranges
  min_x = min_y = min_z =  32767;
  max_x = max_y = max_z = -32768;

  const uint32_t start = millis();
  uint32_t last_print = 0;

  Serial.println("[CAL] Starting live compass calibration");
  Serial.println("[CAL] Rotate the boat/device slowly in ALL orientations...");

  while (millis() - start < duration_ms){
    int16_t x,y,z;
    if (compass_sample_raw(x,y,z)){
      if (x < min_x) min_x = x; if (x > max_x) max_x = x;
      if (y < min_y) min_y = y; if (y > max_y) max_y = y;
      if (z < min_z) min_z = z; if (z > max_z) max_z = z;
    }
    if (millis() - last_print > 500){
      last_print = millis();
      Serial.printf("[CAL] min{%d,%d,%d}  max{%d,%d,%d}\n", min_x,min_y,min_z, max_x,max_y,max_z);
    }
    delay(20);
  }

  if ((max_x-min_x) < 100 || (max_y-min_y) < 100 || (max_z-min_z) < 100){
    Serial.println("[CAL] FAILED: not enough motion; try again.");
    return false;
  }

  return CompassCalib_store_direct(min_x,min_y,min_z, max_x,max_y,max_z);
}
