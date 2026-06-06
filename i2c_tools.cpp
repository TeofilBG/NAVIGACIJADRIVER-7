#include "i2c_tools.h"
#include <Wire.h>

void i2c_scan(TwoWire &bus, const char *label) {
  Serial.printf("I2C scan on %s:\r\n", label);
  uint8_t count = 0;
  for (uint8_t addr = 1; addr < 127; addr++) {
    bus.beginTransmission(addr);
    uint8_t err = bus.endTransmission();
    if (err == 0) {
      Serial.printf("  - 0x%02X\r\n", addr);
      count++;
    }
  }
  if (!count) Serial.println("  (none)");
}
