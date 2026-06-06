#pragma once
#include <Arduino.h>
#include <Wire.h>   // <-- make TwoWire visible to any TU that includes this header

// Scan an I2C bus and print any devices found.
void i2c_scan(TwoWire &bus, const char *label);
