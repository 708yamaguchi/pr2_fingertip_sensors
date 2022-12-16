#include <M5Stack.h>
#include <Wire.h>

#define I2C_MASTER
#include "symlink_libs/pfs.h"

void setup() {
  M5.begin();
  Serial.begin(115200);
  M5.Speaker.begin();
  M5.Speaker.mute();
  begin_pfs();
}

void loop() {
  struct pfs_sensors sensors;
  read_sensors(&sensors);
  print_sensors(&sensors); // For debug
}
