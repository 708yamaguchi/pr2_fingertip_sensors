#include <M5Stack.h>
// Define serial type before including pfs.h
#define SOFTWARE_SERIAL
#include "symlink_libs/pfs.h"

void setup() {
  M5.begin();
  Serial.begin(115200);
  begin_pfs_serial();
}

void loop() {
  struct pfs_sensors sensors;
  read_sensors(&sensors);
  print_sensors(&sensors); // For debug
}
