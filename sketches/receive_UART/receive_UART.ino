#include <M5Stack.h>
#include "pfs.h"

// SoftwareSerial to use UART via Grove connector
SoftwareSerial GroveA(22, 21);

void setup() {
  M5.begin();
  Serial.begin(115200);
  GroveA.begin(57600);
}

void loop() {
  struct pfs_sensors sensors;
  read_sensors(&GroveA, &sensors);
  print_sensors(&sensors); // For debug
}
