#include <M5Stack.h>

#define I2C_MASTER
// #define SOFTWARE_SERIAL
// #define HARDWARE_SERIAL
#define PFS_ADDRESSES {0x01}
// #define PFS_ADDRESSES {0x01, 0x02}
#include "symlink_libs/pfs.h"
  
void setup() {
  M5.begin();
  Serial.begin(115200);
  M5.Speaker.begin();
  M5.Speaker.mute();
  begin_pfs();
}

void loop() {
  receive_pfs();
}
