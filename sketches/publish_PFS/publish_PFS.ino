// https://qiita.com/ntrlmt/items/b7794f5f0705e1971b72

#include <M5Stack.h>

#define I2C_MASTER
// #define SOFTWARE_SERIAL
// #define HARDWARE_SERIAL
#define PFS_ADDRESSES {0x02}
#include "symlink_libs/pfs_ros.h"

void setup() {
  // Setup M5Stack
  M5.begin();
  M5.Speaker.begin();
  M5.Speaker.mute();
  setup_nodehandle();
  begin_pfs();
}

void loop() {
  publish_pfs();
}
