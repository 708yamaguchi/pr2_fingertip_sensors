// https://qiita.com/ntrlmt/items/b7794f5f0705e1971b72

#include <M5Stack.h>

#define I2C_MASTER
// #define SOFTWARE_SERIAL
// #define HARDWARE_SERIAL
#include "symlink_libs/pfs_ros.h"

void setup() {
  // Setup M5Stack
  M5.begin();
  setup_nodehandle();
  begin_pfs();
}

void loop() {
  // Read sensor data and set ROS msg
  get_pfs_msg(&pfs_msg);

  // Publish ROS message
  // Make sure that other Serial is not used during rosserial communication
  pfs_pub.publish(&pfs_msg);
  nh.spinOnce();
}
