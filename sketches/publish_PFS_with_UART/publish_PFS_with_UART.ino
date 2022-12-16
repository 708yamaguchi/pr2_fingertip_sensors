// https://qiita.com/ntrlmt/items/b7794f5f0705e1971b72

#include <M5Stack.h>
#undef ESP32
#include <ros.h>
#define ESP32

#define HARDWARE_SERIAL
#include "symlink_libs/pfs_ros.h"

// ROS node handle.
ros::NodeHandle nh;
pr2_fingertip_sensors::PR2FingertipSensor pfs_msg;
ros::Publisher pfs_pub("/pfs/from_uart", &pfs_msg);

void setup() {
  // Setup M5Stack
  M5.begin();

  // Advertise ROS message
  nh.initNode();
  nh.getHardware()->setBaud(57600);
  nh.advertise(pfs_pub);
}

void loop() {
  // Read sensor data.
  begin_pfs_serial();
  struct pfs_sensors sensors;
  read_sensors(&sensors);
  end_pfs_serial();

  // Publish PFS ROS topic
  // Do not use other Serial during rosserial communication
  set_pfs_msg(&sensors, &pfs_msg);
  pfs_msg.header.stamp = nh.now();
  pfs_msg.header.frame_id = "pfs_link";
  // Publish
  pfs_pub.publish(&pfs_msg);
  nh.spinOnce();
}
