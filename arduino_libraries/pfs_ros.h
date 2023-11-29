#include "pfs.h"

#undef ESP32
#include <ros.h>
#define ESP32

#include <pr2_fingertip_sensors/PR2FingertipSensor.h>

// ROS node handle.
ros::NodeHandle nh;
pr2_fingertip_sensors::PR2FingertipSensor pfs_msg;
#if (defined SOFTWARE_SERIAL) || (defined HARDWARE_SERIAL)
  ros::Publisher pfs_pub("/pfs/from_uart", &pfs_msg);
#endif
#if (defined I2C_MASTER)
  ros::Publisher* pfs_pubs;
  char** topic_names;
#endif


void setup_nodehandle () {
  // Advertise ROS message
  nh.initNode();
  nh.getHardware()->setBaud(57600);

  // Advertise ROS message
  int pfs_num = sizeof(pfs_addresses)/sizeof(uint8_t);
  pfs_pubs = (ros::Publisher*)malloc(pfs_num * sizeof(ros::Publisher));
  // 2D char array
  // https://stackoverflow.com/questions/2614249/dynamic-memory-for-2d-char-array
  topic_names = (char**)malloc(pfs_num * sizeof(char*));
  for(int i=0; i<pfs_num; i++) {
    char* topic_name = (char*)malloc(40 * sizeof(char));
    sprintf(topic_name, "/pfs/from_i2c/addr_0x%02x", pfs_addresses[i]);
    ros::Publisher pfs_pub(topic_name, &pfs_msg);
    pfs_pubs[i] = pfs_pub;
    nh.advertise(pfs_pubs[i]);
  }
}

void set_pfs_fields(struct pfs_sensors* sensors,
                    pr2_fingertip_sensors::PR2FingertipSensor* pfs_msg) {
  // Proximity
  pfs_msg->proximity_length = NUM_SENSORS;
  pfs_msg->proximity = sensors->proximities;
  // Force
  pfs_msg->force_length = NUM_SENSORS;
  pfs_msg->force = sensors->forces;
  // IMU
  pfs_msg->imu.angular_velocity.x = sensors->gyro[0];
  pfs_msg->imu.angular_velocity.y = sensors->gyro[1];
  pfs_msg->imu.angular_velocity.z = sensors->gyro[2];
  pfs_msg->imu.linear_acceleration.x = sensors->acc[0];
  pfs_msg->imu.linear_acceleration.y = sensors->acc[1];
  pfs_msg->imu.linear_acceleration.z = sensors->acc[2];
  return;
}

void get_pfs_msg (pr2_fingertip_sensors::PR2FingertipSensor* pfs_msg, uint8_t pfs_address = 0x01) {
  // Get sensor data
  // If you use HARD/SOFTWARE Serial, stop them before publishing rostopic
  #if (defined SOFTWARE_SERIAL) || (defined HARDWARE_SERIAL)
    begin_pfs();
  #endif
  struct pfs_sensors sensors;
  read_sensors(&sensors, pfs_address);
  #if (defined SOFTWARE_SERIAL) || (defined HARDWARE_SERIAL)
    end_pfs_serial();
  #endif

  set_pfs_fields(&sensors, pfs_msg);
  pfs_msg->header.stamp = nh.now();
  char frame_id[25];
  sprintf(frame_id, "pfs_link_addr_0x%02x", pfs_address);
  pfs_msg->header.frame_id = frame_id;
  char imu_frame_id[40];
  sprintf(imu_frame_id, "%s_pfs_a_front", frame_id);
  pfs_msg->imu.header.frame_id = imu_frame_id;
}

void publish_pfs () {
  #if (defined SOFTWARE_SERIAL) || (defined HARDWARE_SERIAL)
    // Make sure that other Serial is not used during rosserial communication
    get_pfs_msg(&pfs_msg);
    pfs_pub.publish(&pfs_msg);
  #endif
  #if (defined I2C_MASTER)
    for(int i=0; i<sizeof(pfs_addresses)/sizeof(uint8_t); i++) {
      get_pfs_msg(&pfs_msg, pfs_addresses[i]);
      pfs_pubs[i].publish(&pfs_msg);
    }
  #endif
  nh.spinOnce();
}
