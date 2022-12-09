#include "pfs.h"
#include <pr2_fingertip_sensors/PR2FingertipSensor.h>

void set_pfs_msg(struct pfs_sensors* sensors,
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
