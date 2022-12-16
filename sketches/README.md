# Usage

Before burning the firmware, create `ros_lib` directory with `pr2_fingertip_sensors/PR2FingertipSensor.h`.

```
# Remove existing ros_lib
cd <YOUR_ARDUINO_IDE_PATH>/libraries
rm -rf ros_lib
# Create ros_lib
source ~/pr2_fingertip_ws/devel/setup.bash
rosrun rosserial_arduino make_libraries.py .
```

Burn the firmware.
- `receive_PFS.ino` is to print PFS sensor data on serial monitor.
- `publish_PFS.ino` is to publish PFS sensor data on ROS.

Inside each firmware, the communication method can be switched by commenting in/out the following define.

```
#define I2C_MASTER
// #define SOFTWARE_SERIAL
// #define HARDWARE_SERIAL
```

If you use rosserial (`publish_PFS.ino`) , run

```
rosrun rosserial_python serial_node.py _baud:=57600 _port:=<your_m5stack_device>
```

# NOTE

The directory `sketch/**/symlink_libs` is automatically created by `catkin build` and the files in the directory is ignored by git. Do not place your original header files here.
