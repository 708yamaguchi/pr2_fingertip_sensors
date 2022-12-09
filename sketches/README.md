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

Burn the firmware

Run

```
rosrun rosserial_python serial_node.py _baud:=57600 _port:=<your_m5stack_device>
```

# NOTE

The directory `sketch/**/symlink_libs` is automatically created by `catkin build` and the files in the directory is ignored by git. Do not place your original header files here.
