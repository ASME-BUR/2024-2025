### Build Instructions
Requires [PlatformIO IDE for VSCode](https://platformio.org/install/ide?install=vscode)
1. Open PlatformIO IDE (in VSCode) and select the folder of the to-be-built package
	- Pick the folder containing the file `platformio.ini`
2. Build the package (bottom of window, checkmark icon)
3.  *While connected to target Arduino via USB*: Upload the package (bottom of window, right arrow icon)

### Contents
1. `arduino_imu`: Arduino Micro-ROS publisher code for our IMU
2. `arduino_merge`: Unused, contains Micro-ROS Arduino code for both IMU & thrusters
3. `arduino_thruster`: Arduino Micro-ROS code for running our thrusters
