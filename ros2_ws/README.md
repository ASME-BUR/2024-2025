## Building
To build all packages, run the following (from this directory):
```
colcon build
```
Alternatively, to build a specific package:
```
colcon build --packages-select <PACKAGE_NAME>
```
(*See ``colcon build -h`` for more options.*)

To remove all packages, run (from this directory):
```
rm -r build/ install/ log/
```

## Packages

1. `bur_launch`: Launch files for sensors & autonomous operation
2. `bur_msgs`: Custom ROS message files
3. `bur_rov`: Launch files & scripts for robot tele-operation
4. `gnc/`
   - `bur_controller`: PID controllers for translational/rotational movement
   - `bur_localization`: Parameters & launch file for EKF sensor fusion
   - `bur_planner`: BehaviorTree.CPP/Navigation2-based behavior tree mission manager (unused)
   - `simple_manager`: Simple implementation of a BehaviorTree.CPP behavior tree mission manager
   - `simple_planner`: Simple implementation of a straight-line motion planner
   - `thruster_manager`: Determines power for each thruster based on target force Wrench
5. `sensors/`
   - `bur_sensors`: ROS nodes for publishing pressure sensor/IMU measurements from Raspberry Pi
   - `drivers`: sensor drivers
6. `submodules/`: Contains Git submodules

