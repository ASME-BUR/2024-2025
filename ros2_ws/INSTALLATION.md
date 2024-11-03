# System #

- Ubuntu 22.04
- ROS2 Humble
- Gazebo 11

# Installation #

### 1. If you don’t have ROS 2 Humble installed ###
Follow the instructions below and install the `ros-humble-desktop`/`ros-dev-tools` packages: <https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html>

Additionally, follow the instructions for environment setup here: <https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Configuring-ROS2-Environment.html>


### 2. Clone this repository into your root directory: ###

   ```
   cd ~/ && git clone --recurse-submodules https://github.com/ASME-BUR/2024-2025.git
   ```

### 3. Put these commands into your ~/.bashrc: ###
Add the ROS2 source commands to your bashrc
   ```
   echo "source /opt/ros/humble/setup.bash
   source ~/2024-2025/ros2_ws/install/setup.bash" >> ~/.bashrc
   ```

- The `/opt/ros/humble/setup.bash` file sources all your ros packages installed in `/opt`, e.g. all packages installed via `sudo apt-get <PACKAGE>`   
- The `install/setup.bash` file sources all packages built via `colcon build` (in your ws); however, you will need to re-source it every time you build a package for the first time. (You can simply create a new terminal.)

### 4. Install the ROS dependency manager. ###
Run the following in a terminal:

   ```
   sudo apt install python3-rosdep
   sudo rosdep init
   rosdep update
   ```
   Browse to the root of your workspace and check for missing dependencies:

   ```
   cd ~/2024-2025/ros2_ws/
   rosdep install -i --from-path src --rosdistro humble -y
   ```

   - If rosdep is having trouble installing packages, you can also install them manually using `sudo apt install ros-humble-<PACKAGE_NAME>`

### 5. Install Colcon, the build tool system: ###
   ```
   sudo apt install python3-colcon-common-extensions
   ```

### 6. Build packages in ros2_ws: ###
   ```
   cd ~/2024-2025/ros2_ws
   colcon build
   ```
   * Some of these packages are very big so it might take a couple builds
   * If a package fails once, you  may just have to try it again; if it fails more than once, then you may still be missing dependencies
   * You can also build a specific package:
      ```
      colcon build --packages-select <PACKAGE-NAME>
      ```
      or build packages recursively
      ```
      colcon build --packages-up-to <PACKAGE-NAME>
      ```


### 7. Install Pigpio ### 
Follow the instructions below:
   <https://abyz.me.uk/rpi/pigpio/download.html>


### (Optional) Install Gazebo: ###

   ```
   sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list'
   wget https://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -
   sudo apt-get update
   sudo apt-get install gazebo11
   sudo apt-get install libgazebo11-dev
   ```
   You can write gazebo in a terminal to make sure that gazebo runs correctly. Write gazebo --version to ensure that the version number is 11.X.

### (Optional) Install the ROS packages for gazebo: ###
   ```
   sudo apt install ros-humble-gazebo-ros-pkgs
   colcon build --symlink-install
   ```
