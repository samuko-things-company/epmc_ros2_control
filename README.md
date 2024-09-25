## Easy PID Motor Controller (EPMC) Ros2 Control
This the **ROS2** Package for the using the **`Easy PID Motor Controller (EPMC) Module`** (i.e **`L298N EPMC Module`** or any **`Custom EPMC Interface Board`**) with **ROS2** in a PC or microcomputer, after successful setup with the [epmc_setup_application](https://github.com/samuko-things-company/epmc_setup_application).

> [!NOTE]  
> It should be used with your ros2 project running on linux `Ubuntu 22.04` [`ros-humble`] (e.g Raspberry Pi, PC, etc.) 
> It also shows how to set up `ros2-control`, `ros2-controllers`, and the `controller-manager` to work with the **`Easy PID Motor Controller (EPMC) Module`** (i.e **`L298N EPMC Module`** or any **`Custom EPMC Interface Board`**).  
> You can copy and use the codes in your projects as you see fit.

#

## How to Use the Package

#### Prequisite
- ensure you've already set up your microcomputer or PC system with [`ros-humble`](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html) with [`colcon`](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Colcon-Tutorial.html) and your `ros workspace` also setup

- install the `libserial-dev` package on your linux machine
  ```shell
  sudo apt-get update
  sudo apt install libserial-dev
  ```

- install `rosdep` so you can install necessary ros related dependencies for the package.
  ```shell
  sudo apt-get update
  sudo apt install python3-rosdep2
  sudo rosdep init
  rosdep update
  ```

#

#### How to build the epmc_hardware_interface plugin package 
- In the `src/` folder of your `ros workspace`, clone the repo
  (or you can download and add it manually to the `src/` folder)
  ```shell
  git clone https://github.com/samuko-things-company/epmc_ros2_control.git
  ```
> [!NOTE] 
> if you download it, extract it and eusre to change the folder name to **`epmc_ros2_control`** before moving it to the `src/` folder

- from the `src/` folder, cd into the root directory of your `ros workspace` and run rosdep to install all necessary ros dependencies
  ```shell
  cd ../
  rosdep install --from-paths src --ignore-src -r -y
  ```
- build the `epmc_hardware_interface` package with colcon (in the root folder of your ros workspace):
  ```shell
  colcon build --packages-select epmc_hardware_interface
  ```
> [!NOTE]   
> The **epmc_hardware_interface** package will now be available for use in any project in your ros workspace.
> You can see example of how the use the `epmc_hardware_interface` with `<ros2_control>` in a `URDF` file from the description package - `epmc_demo_bot_description`

#

#### How to test the epmc_hardware_interface with the demo packges
- ensure the **`Easy PID Motor Controller (EPMC) Module`** (i.e **`L298N EPMC Module`** or any **`Custom EPMC Interface Board`**), with the motors connected and fully set up for velocity PID, is connected to the microcomputer or PC via USB.

- check the serial port the driver is connected to:
  > The best way to select the right serial port (if you are using multiple serial device) is to select by path
  ```shell
  ls /dev/serial/by-path
  ```
  > you should see a value (if the driver is connected and seen by the computer), your serial port would be -> /dev/serial/by-path/[value]. for more info visit this tutorial from [ArticulatedRobotics](https://www.youtube.com/watch?v=eJZXRncGaGM&list=PLunhqkrRNRhYAffV8JDiFOatQXuU-NnxT&index=8)

  - OR you can also try this:
  ```shell
  ls /dev/ttyU*
  ```
  > you should see /dev/ttyUSB0 or /dev/ttyUSB1 and so on

- once you have gotten the **port**, update the **port** parameter in the **`<ros2_control>`** tag in the URDF's **`ros2_control.xacro`** file in the **`epmc_demo_bot_description`** package, with the discovered port in the previous step

- build the packages with colcon (in your `ros workspace` root folder):
  ```shell
  colcon build --packages-select epmc_demo_bot_description epmc_demo_bot_bringup --symlink-install
  ```
- lauch it following any of the two steps below:

#### STYLE 1
- open a new terminal and ensure you source your `ros workspace` in the terminal

- start the base_control and rviz viewing
  ```shell
  ros2 launch epmc_demo_bot_bringup bringup.launch.py
  ```

#### STYLE 2
> [!NOTE]   
> This style is useful when runnig the base_control and the rviz viewing seperately.  
> For example running the base_control on a Rapberry PI and running the rviz viewing in a dev PC

- open a new terminal and ensure you source your `ros workspace` in the terminal
- start the base_control
  ```shell
  ros2 launch epmc_demo_bot_bringup base_control.launch.py
  ```

- open another terminal and ensure you source your `ros workspace` in the terminal
- start the rviz viewing
  ```shell
  ros2 launch epmc_demo_bot_bringup rviz.launch.py
  ```
#

#### DRIVING / CONTROL
- you can now control the motors via a teleop package and notice how the robot changes position in rviz
> you can also use my [arrow_key_teleop_package](https://github.com/samuko-things/arrow_key_teleop_drive)

> [!NOTE]
> **feel free to use, copy and edit the `epmc_ros2_control` package codes and launch files in your preferred project**
