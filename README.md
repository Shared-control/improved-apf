# improved-apf

### Introduction
Shared-control teleoperation framework based on Artificial Potential Fields improved by the generation of escape points around the obstacles.

Framework is implemented in ROS and works both with Kinetic and Melodic versions. For the hardware we suggest a configuration with at least processor i7, 8GB memory and graphics cards.

### Required Python module
- Numpy
- Sympy
- IPython
- PyUserInput

### Required ROS pakages
- Simulation Environment setup: https://github.com/Robin4-0/robin_arena
- ps3joy: http://wiki.ros.org/ps3joy
- Rosmyo: https://github.com/ste92uo/ROS_multipleMyo

### Commands
1. Start Gazebo simulation and kinect
```sh
$ roslaunch robin_arena robin_bringup.launch simulation:=BOOL_SIM gripper_enable:=BOOL_GRIPPER
```
where *BOOL_SIM == true* if the simulated environment is launched; otherwise, *BOOL_SIM == false*. 
If *BOOL_GRIPPER == true*, the command launches UR5 with a Robotiq 3-finger gripper attached on its end-effector; otherwise, *BOOL_GRIPPER == false* launches UR5 with a magnet.

2. Start Apriltag detection
```sh
$ roslaunch robin_arena apriltag.launch simulation:= BOOL
```
where *BOOL* is defined as before.

3. Start type of user control
    * Keyboard
    ```sh
    $ roslaunch keyboard keyboard_control.launch
    ```
    * Joystick
     ```sh
    $ roslaunch joystick joystick_control.launch
    ```
    * Myo
    ```sh
    $ roslaunch myo myo_control.launch
    ```
4. Start system
```sh
$ roslaunch shared_control start.launch sim:=BOOL_SIM gripper:=BOOL_GRIPPER grasp:=BOOL_GRASP teleop:=BOOL_TELEOP escape:=BOOL_ESCAPE dynamic:=BOOL_DYNAMIC user_type:="USER_TYPE"
```
   * where *BOOL_SIM == true* if the simulated environment is launched; otherwise *BOOL_SIM == false*. 
   * where *BOOL_GRIPPER == true* if gripper is attached on the end-effector; otherwise *BOOL_GRIPPER == false*.
   * where *BOOL_GRASP == true* to activate grasp routine; otherwise *BOOL_GRASP == false*.
   * where *BOOL_TELEOP == true* to activate teleoperation system, otherwise *BOOL_TELEOP == false* to activate shared-control system.
   * where *BOOL_ESCAPE == true* to activate our framework with improved APF with escape points; otherwise *BOOL_ESCAPE == false* to activate only APF shared-control system.
   * where *BOOL_DYNAMIC == true* to activate dynamic version of our framework; otherwise *BOOL_DYNAMIC == false* to activate static version.
   * where *USER_TYPE == "keyboard"* if the keyboard control is launched; *USER_TYPE == "joystick"* if the joystick control is launched; *USER_TYPE == "myo"* if myo control is launched
