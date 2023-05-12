

## General details about robot bringup packages

A bringup package holds config und launch files for starting different scenarios with a robot.

The general package structure is the following:

```
<robot_name>_bringup/                              # Launch and config files for starting the robot using ros2_control
├── [CMakeLists.txt]                               # if ament_cmake is used (recommended)
├── package.xml
├── [setup.py]                                     # if amend_python is used
├── [setup.cfg]                                    # if amend_python is used
├── config/
│   ├── <robot_name>_controllers.yaml              # Controllers' configuration for ros2_control
│   ├── <robot_name>_forward_position_publisher.yaml  # Setup test publisher for forward position controller
│   └── <robot_name>_joint_trajectory_publisher.yaml  # Setup test publisher for joint trajectory controller
└── launch/
    ├── <robot_name>.launch.py                     # Robot's default launch file
    ├── test_forward_position_controller.launch.py # Start test publisher for forward position controller
    └── test_joint_trajectory_controller.launch.py # Start test publisher for joint trajectory controller

```

**NOTE**: Most of the following steps are equivalent to the description in [ros2_control_demos](https://github.com/ros-controls/ros2_control_demos) repository.
Consult the repository and [ros2_control documentation](https://ros-controls.github.io/control.ros.org/) for more details.


## Testing the *mock* robot using ros2_control-framework

**ATTENTION**: if the package is not build and sourced do this first

1. Start robot's hardware and load controllers (default configuration starts mock hardware)
   ```
   ros2 launch $PKG_NAME$ $ROBOT_NAME$.launch.py
   ```

2. Open another terminal and check if your hardware is loaded properly:
   ```
   ros2 control list_hardware_interfaces
   ```
   You should see list of *command*- and *state* interfaces for your hardware.

3. Now you should load controllers as described in [loading controllers](#loading-controllers) section.

4. Check the result as described in the [result](#result) section.


## Testing the real robot using ros2_control-framework

**TBD**


## Loading Controllers

To move the robot you should load and start contorllers.
To get feedback about robot's state `JointStateController` is used.
to send command to the robot `ForwardCommandController` (direct goals) or `JointTrajectoryController` (interpolates trajectory).
The sections below describe their usage.

### Joint State Controller
Joint state Controllers provides output of robot's internal states to `/joint_states` and `/dynamic_joint_states` ROS 2-topics.

In a new terminal with sourced ROS 2 environment load, configure and start `joint_state_broadcaster`:
  ```
  ros2 control load_start_controller joint_state_broadcaster
  ```
Check if controller is loaded properly:
 ```
 ros2 control list_controllers
 ```
You should get similar response to:
 ```
 joint_state_broadcaster[joint_state_broadcaster/JointStateController] active
 ```

Now you should also see your robot represented correctly in the `rviz2`.

### Using Forward Command Controllers

1. If you want to test hardware with `ForwardCommandController` first load and configure it. Controller types are e.g., "position", "velocity", and depend on configured names in the [`config/$ROBOT_NAME$_controllers.yaml`](config/$ROBOT_NAME$_controllers.yaml):
   ```
   ros2 control load_configure_controller forward_<controller_type>_controller
   ```
   Check if controller is loaded properly:
   ```
   ros2 control list_controllers
   ```
   You should get the response:
   ```
   joint_state_broadcaster[joint_state_broadcaster/JointStateController] active
   forward_<controller_type>_controller[forward_command_controller/ForwardCommandController] inactive
   ```

2. Now start the controller:
   ```
   ros2 control switch_controllers --start-controllers forward_<controller_type>_controller
   ```
   Check if controllers are activated:
   ```
   ros2 control list_controllers
   ```
   You should get `active` in the response:
   ```
   joint_state_broadcaster[joint_state_broadcaster/JointStateController] active
   forward_<controller_type>_controller[forward_command_controller/ForwardCommandController] active
   ```

**NOTE**: You can do this in only one step by using `load_start_controller` verb instead of `load_configure_controller`.

3. Send command to the controller, either:

   a. Manually using ROS 2 cli interface:
   ```
   ros2 topic pub /forward_<controller_type>_controller/commands std_msgs/msg/Float64MultiArray "data:
   - 0.5
   - 0.5
   - 0.5
   - 0.5
   - 0.5
   - 0.5"
   ```
   b. Or you can start demo node which sends two goals every 5 seconds in a loop (**Only with position controller!**):
   ```
   ros2 launch $PKG_NAME$ test_forward_position_controller.launch.py
   ```

### Using JointTrajectoryController (**Not working yet!**)

1. If a `ForwardCommandController` is started you should stop it first by using:
   ```
   ros2 control switch_controllers --stop-controllers forward_<controller_type>_controller
   ```
   Check if controllers are activated:
   ```
   ros2 control list_controllers
   ```
   You should get `active` in the response:
   ```
   joint_state_broadcaster[joint_state_broadcaster/JointStateController] active
   forward_<controller_type>_controller[forward_command_controller/ForwardCommandController] inactive
   ```

2. If you want to test hardware with `JointTrajectoryController` first load, configure and start it:
   ```
   ros2 control load_start_controller joint_trajectory_controller
   ```
   Check if controllers are activated:
   ```
   ros2 control list_controllers
   ```
   You should get `active` in the response:
   ```
   joint_state_broadcaster[joint_state_broadcaster/JointStateController] active
   joint_trajectory_controller[joint_trajectory_controller/JointTrajectoryController] active
   ```

3. Send command to the controller using test node:
   ```
   ros2 launch ros2_control_demo_robot test_joint_trajectory_controller.launch.py
   ```

**NOTE**: You can switch contorllers (step 1 and 2) also with one command:
```
ros2 control switch_controllers --stop-controllers forward_<controller_type>_controller --start-controllers joint_trajectory_controller
```


## Result

1. If you echo the `/joint_states` or `/dynamic_joint_states` topics you should changing values when the robot is moving values.
   ```
   ros2 topic echo /joint_states
   ros2 topic echo /dynamic_joint_states
   ```

3. The robot should also move in `rviz2`.
