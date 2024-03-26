

## Running MockHW
> **NOTE:** If you are not familiar with ros2_control check the docs for some [useful examples](https://control.ros.org/master/doc/ros2_control_demos/doc/index.html) and [additional information](https://control.ros.org/master/doc/getting_started/getting_started.html).

To test the mock hardware with standard controllers we first start the bringup with the mock hardware enabled:
```
ros2 launch $PKG_NAME$ $ROBOT_NAME$_control.launch.xml use_mock_hardware:=true
```
In a second terminal we then publish goals for the jtc:
```
ros2 launch $PKG_NAME$ test_joint_trajectory_controller.launch.xml
```

After stopping the goal publisher, you can deactivate `JointTrajectoryController`, and activate `ForwardPositionController`:
```
ros2 control switch_controllers --activate forward_position_controller --deactivate joint_trajectory_controller
```
After successful controller switch, start goal publisher for another controller:
```
ros2 launch $PKG_NAME$ test_forward_position_controller.launch.xml
```
