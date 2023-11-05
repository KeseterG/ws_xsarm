# WS XSARM

This repository is a sample control repository for Interbotix PX150 Arm.

## Bringing Up

To start the arm in simulation, use the command:

```
ros2 launch interbotix_xsarm_moveit xsarm_moveit.launch.py robot_model:=px150 hardware_type:=fake
```

Note that the last parameter determines controlling target. It can take value of "actual", "fake" and "gazebo_classic"

To start the arm planning sever, use the command:

```
ros2 launch arm_test armserver.launch.py // planning
ros2 launch arm_test armrt.launch.py // realtime servo and pose trakcing, need to have more work done
```


