# WS XSARM

This repository is a sample control repository for Interbotix PX150 Arm.

## Bringing Up

Before getting started, make sure interbotix arm development kit is installed. Check out the [installation link](https://github.com/Interbotix/interbotix_ros_manipulators/blob/main/interbotix_ros_xsarms/install/amd64/xsarm_amd64_install.sh).

To start the interbotix arm in simulation, use the command:

```
ros2 launch interbotix_xsarm_moveit xsarm_moveit.launch.py robot_model:=px150 hardware_type:=fake // start interbotix arm
ros2 launch c_moveit_config demo.launch.py // start walli
```

Note that the last parameter determines controlling target for startimng up interbotix arm. It can take value of "actual" (real robot), "fake" (simulated hardware) and "gz_classic" (gazebo).

To start the arm sever, use the command:

```
ros2 launch arm_test armrt.launch.py // realtime servo and pose trakcing
ros2 launch arm_test armrt_walli.launch.py // walli realtime servo and pose tracking
```

To start keyboard control, use the command:

```
ros2 run arm_test servo_keyboard
```

Note that the parameters inside `servo_keyboard` is configed for controlling walli. Parameters (e.g. end effector link name) should be changed if you want to control other arms.

