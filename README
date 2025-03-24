# About

- ros-humble
- gazebo-11
- plugin to emulate thruster force on spacecraft
- one plugin "instance" for each thruster block (2D thruster)

![screenshot of gazebo simulation](image.png)

## Commands

#### Build
```colcon build --symlink-install```

#### Launch
```ros2 launch spacecraft spacecraft.launch.py```

#### Manual Thruster fire
```ros2 topic pub --once /spacecraft/thruster/right spacecraft_msgs/msg/ThrustCommand '{direction_selection: {x: 0, y: -1, z: 100}, duration: {sec: 0, nanosec: 200000000}}'```
