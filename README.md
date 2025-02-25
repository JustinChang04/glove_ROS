# Instructions

Before any actions, run
```bash
source /opt/ros/humble/setup.bash
source ~/glove_ROS/install/setup.bash
```

To build the script, cd into the glove_ROS workspace and run
```bash
colcon build
```

To run leap_v1_ik.py, from the workspace folder, cd into src/launch, and run
```bash
ros2 launch leap_v1_ik.py
```
