cd ~/soft_world

source /opt/ros/humble/setup.bash

colcon build --symlink-install

source install/setup.bash


////////////////launch//////////

ros2 launch soft_world soft_world.launch.py
