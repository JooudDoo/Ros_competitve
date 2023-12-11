colcon build

. install/setup.bash

ros2 launch robot_bringup autorace_2023.launch.py & 

ros2 run referee_console mission_autorace_2023_referee &

ros2 launch autorace_core_TheRosBoss autorace_core.launch.py