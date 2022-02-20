#!/bin/bash
# ros2 run turtlesim turtlesim_node &
colcon build --packages-select turtle_sim_searcher
. install/setup.sh
ros2 run turtle_sim_searcher grid_searcher
