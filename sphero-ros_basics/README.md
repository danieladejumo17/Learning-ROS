## Launching

Start Gazebo
```bash
roslaunch sphero_gazebo main.launch
```

Start the Gazebo Client if it's not started already
```bash
gzclient
```

Source the catkin workspace
```bash
source ~/catkin_ws/devel/setup.bash
```

Start the main program
```bash
roslaunch my_sphero_main main.launch
```

To restart the simulation, reset the gazebo world
```bash
rosservice call /gazebo/reset_world "{}"
```