# How to Run
save folder in catkin_ws
```
$ catkin_make
$ roslaunch turtle_mover turtle_mover.launch
```

## To run the python node
```
$ roscore
$ rosrun turtlesim turtlesim_node
$ rosrun turtle_mover turtle_mover.py # in another terminal tab
```
### OR
Edit the `turtle_mover.launch` file by
- uncomment the `python node` on line `6` and comment the `C++ node`