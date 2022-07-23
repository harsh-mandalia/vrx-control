# vrx-control
Control implementation on Virtual RobotX (VRX) simulator as part of NTU-India connect Research internship

## Build
This python script can be used with [VRX simulator](https://github.com/osrf/vrx). Follow their [tutoral](https://github.com/osrf/vrx/wiki/tutorials) page to install the VRX simulator environment.
<br/>
After installation, add the [controller.py](https://github.com/harsh-mandalia/vrx-control/blob/main/controller.py) file to ~/vrx_ws/src/vrx/vrx_gazebo/nodes.
<br/>
Now, go to ~/vrx_ws/src/vrx/vrx_gazebo/CMakeLists.txt and add "nodes/controller.py" to the catkin_install_python function.

Make the python script executable:
```
$ chmod +x ~/vrx_ws/src/vrx/vrx_gazebo/nodes/controller.py
``` 
Use `catkin_make` to build:
```
$ cd ~/vrx_ws
$ catkin_make
```

## Run
Launch the VRX simulation with a simple world:
```
$ roslaunch vrx_gazebo vrx.launch
```
Open a new terminal and start the python node:
```
$ rosrun vrx_gazebo controller.py
```
