# dyn_3d_mod

Dynamic 3D modeling

- [ROS](#ros)
- [Building Workflow](#project-building-workflow)
- [Recording Sensor data](#recording-sensor-streams)


## ROS Tutorials
- General: http://wiki.ros.org/ROS/Tutorials
- Publisher/Subscriber nodes: http://wiki.ros.org/ROS/Tutorials/WritingPublisherSubscriber%28c%2B%2B%29
- Working with IDEs in ROS: http://wiki.ros.org/IDEs

## ROS commands

- ``$ roscore`` launch the ROS core
- ``$ roscd dyn_3d_mod`` switch to the package folder
- ``$ roscd ros_workspace`` switch to the workspace folder
- ``$ rosrun dyn_3d_mod {execname}`` run package
- ``$ roslaunch freenect_launch freenect.launch`` launch freenect driver

## Project building workflow

1. specify which files to compile in the CMakeLists.txt
``$ rosed beginner_tutorials CMakeLists.txt ``
> 
Example (insert at the bottom):
```
rosbuild_add_executable(hello src/hello.cpp)
```

2. Compile
``$ make``

3. Execute
``$ rosrun dyn_3d_mod hello``

## Connecting with the Kinect Sensor
1. ``$ roscore`` launch the ROS core
2. ``$ roslaunch freenect_launch freenect.launch`` launch freenect driver
3. ``$ rosrun dyn_3d_mod {execname}`` run package


## Eclipse & ROS packages
Use ``make eclipse-project`` to create the Eclipse project files (run again if manifest.xml has been changed)

