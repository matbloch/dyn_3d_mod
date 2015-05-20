# Dynamic 3D modeling

Lorem ipsum package description.

## Recording setup
The recording software is configured for two Kinect for XBox devices.
To avoid driver problems, they should both be connected over USB 2.0.

## Software requirements
The software has been tested with the following system configuration:
- Ubuntu 14.04.2 LTS 64bit
- ROS Indigo
- OpenCV

## Installation

1. **Install the Robot Operating System ROS (Indigo)**: [installation link](http://wiki.ros.org/indigo/Installation/Ubuntu)
    setup sources.list:
    ```
    sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
    ```
    Set up keys
    ```
    wget https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -O - | sudo apt-key add -
    ```
    **Installation**
    ```
    sudo apt-get update
    sudo apt-get install ros-indigo-desktop-full
    sudo rosdep init
    rosdep update
    ```
    Setup environment variables
    ```
    echo "source /opt/ros/indigo/setup.bash" >> ~/.bashrc
    source ~/.bashrc
    ```
    Create a folder called `ros_workspace` in Home and add this line to your ~/.bashrc directly under the call to /opt/ros/distro/setup.bash:
    ```
    export ROS_PACKAGE_PATH=~/ros_workspace:${ROS_PACKAGE_PATH}
    ```
    Restart your machine and test if the path has been added
    ```
    printenv ROS_PACKAGE_PATH
    ```

2. **Install the Kinect drivers and ROS packages**   
    ```
    sudo apt-get install ros-indigo-freenect-stack 
    ```
3. **OpenCV non-free**   
    ```
    sudo add-apt-repository --yes ppa:xqms/opencv-nonfree
    sudo apt-get update 
    sudo apt-get install libopencv-nonfree-dev
    ```
4. **Install visualization libraries**   
    ```
    LIB IGL stuff
    ```
5. **Install the Dynamic 3D Modeling software**   
	Clone/copy the repository to home/ros_workspace and `$ cd ros_workspace` and build it using `$ make` from the package directory


## Running the software

### Driver configuration
The OpenKinect drivers have been modified in order to support multiple Kinects. The custom launch files can be found in package under dyn_3d_mod/driver/.
To avoid registration problems, the **device ids** have been added to the launch files.

- **Using the custom launch files:**
You need to replace the hardeware ids of your Kinect devices in the code of the launch files cam1.launch and cam2.launch where it says:
`<arg name="device_id" value="%replace with your ID%" />`

- **Custom camera intrinsics support:** for calibration see [this package](http://wiki.ros.org/camera_calibration)
	If you would like to use custom intrinsics rather than the ones provided by the OpenKinect driver, you additionally need to edit the paths to the rgb and the depth configuration file in cam1.launch and cam2.launch.
	```
	<arg name="rgb_camera_info_url" value="file:///$(find dyn_3d_mod)/config/rgb_A00365805019051A.yaml" />
	<arg name="depth_camera_info_url" value="file:///$(find dyn_3d_mod)/config/depth_A00365805019051A.yaml" />
	```
	The file names should follow the convention in this example, where **A00365805019051A** equals to the devide id. The configuration files can be placed in the package path under dyn_3d_mod/config. Sample files are provided with this package.

### Camera calibration
Before you start recording a scene, the relative positioning of the cameras has to be calculated, following the description bellow:
0. Make sure, the cameras are connected: ``$ lsusb``
1. Start the ROS core: ``$ roscore``
	ROS core always has to be running when interfacing with the cameras
2. Open a new terminal and launch the camera drivers:
	```
	$ roslaunch dyn_3d_mod cam1.launch
	$ roslaunch dyn_3d_mod cam2.launch
	```
3. Open a new terminal and start the calibration:
	```
	$ roscd dyn_3d_mod
	$ rosrun dyn_3d_mod start_calibration
	```
5. **Optional**: Use cutoff filtering
If the configuration fails due to noise or a very small overlapping window, you can apply a additional cutoff filter to get a better initial guess of the transformation. Use the tuning tool to estimate the filter parameters which you can add to the config file under **config/config.ini**

    ```
    $ rosrun dyn_3d_mod cutoff_tuning
    ```


### Recording
0. Make sure, the cameras are connected: ``$ lsusb``
1. Start the ROS core: ``$ roscore``
> ROS core always has to be running when interfacing with the cameras   
2. Open a new terminal and launch the camera drivers:
```
$ roslaunch dyn_3d_mod cam1.launch
$ roslaunch dyn_3d_mod cam2.launch
```
3. Open a new terminal and start the recording software:
```
$ roscd dyn_3d_mod
$ rosrun dyn_3d_mod start_recording
```


## Sample data
If you don't have any Kinect devices and want to test the software, you can use a prerecorded scene.
Instead of launching the camera drivers, you only have to use the rosbag package.

### Emulating the configuration scene
To emulate the camera sensor streams for the extrinsics calibration, open a new terminal, head to the sample_scene_configuration.bag file and run:
```
$ rosbag play -l sample_scene_configuration.bag
```
This will run the configuration scene in a loop.

### Emulating the recording scene
Once the extrinsics calibration is done, the camera sensor streams for the extrinsics calibration can be simulated: open a new terminal, head to the config_scene.bag file and run:
```
$ rosbag play -l sample_scene.bag
```
This will run the configuration scene in a loop.


