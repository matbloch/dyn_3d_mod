# Dynamic 3D modeling

This software package provides a pipeline to record and visualize dynamic scenes in 3D using multiple RGB-D devices.
An implicite surface representation is used to encode the surface in a voxel representation. For efficient storage, the voxel grids are reduced to octrees and integrated into a time-space tree. This allows to only save the changing parts of the recorded scene.   
To visualize the recordings, a marching cubes implementation from [LibIGL](https://github.com/libigl/libigl) is used.

**Table of contents:**

- [Recording setup](#recording-setup)
- [Installation](#installation)
- [Running the software](#running-the-software)
	- [Driver Configuration](#driver-configuration)
	- [Camera setup calibration](#camera-setup-calibration)
	- [Recording](#recording)
- [Sample Data](#sample-data)
- [File Index](#file-index)

## Recording setup
The recording software is configured for two Kinect for XBox devices.
To avoid driver problems, they should both be connected over USB 2.0.

## Installation
**Software requirements:**
The software has been tested with the following system configuration:
- Ubuntu 14.04.2 LTS 64bit
- ROS Indigo
- OpenCV2 (partially installed with ROS)
- PCL (installed with ROS)
- LibIGL 1.1.5

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
    - Download libraries:

    ```
    cd dyn_3d_mod
    git clone https://github.com/olkido/libigl smgpclass libigl
    ```
    Download Eigen and place it in dyn_3d_mod directory like dyn_3d_mod/Eigen

    - Install required libraries:
    ```
    sudo apt-get install g++
    sudo apt-get install make
    sudo apt-get install freeglut3-dev
    sudo apt-get install libxmu-dev libxi-dev
    ```
    Install GLFW from http://www.glfw.org/download.html
    Install xorg-dev

    - Compile libigl:
    ```
    cd dyn_3d_mod/libigl/tutorial
    sh compile_dependencies_linux.sh
    mkdir build
    cd build
    cmake -DCMAKE_BUILD_TYPE=Release ../
    ```
    If you got no error on this procedure and can run some tutorial codes, installation is successfully done

5. **Install the Dynamic 3D Modeling software**   
	Clone/copy the repository to home/ros_workspace and `$ cd ros_workspace` and build it using `$ make` from the package directory


## Running the software

### Driver configuration
The OpenKinect drivers have been modified in order to support multiple Kinects. The custom launch files can be found in package under dyn_3d_mod/driver/.
To avoid registration problems, the **device ids** have been added to the launch files.

- **Using the custom launch files:**
You need to replace the hardeware ids of your Kinect devices in the code of the launch files cam1.launch and cam2.launch where it says:    
`<arg name="device_id" value="%replace with your ID%" />`

- **Camera intrinsics (optional):** for calibration see [this package](http://wiki.ros.org/camera_calibration)
To test the recording software, you do not necessarily need to configure the intrinsics yourself. Default parameters are already included in the driver launch files and the config file that comes with this package.
	1. **Driver intrinsics:** If you would like to use custom intrinsics rather than the ones provided by the OpenKinect driver, you additionally need to edit the paths to the rgb and the depth configuration file in cam1.launch and cam2.launch.
		```
		<arg name="rgb_camera_info_url" value="file:///$(find dyn_3d_mod)/config/rgb_A00365805019051A.yaml" />
		<arg name="depth_camera_info_url" value="file:///$(find dyn_3d_mod)/config/depth_A00365805019051A.yaml" />
		```
	The file names should follow the convention in this example, where **A00365805019051A** equals to the devide id. The configuration files can be placed in the package path under dyn_3d_mod/config. Sample files are provided with this package.
	
    2. **Intrinsics for voxel mapping:** The same intrinsics as used to launch the Kinect driver should be added to the configuration file dyn_3d_mod/config/config.ini under the entry "intrinsics". You can use `,` to separate matrix elements and `;` to begin a new row. Visit [Wikipedia](http://en.wikipedia.org/wiki/Camera_resectioning) to get more information about how the intrisics matrix is structure.

### Camera setup calibration

1. **Make sure, the cameras are connected:**   
    ```
	$ lsusb
    ```
2. **Open a new terminal and launch the camera drivers:**
    ```
	$ roslaunch dyn_3d_mod cam1.launch
	$ roslaunch dyn_3d_mod cam2.launch
    ```
3. **Open a new terminal and start the calibration and follow the instructions:**   
    ```
	$ roscd dyn_3d_mod
	$ rosrun dyn_3d_mod camera_registration
    ```

4. **Optional**: Use cutoff filtering
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
3. Open a new terminal and start the recording software and follow the instructions:
```
$ roscd dyn_3d_mod
$ rosrun dyn_3d_mod recording_node
```


## Sample data
If you don't have any Kinect devices and want to test the software, you can use a prerecorded scene.
Instead of launching the camera drivers, you only have to use the rosbag package and launch the sample scenes:

- **Camera configuration scene (42MB)**: [Download](https://mega.co.nz/#!uZ4gDSKL!E-3RokzOyVM7V2UwEj-qPZTofZOO4BDeHADDxA24fnE)
- **Recording scene (528MB)**: [Download](https://mega.co.nz/#!iVAkwaSY!JX-_gz-N3IHFBU0yj0Ba9TrBkoLt6UJV76uRIelKIPE)

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

## File index

### Main programs
`src/camera_registration.cpp` - Initializes the camera registration cycle
`src/cutoff_tuning.cpp` - Allows to tune a 3D cutoff filter to increase the camera registration accuracy (see config/config.ini for the available parameters)
`src/recording_node.cpp` - Allows to record and visualize a dynamic scene in 3D
`src/snapshot.cpp` - Allows to take IR/Color images e.g. for the camera calibration

### Utilities
`lib/config/config_handler.h` - A small to configuration library to read, write and parse settings
`lib/cv/voxel_grid.h` - Utilities to calculate a Voxel grid, filled with a signed distance function from a depth image
`lib/pcl/pc_aligner.h` - Registers two point clouds and calculates the relative transformation
`lib/tree/TStree.hh` - Time-space tree structure library
