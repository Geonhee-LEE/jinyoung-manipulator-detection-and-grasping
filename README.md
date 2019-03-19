# Requirements

1. Ubuntu 16.04

2. ROS Kinetic

3. MoveIt!

4. Realsense ROS Package

5. OpenCV ROS Package

6. [Intel SDK](https://github.com/intel-ros/realsense)

## Prerequisites
**Important:** Running RealSense Depth Cameras on Linux requires patching and inserting modified kernel drivers. Some OEM/Vendors choose to lock the kernel for modifications. Unlocking this capability may requires to modify BIOS settings

  **Make Ubuntu Up-to-date:**  
  * Update Ubuntu distribution, including getting the latest stable kernel:
    * `sudo apt-get update && sudo apt-get upgrade && sudo apt-get dist-upgrade`  <br />  
**Prepare Linux Backend and the Dev. Environment:**  
  1. Navigate to *librealsense* root directory to run the following scripts.<br />
     Unplug any connected Intel RealSense camera.<br />  

  2. Install the core packages required to build *librealsense* binaries and the affected kernel modules:  
    `sudo apt-get install git libssl-dev libusb-1.0-0-dev pkg-config libgtk-3-dev`  <br /><br />
    Distribution-specific packages:  <br />
     * Ubuntu 14 or when running of Ubuntu 16.04 live-disk:<br />
      `sudo apt-get install`<br />
      `./scripts/install_glfw3.sh`  <br />

     * Ubuntu 16:<br />
      `sudo apt-get install libglfw3-dev`<br />

     * Ubuntu 18:<br />
      `sudo apt-get install libglfw3-dev libgl1-mesa-dev libglu1-mesa-dev`  <br /><br />

> **Cmake Note**: certain librealsense CMAKE flags (e.g. CUDA) require version 3.8+ which is currently not made available via apt manager for Ubuntu LTS.   
    Go to the [official CMake site](https://cmake.org/download/) to download and install the application  

     **Note** on graphic sub-system utilization:<br />
     *glfw3*, *mesa* and *gtk* packages are required if you plan to build the SDK's OpenGl-enabled examples. The *librealsense* core library and a range of demos/tools are designed for headless environment deployment.

  3. Install Intel Realsense permission scripts located in librealsense source directory:<br />
    `sudo cp config/99-realsense-libusb.rules /etc/udev/rules.d/`  <br />
    `sudo udevadm control --reload-rules && udevadm trigger`
    <br />

  4. Build and apply patched kernel modules for: <br />
    * **Ubuntu 14/16/18 with LTS kernel**
      script will download, patch and build realsense-affected kernel modules (drivers).<br />
      Then it will attempt to insert the patched module instead of the active one. If failed
      the original uvc modules will be restored.  

      `./scripts/patch-realsense-ubuntu-lts.sh`<br />
    * **Ubuntu with Kernel 4.16**

      `./scripts/patch-ubuntu-kernel-4.16.sh`<br />  
    * **Intel® Joule™ with Ubuntu**
      Based on the custom kernel provided by Canonical Ltd.  

      `./scripts/patch-realsense-ubuntu-xenial-joule.sh`<br />
    * **Arch-based distributions**
      * You need to install the [base-devel](https://www.archlinux.org/groups/x86_64/base-devel/) package group.
      * You also need to install the matching linux-headers as well (i.e.: linux-lts-headers for the linux-lts kernel).<br />
        * Navigate to the scripts folder  `cd ./scripts/`<br />
        * Then run the following script to patch the uvc module: `./patch-arch.sh`<br /><br />
    * **Odroid XU4 with Ubuntu 16.04 4.14 image**
      Based on the custom kernel provided by Hardkernel

      `./scripts/patch-realsense-ubuntu-odroid.sh`<br />
      Some additional details on the Odroid installation can also be found in [installation_odroid.md](installation_odroid.md)

> Check the patched modules installation by examining the generated log as well as inspecting the latest entries in kernel log:<br />
      `sudo dmesg | tail -n 50`<br />
    The log should indicate that a new uvcvideo driver has been registered.  
       Refer to [Troubleshooting](#Troubleshooting) in case of errors/warning reports.

  5. TM1-specific:
     * Tracking Module requires *hid_sensor_custom* kernel module to operate properly.
      Due to TM1's power-up sequence constrains, this driver is required to be loaded during boot for the HW to be properly initialized.

      In order to accomplish this add the driver's name *hid_sensor_custom* to `/etc/modules` file, eg:
      ```sh
      echo 'hid_sensor_custom' | sudo tee -a /etc/modules
      ```

## Building librealsense2 SDK
  * On Ubuntu 14.04, update your build toolchain to *gcc-5*:
    * `sudo add-apt-repository ppa:ubuntu-toolchain-r/test`
    * `sudo apt-get update`
    * `sudo apt-get install gcc-5 g++-5`
    * `sudo update-alternatives --install /usr/bin/gcc gcc /usr/bin/gcc-5 60 --slave /usr/bin/g++ g++ /usr/bin/g++-5`
    * `sudo update-alternatives --set gcc "/usr/bin/gcc-5"`

    > You can check the gcc version by typing: `gcc -v`
    > If everything went fine you should see gcc 5.0.0.


  * Navigate to *librealsense* root directory and run `mkdir build && cd build`<br />
  * Run CMake:
    * `cmake ../` - The default build is set to produce the core shared object and unit-tests binaries in Debug mode. Use `-DCMAKE_BUILD_TYPE=Release` to build with optimizations.<br />
    * `cmake ../ -DBUILD_EXAMPLES=true` - Builds *librealsense* along with the demos and tutorials<br />
    * `cmake ../ -DBUILD_EXAMPLES=true -DBUILD_GRAPHICAL_EXAMPLES=false` - For systems without OpenGL or X11 build only textual examples<br /><br />

  * Recompile and install *librealsense* binaries:<br />  
  `sudo make uninstall && make clean && make && sudo make install`<br />  
  The shared object will be installed in `/usr/local/lib`, header files in `/usr/local/include`.<br />
  The binary demos, tutorials and test files will be copied into `/usr/local/bin`<br />
  **Tip:** Use *`make -jX`* for parallel compilation, where *`X`* stands for the number of CPU cores available:<br />
  `sudo make uninstall && make clean && make **-j8** && sudo make install`<br />
  This enhancement may significantly improve the build time. The side-effect, however, is that it may cause a low-end platform to hang randomly.<br />
  **Note:** Linux build configuration is presently configured to use the V4L2 backend by default.<br />
  **Note:** If you encounter the following error during compilation `gcc: internal compiler error` it might indicate that you do not have enough memory or swap space on your machine. Try closing memory consuming applications, and if you are running inside a VM increase available RAM to at least 2 GB.


7. [ROS Qt Creator](https://ros-qtc-plugin.readthedocs.io/en/latest/)



-------------

# How to run the functions of 'manipulator'

1.  Run the moveIt! and RViz

 <pre><code>
 roslaunch srdf demo.launch 
 </pre></code>
 
 
<p align="center">
    <img src="./img/srdf1.png" width="640" height="480" >
    <img src="./img/srdf2.png"  width="640" height="480" >
</p>

 
2.  Run the realsense cameras(camera1(D415): world camera, camera2(D435)" End-effector) 

Note: Connect the camera to the USB 3.0

 <pre><code>
roslaunch realsense2_camera rs_multiple_devices.launch serial_no_camera1:=732612061000 serial_no_camera2:=819312073026
 </pre></code>
 
 
<p align="center">
    <img src="./img/realsense1.png" width="640" height="480" >
</p>
 
 
3.  Run Qt GUI for controlling the manipulator

 <pre><code>
 rosrun qtros qtros
 </pre></code>
 
 
<p align="center">
    <img src="./img/qtros_default.png" width="640" height="480" >
</p>
 
 
4.  Run image process using OpenCV 

 <pre><code>
 rosrun camera_node camera_node
 </pre></code>
 
 
<p align="center">
    <img src="./img/camera_node_default.png" width="640" height="480" >
</p>
 
 
 


-------------

# qtros GUI guide

 To be more specific, the functions of qtros node is like below:
 
 
## Control tab
 
<p align="center">
    <img src="./img/qtros_default.png" width="640" height="480" >
</p>
 
- ROS Master
    - ROS MasterUrl: ROS MasterUrl
    - ROS IP: ROS IP
    - **Connect** : To connect for ROS Master with Qt GUI
    
- TCP/IP
    - ServerIP: IP of EC-Master
    - Port: Port of EC-Master(Default)
    - **Connect** : To connect for EC Master with Qt GUI
 
-  ForwardJoint_value: Get the joint angles in RViz.

-  EndPointJoint_value: Get the 3D position such as x, y, z and angle such as roll, pitch, yaw in RViz.

-  EndToCamera: Move the manitulator to the desired position through inverse kinematics in RViz. 

## Setting tab

<p align="center">
    <img src="./img/qtros_setting.png" width="640" height="480" >
</p>


-  Set_JointValue: Set the joint angles for moving according to absolute joint angles .

-  Get_JointValue: Get the joint angles from real manipulator.

-  Robot_Server: 
    - SRV_ON: Servo motor on
    - SRV_OFF: Servo motor off
    - Position_Set: init position setting
    - Free_Set:   Direct teaching
    
-  Robot_Pose: 
    - Home: Move to the home.
    - Pose_set: Move to the designated position 
    - PixelPoint: Track the object using image processing
    - Gripper On/OFF:   Gripper On/OFF
    

 
## Check tab

It is designed for checking repeatability of manipulator. 


-------------

# camera node guide

Drag the desired object area on image window with keeping left button of mouse.

<p align="center">
    <img src="./img/camera_node_init.png" width="640" height="480" >
</p>
