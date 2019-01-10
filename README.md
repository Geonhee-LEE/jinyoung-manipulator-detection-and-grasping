# Requirement

1. Ubuntu 16.04

2. ROS Kinetic

3. MoveIt!

4. Realsense ROS Package

5. OpenCV ROS Package

6. [ROS Qt Creator](https://ros-qtc-plugin.readthedocs.io/en/latest/)



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
 
 
 To be more specific, the functions of qtros node is like below:
 
 
 
<p align="center">
    <img src="./img/qtros_default.png" width="640" height="480" >
</p>
 
- ROS Master
    -conect
    
- TCP/IP
    - ServerIP: IP of EC-Master
    - Port: Port of EC-Master
 
