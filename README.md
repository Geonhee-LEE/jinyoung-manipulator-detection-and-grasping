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
 
<img src="./img/srdf1.png"  class="center">
<img src="./img/srdf2.png"  class="center">
 
2.  Run the realsense cameras(camera1(D415): world camera, camera2(D435)" End-effector) 

Note: Connecttion of USB 3.0

 <pre><code>
roslaunch realsense2_camera rs_multiple_devices.launch serial_no_camera1:=732612061000 serial_no_camera2:=819312073026
 </pre></code>
 
 
<img src="./img/realsense1.png"  class="center">
 
 
3.  Run Qt GUI for controlling the manipulator

 <pre><code>
 rosrun qtros qtros
 </pre></code>
 
<img src="./img/qtros_default.png"  class="center">
 
4.  Run image process using OpenCV 

 <pre><code>
 rosrun camera_node camera_node
 </pre></code>
 
 
<img src="./img/camera_node_default.png"  class="center">
 
