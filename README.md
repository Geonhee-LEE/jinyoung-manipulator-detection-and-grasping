# Requirement



-------------

# How to run the functions of 'manipulator'

1.  Run the moveIt! and RViz
 <pre><code>
 roslaunch srdf demo.launch 
 </pre></code>
 
 
2.  Run the realsense cameras(camera1(D415): world camera, camera2(D435)" End-effector) 
Note: Connecttion of USB 3.0

 <pre><code>
roslaunch realsense2_camera rs_multiple_devices.launch serial_no_camera1:=732612061000 serial_no_camera2:=819312073026
 </pre></code>
 
 
3.  Run Qt GUI for controlling the manipulator
 <pre><code>
 rosrun qtros qtros
 </pre></code>
 
 
4.  Run image process using OpenCV 
 <pre><code>
 rosrun camera_node cameranode
 </pre></code>
 
