# An Autonomous Object Searching UAV system
<div align="justify">
This project serves as my final year project. We have developed two modules: for both static object searching and dynamic object tracking. In particular, this repo is named after the EDP-Planner of the static object searching module. For tracking, please refer to this <a href="https://github.com/pattylo/AUTO">repo</a>. The project has developed a object searching system that adopts a image-process-technique-based path planning. </div>
(EDP-planner) method as well as a learning based object searching module.<br/>
<br/>
Some conducted experiment could be reviewed @:<br/>
Edge Detector Planner in Gazebo: https://youtu.be/7BZmHzc9BWQ<br/>
Object Searching flight test in VICON: https://youtu.be/ZH__uiO9Vo8<br/>
Object Searching flight test in Gazebo: https://youtu.be/AzSfZh_GZ4A<br/>

<br/>
<div align="justify">
  The ever-burgeoning growth of autonomous unmanned aerial vehicles (UAVs) has 
demonstrated a promising platform for utilization in real-world applications. Autonomous 
UAVs can be of invaluable assistance in both search & rescue (SAR) and reconnaissance 
missions if they are able to search and track target objects. Hence, this paper proposes a 
learning-based navigation system for quadrotor UAV that addresses 2 missions: to 
autonomously accomplish an object-searching task and an object-tracking task under an 
unfamiliar environment. 
In specifics, we adopt deep-learning based YOLOv4-Tiny algorithm for the purpose of 
semantic object detection. We further consolidate the algorithm with a learning-based 3D 
object pose estimation method as well as a collision-avoidance path-planning solution in order 
to search and locate static target objects. And for object tracking, we fuse the object detector 
YOLOv4-Tiny with Kalman Filter to track the dynamic target object and design the back-end 
UAV actions to follow the target object. 
We validate the presented system in both simulated situations and real-world environments. 
Experimental data are collected and analysed through the ‘Gazebo’ simulator program and 
several flight tests under the VICON arena. 
The findings of the study demonstrate the effectiveness and reliability of such autonomous 
navigation system: in which the object searching system is capable to precisely search and 
locate target in an unknown environment with a collision-free flight. Meanwhile, the object 
tracking system is able to consistently track and follow the movement of the target object. </br>
Keywords: UAV; autonomous object-searching; autonomous object-tracking 
</div>
