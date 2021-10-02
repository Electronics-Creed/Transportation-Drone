# Transportation-Drone

In this project we deal with the design and control of a drone that can transport packages to different locations. The drone consists of 4 propellers and a gripper at the bottom which can be used to pick and place objects. It consists of a camera, four laser sensors that are used to detect any surrounding obstacles, and GPS and IMU sensors to get the position and orientation estimate of the drone. We built a cascaded control system which uses two PID controllers to control the roll, pitch, yaw and position of the drone. We used motor mixing algorithms to convert these control signals into signals that power the propellers. To avoid obstacles we had used bug0 algorithms. We used machine learning algorithms to generate haarcascade files to detect markers so that the drone could be landed on the marker. 

---

### Table of Contents

- [Installation](#installation)
- [Technologies](#technologies)
- [How To Use](#how-to-use)
- [Other links](#other-links)
- [References](#references)
- [Author Info](#author-info)

---

## Installation

- Use these commands to install ROS Melodic on Ubuntu 18.04: 
    > sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'  
    > sudo apt install curl  
    > curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -  
    > sudo apt update  
    > sudo apt install ros-melodic-desktop-full

- Install git
    > apt-get install git

- Creating catkin workspace: 
    > source /opt/ros/melodic/setup.bash  
    > mkdir -p ~/catkin_ws/src  
    > cd ~/catkin_ws/  
    > catkin build  
    > source devel/setup.bash  
 
 - Cloning the github repository
    > cd ~/catkin_ws/src/
    > git clone https://github.com/Electronics-Creed/Transportation-Drone.git
 
---

## Technologies

- Arduino
- MATLAB
- Simulink

---

## How To Use

#### Simulation
Run the simulaiton.mlx file section by section in MATLAB to visualize the motion of the robotic arm which is created using the robotics toolbox. In the first and second section we create the robot model and initialize necessary parameters. In the third section we move the robot arm to a given point and in the fourth section we move the robotic arm through given set of waypoints. In the subsequent section some funtions have been defined.

#### Hardware
The files used for hardware implementation are hardware_implementation1.slx and hardware_implementation2.slx. The file hardware_implementation1.slx is used to move the robot arm to a given point and activate the gripper. The second file is used to move the robotic arm through given set of waypoints and activate the gripper at the required points. Refer the circuit diagram for the connections to Arduino.

---

## Other links

Drive link: 
 > https://drive.google.com/drive/folders/1rfMxBnnn4IuiM7Yzvbfr_llIJidcPY3G?usp=sharing

Youtube playlist
 > https://www.youtube.com/playlist?list=PLSWRiv_s7diOQ1Fk6TX-yL9LCNg870Yoa

---

## References

Robotics toolbox, MATLAB robotics toolbox by Peter Corke
 > https://www.petercorke.com/RTB/r9/html/SerialLink.html

Simulink Support Package for Arduino Hardware User’s Guide, Mathworks
 > https://in.mathworks.com/help/pdf_doc/supportpkg/arduino/arduino_ug.pdf

3 Axis Robotic Arm , Abhivyakti Sharma, Kshitija Kanase, Vipul Pandey, C. K. Bhange
 > https://www.ijresm.com/Vol.2_2019/Vol2_Iss6_June19/IJRESM_V2_I6_25.pdf

---

## Author Info

- H P Jeevan https://www.linkedin.com/in/h-p-jeevan-08607a1a8
- G Rohith https://www.linkedin.com/in/g-rohith-17921a1b8
- Emyl Varghese George https://www.linkedin.com/in/emyl-varghese-george-4aa53220b
