# ROS-Simulation-of-KUKA-LBR-iiwa-manipulator

## ENPM662 Final Project (Cyberknife: A non invasive radiosurgical application of robotics)

|Team Members
|--
|Bharadwaj Chukkala
|Joseph Pranadeer Reddy Katakam
|Bhargav Kumar Soothram


## Contents
1. Part Files and Assembly
2. Package

## Dependencies
- **python 3.9** [works for any python 3 version]
- **IDE** [I used PyCharm IDE to program the Code and Execute the Code]
- **Libraries**: numpy, matplotlib.pyplot, heapq, time

## How to run the code
--> Create a catkin_ws and build it, then source it

--> Download the package

--> Paste the package in the source directory

--> Build and source the workspace

--> Commands to run (open separate terminals and run in the same order)
```bash
  roslaunch Assembly_Toby template_launch.launch
```
```bash
  rosrun Assembly_Toby publisher.py
```
```bash
  rosrun Assembly_Toby subscriber.py
```

### Check out the final video here 👇
https://drive.google.com/file/d/1k4bVFm9eszw6IDoCDINYM6II19aMtiYY/view?usp=sharing

### Contact Author

Name : __Bharadwaj Chukkala__ <br>
Email : bchukkal@terpmail.umd.edu <br>

[![forthebadge](https://forthebadge.com/images/badges/made-with-python.svg)](https://forthebadge.com)





