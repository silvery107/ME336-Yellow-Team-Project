# ME336-Yellow-Team-Project1
Project source code for Collaborative Robot Learning
## Instruction
In the industrial field, picking target objects in 2D plane with the manipulator arm is applied in many scenarios. This 2D-picking project is  to reproduce the whole procedure of  picking objects with the manipulator arm(Franka) in 2D plane, basing on the result of visual observation(RGB images) and object detection(YOLO5).
Firstly, in this work, we need to complete the hand-eye calibration. With the eye-to-hand camera fixation, the positions of several points both in camera frame and manipulator's  workspace frame are recorded to get the perspective transform matrix. In this way, coordinate transformation can be achieved. After the completion of calibration accuracy test, YOLO5 algorithm is used to detect the position of the target object based on the visual images information collected by the realsense camera. With the help of the transformation matrix, target position of the end-effector in the arm can be obtained. With all these information, the manipulator arm can be controlled to pick the object in 2D work plane.
## Demo video
 Demo videos of 2D-picking with Franka manipulator arm can be found here. 
## Contact
- SUSTech me336 yellow team
## Installation
This 2D-Image-Picking project requires the following environments (Required System: Ubuntu 18.04)
1. Anaconda Installation and Environment Configuration
  - Use this link Anaconda to download sh. File and run to install
  ```
  cd Downloads
  ls
  bash Anaconda3-2020.11-Linux-x86_64.sh
  ```
  - Set the environment configuration for Anaconda
  - Creative a me336 conda environment (python 3.6)
  ```
  conda create -n me336 python=3.6
  ```




## Improvement
