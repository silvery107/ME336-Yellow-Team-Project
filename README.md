# ME336-Yellow-Team-Project1
Project source code for Collaborative Robot Learning
## Instruction
In the industrial field, picking target objects in 2D plane with the manipulator arm is applied in many scenarios. This 2D-picking project is  to reproduce the whole procedure of  picking objects with the manipulator arm(Franka) in 2D plane, basing on the result of visual observation(RGB images) and object detection(YOLO5).
Firstly, in this work, we need to complete the hand-eye calibration. With the eye-to-hand camera fixation, the positions of several points both in camera frame and manipulator's  workspace frame are recorded to get the perspective transform matrix. In this way, coordinate transformation can be achieved. After the completion of calibration accuracy test, YOLO5 algorithm is used to detect the position of the target object based on the visual images information collected by the realsense camera. With the help of the transformation matrix, target position of the end-effector in the arm can be obtained. With all these information, the manipulator arm can be controlled to pick the object in 2D work plane.
## Demo video
 Demo videos of 2D-picking with Franka manipulator arm can be found here. 
 ![demo_video](https://github.com/1079931505/ME336-Yellow-Team-Project1-2D-Picking/tree/main/images/demo.mp4)
<video src="./images/demo.mp4" width="800px" height="600px" controls="controls"></video>
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
2. Pycharm-Community Installation

- Open File at home/user/Me336/ME336-2021Spring and choose the conda environment with Interpreter me336



3. Conda me336 Environment Configuration

- Change the Download source to mirror.tuna.tsinghua.edu.cn
- Enter the environment to install requirements

```
conda activate me336
cd Me336
git clone https://github.com/bionicdl-sustech/ME336-2021Spring.git
cd ME336-2021Spring
pip install -r requirements.txt
```



4. Franka FCI Installation

```
sudo apt install git
cd Me336
git clone --recursive https://github.com/frankaemika/libfranka
cd libfranka
git checkout 0.7.1
git submodule update
sudo apt install build-essential cmake git libpoco-dev libeigen3-dev
mkdir build
cd build
cmake -DCMAKE_BUILD_TYPE=Release ..
cmake --build
sudo make install
```



5. Realsense SDK Installation

- Be sure that the realsense camera is **unconnected** with the host during this installation
- Open the terminal and install

```
sudo apt-get update && sudo apt-get upgrade && sudo apt-get dist-upgrade
sudo apt-get install git libssl-dev libusb-1.0-0-dev pkg-config libgtk-3-dev
sudo apt-get install libglfw3-dev libgl1-mesa-dev libglu1-mesa-dev
git clone https://github.com/IntelRealSense/librealsense.git
cd librealsense/
sudo cp config/99-realsense-libusb.rules /etc/udev/rules.d/ 
sudo udevadm control --reload-rules && udevadm trigger
./scripts/patch-realsense-ubuntu-lts.sh
echo 'hid_sensor_custom' | sudo tee -a /etc/modules
mkdir build && cd build
cmake ../ -DBUILD_EXAMPLES=true
sudo make uninstall && make clean && make -j4 && sudo make install
```

- The camera needs to be connected to the host through a 3.0 or above USB interface after installation
- Open the terminal and run realsense viewer to ensure a successful installation

```
realsense-viewer
```



6. PCL Installation

- Open the terminal and run

```
sudo apt-get update && sudo apt-get upgrade
sudo apt-get install libusb-1.0-0-dev libusb-dev libudev-dev
sudo apt-get install mpi-default-dev openmpi-bin openmpi-common  
sudo apt-get install libflann1.9 libflann-dev
sudo apt-get install libeigen3-dev
sudo apt-get install libboost-all-dev
sudo apt-get install libvtk7.1-qt libvtk7.1 libvtk7-dev
sudo apt-get install libqhull* libgtest-dev
sudo apt-get install freeglut3-dev pkg-config
sudo apt-get install libxmu-dev libxi-dev 
sudo apt-get install mono-complete
sudo apt-get install libpcap-dev
```

- Open this link [PCLgithub](https://github.com/PointCloudLibrary/pcl/releases/tag/pcl-1.8.1) to download the source code(tar.gz)
- Open the terminal again and run

```
cd Downloads
ls
tar -xvzf pcl-pcl-1.8.1.tar.gz
cd pcl-pcl-1.8.1
mkdir build
cd build
cmake ..
make -j4
sudo make install
```



7. pybind11 Environment Installation

- Open the terminal and run 

```
cd Me336
git clone https://github.com/pybind/pybind11.git
cd pybind11
mkdir build
cd build 
cmake ..
make -j4
sudo make install
cd 
cd /home/doyle/Me336/ME336-2021Spring/deepclaw/modules/grasp_planning/GeoGrasp
rm -r build
mkdir build
cd build
cmake ..
make -j4 
cp ./*.so ../
```



8. Franka Driver Installation (Ignored if using the file of teaching)

- Open the terminal and run 

```
conda activate me336
cd /home/doyle/Me336/ME3362021Spring/deepclaw/driver/arms/franka/RobotDirver
rm -r build
mkdir build
cd build
cmake ..
cp RobotDriver.cpython-36m-x86_64-linux-gnu.so ../..
```

# Experimental Procedure

## Get started

1. Check out this repository and download our source code

```
git clone git@github.com:1079931505/ME336-Yellow-Team-SUSTech.git
cd ME336-Yellow-Team-SUSTech 
```

2. Install the required python modules

```
pip install -r requirements.txt
```

3. Update weights of yolo v5

```
sh ./deepclaw/modules/end2end/yolov5/weights/download_weights.sh
```

## Hand eye calibration

1. Install the calibration pen tip on the end flange of the robot arm

![img](https://bionicdl.feishu.cn/space/api/box/stream/download/asynccode/?code=M2E3MzlmNjlhMGY2ZWYzNmJmNzY3MTFkMThiMTM4YmZfUUI2RVhWdW1BUmRMZW1KOUhQNGFmeDM2TkVWOVZtcUtfVG9rZW46Ym94Y25oQUVmdXdITXhOOTQxNkNRcFZUUXNkXzE2MTY5MDUzMzk6MTYxNjkwODkzOV9WNA)

2. Run realsense-viewer in the terminal, click RGB Camera Off and open 2D Camera. Adjust the realsense and camera shelf to ensure that the realsense view is as shown below. 

![img](https://bionicdl.feishu.cn/space/api/box/stream/download/asynccode/?code=YTQzYjdlMTIwZjNkYmRiNmI2Mjg4MDliMWQyNGUzNDRfSzBjVWV1bkVoTVhqZzdTR2h3RU9MVlJMQVo3QW8yZDhfVG9rZW46Ym94Y25TdW1nSDRQa0xkY05YN01xd0VPTGxkXzE2MTY5MDUzMzk6MTYxNjkwODkzOV9WNA)



3. Run Calibration2D.py and record two return values.

```
python ./deepclaw/modules/calibration/Calibration2D.py
```

4. Modify the first two values in FC.move_p() in FrankaController.py, then run it.

```
vim ./deepclaw/driver/arms/franka/FrankaController.py
python ./deepclaw/driver/arms/franka/FrankaController.py
```

![img](https://bionicdl.feishu.cn/space/api/box/stream/download/asynccode/?code=NjRkMzdiNmM3NDFkOGEyMmRjZmMzYWI1NzRiODg4MWJfZE1PbGxSSnB1YnZVQUdITE5zQUVhM2ZjZHlDR3lXQWJfVG9rZW46Ym94Y25NTDUyd21ETlBZdGZUNE1ldU04bFJjXzE2MTY5MDUzMzk6MTYxNjkwODkzOV9WNA)

5. Observe the gap between the end of the robotic arm calibration board and the calibration object on the conveyor belt to check whether the error is reasonable.

## 2-D picking

1. Start the manipulator and connect the manipulator with the internet 192.168.1.102.

![img](https://bionicdl.feishu.cn/space/api/box/stream/download/asynccode/?code=M2VlOGFkNGFkM2M1NDdiMmI4ZTE4MGNiOGIxYjk2NGRfbnkzVWtDQVdxeW9NR00zeTZQQ3BmNG00UnkyZUViTHBfVG9rZW46Ym94Y25UY2tvRUhGY2JzV095WVc1cGpOVnVIXzE2MTY5MDUzMzk6MTYxNjkwODkzOV9WNA)

2. Change the end-effector into Frank Hand and click to unlock joints. 

![img](https://bionicdl.feishu.cn/space/api/box/stream/download/asynccode/?code=OTg0YmFjZTk2NWE0MmM0OWU3M2RlYjQ0YjMwMmZjZTRfdmh4UnhTU0o1TDJBbEk3MWtqOXJtbHZoazlMcW01MExfVG9rZW46Ym94Y252SmtyRFVBQ2VDUjhTejlnMHVxVzVlXzE2MTY5MDUzMzk6MTYxNjkwODkzOV9WNA)


3. Run realsense-viewer then adjust the resolution ratio to 1280x720. 

![img](https://bionicdl.feishu.cn/space/api/box/stream/download/asynccode/?code=MTBkNDU0Mjg0YzlmOGFiYjkwOGQzYTk3MmEyODc1YjNfa0t2OVROMDhMQjY3QzNiSFZsU2pucGVMeGZ5Vk1TTTFfVG9rZW46Ym94Y25IeGhxdnZnaHFTT2d4b0tUUXY1UTh2XzE2MTY5MDUzMzk6MTYxNjkwODkzOV9WNA)

4. Then select a region which contains the object and record the coordinates of the top left corner (x1,y1) and the lower right corner (x2,y2). 

![img](https://bionicdl.feishu.cn/space/api/box/stream/download/asynccode/?code=ZmIxMzgxZDg1YTA2ZmVjYTE2YjE3MGY0MTA4ZmIyNmFfcVRMVWMxaFhiaDBEQ3B3cjJqNkVCZlJDbkpJRzdEMjdfVG9rZW46Ym94Y25Ra2lucUU5S20wQ1pDWUJRUkxlN0RnXzE2MTY5MDUzMzk6MTYxNjkwODkzOV9WNA)

5. Modify the crop_bounding =[y1,y2,x1,x2] in main.py

![img](https://bionicdl.feishu.cn/space/api/box/stream/download/asynccode/?code=YzkyNTQ4MmRmMmY1N2U2ZjBhZjk3NzY5NTZiMDRiOWZfajExdlMzMWx0Z2pITGRFQzRnNVB6N0pGdlJ3ekRSYnZfVG9rZW46Ym94Y241V1kwNk1mVDFQSWE4WFBMYWJGOWxnXzE2MTY5MDUzMzk6MTYxNjkwODkzOV9WNA)

6. Adjust the grabbing height. In case of collision, we should change the value of height from high to low. After test, we find that z = 0.03 will ensure that the manipulator picks the bottle successfully.

![img](https://bionicdl.feishu.cn/space/api/box/stream/download/asynccode/?code=MTNiYTRkYmNiMWY0NjRlM2M3Zjg1NTc0ZGEzMDhkM2RfWFIwRDJCdzAzRDlqSHVFeGVBRHBiYzl4RGRlcEZIeHRfVG9rZW46Ym94Y241TWJSOXZSWHY2bzVQRDVISllRMVRjXzE2MTY5MDUzMzk6MTYxNjkwODkzOV9WNA)

7. Shut the realsense-viewer then run main.py.

## Process

![img](https://bionicdl.feishu.cn/space/api/box/stream/download/asynccode/?code=ZDY0YWNhMDM4ZGY2MTFjMWIxM2RmYTUzZDQ5ZGI0ZGZfbHJYN2tVR09XVVBxRXk0MnpNMndxYlJpSmE2czV5Y0NfVG9rZW46Ym94Y251c3VPRURWajAxZG1peHZZREtMSjNjXzE2MTY5MTU0MzA6MTYxNjkxOTAzMF9WNA)

## Problem and Solution

1. Problem: We have encountered the problem that although we start the frank hand, we can't connect the end-effector with the deepclaw.

$\ \ \ $Solution: Restart the frank hand and connect the end-effector with the deepclaw again.

2. Problem: There is a phase difference of 90° while the end-effector is grabbing the object.

Solution: Modify the code in main.py. 

```
# Exchange the value of the angle of these two situations
if abs(uv[2] - uv[0]) >= abs(uv[3] - uv[1]):
    angle = -1.57
else:
    angle = 0
```

3. Problem: The grabbing location is not precise.

Solution:  Do the calibration again or compensate for the error.



4. Problem: The position and the attitude of the manipulator is not correct.

Solution:  Modify the code in main.py. Do not transfer to rotation vector.

The initial code:

```
 r = R.from_euler('xyz', temp_pose[3:6], degrees=False)
 rvc = r.as_rotvec()
 pick_pose = [temp_pose[0], temp_pose[1], temp_pose[2], rvc[0], rvc[1], rvc[2]]
```

Change the code into:

```
temp_pose = [temp[0], temp[1], z, 3.14, 0, angle]
pick_pose = temp_pose
pick_place(robot, robot, home_joints, pick_pose, place_xyzrt)
```


# Evaluation

In the process of the 2D object-grasping experiment with the robotic arm, we found that when the object is placed in different areas of the conveyor belt, there will be different deviations. When the deviation is large, grasping failure occurs.



Therefore, we hope to explore the causes of this phenomenon, and try our best to reduce such errors so as to improve the success ratio of grasping.



First of all, to figure out the deviation of the robot arm recognition in different position, we substitute the bottle into the calibration object and select three representative points. Then we identify the coordinates of these three points through the camera and control the robot arm to move to its upper end. Finally, the deviation between the end of the robot arm and the calibration object is measured approximately using ruler.





| Measurement times                            | 1           | 2           | 3           |
| -------------------------------------------- | ----------- | ----------- | ----------- |
| Calibrator coordinates                       | （273,296） | （445,300） | （691,302） |
| Deviation between the calibrator and gripper | 3.5cm left  | 2.5cm right | 4.5cm right |



![img](https://bionicdl.feishu.cn/space/api/box/stream/download/asynccode/?code=YTdmYTcyMDEzYjg3MmQ4Y2I1OTkzY2FmOWJiZTBmMDZfdXQ4cFlaeUtCN3VBTnhYVkIzQjNVaG5BZlpFTHRlTTlfVG9rZW46Ym94Y250Z3dBNmxuWHlzTjJpMkptODZkdWhjXzE2MTY5MDUzMzk6MTYxNjkwODkzOV9WNA)



​                                   Actual positions of the points in three measurements



![img](https://bionicdl.feishu.cn/space/api/box/stream/download/asynccode/?code=MWNiYTFmNDRjYWQxNDc3Y2RlMmRlOWFmNmMyODk2Y2ZfUEplUFExVXROZ1J1Mk1qQjl6YXBtb2RLYWpTcjMxVjNfVG9rZW46Ym94Y25pMEo0cUJFTWdtRElUS2xQN2c0S0FnXzE2MTY5MDUzMzk6MTYxNjkwODkzOV9WNA)![img](https://bionicdl.feishu.cn/space/api/box/stream/download/asynccode/?code=ZGRjZGM3MmU3YTA5ODJjZTdmMDdlZmU0YTI1ZTcxYzBfY09QM1M4Y2x6VFd2Sk13dzlYZ2FEdnhtYlJXcEFyMDJfVG9rZW46Ym94Y25zODRlWmlJajBxT1BCRXVSMWJqdlhUXzE2MTY5MDUzMzk6MTYxNjkwODkzOV9WNA)

​    



​        Measurement 1                                                      Measurement 2



![img](https://bionicdl.feishu.cn/space/api/box/stream/download/asynccode/?code=YmVlOGFlNjE2NjliZjkxYjFlMjRlNzBjOThhODI4MjNfcHRuNlIxT2NDR3gyYm94MWdycG43eldwODBFanJJanFfVG9rZW46Ym94Y25ReVc1SFU0UlM2ZlZVajY0U2hYc3hkXzE2MTY5MDUzMzk6MTYxNjkwODkzOV9WNA)



​                                                                  Measurement 3

Through the qualitative experiment, we believe that the size of the error may be related to the coordinate position of the target object. We put forward two conjectures about this phenomenon after discussion.



1.The size of the error is mainly related to the relative position of the target object and the initial selected reference point.



2.The size of the error is caused by the relative position of the target object and the camera.



Admittedly, the deviation is caused by multiple reasons, but we can still verify the dominant factors through experiments.



In this experiment, we will mainly test conjecture 1.

In the process of mechanical arm calibration, we need to select four points for calibration. According to conjecture 1, an intuitive hypothesis is that deviation will be smaller when the target object falls within the area enclosed by the four initial calibration points. Correspondingly, the error is larger when the target object is located outside the enclosed region. To testify it, we design the following experiment.





Firstly, four marking points are selected on the left side of the manipulator arm, and the distance between the four marking points is relatively close.





| Calibration point      | 1           | 2           | 3           | 4           |
| ---------------------- | ----------- | ----------- | ----------- | ----------- |
| Manipulator coordinate | （0.5,0.5） | （0.5,0.6） | （0.4,0.5） | （0.4,0.6） |
| Camera space           | （1151,441) | （1124,416) | （1048,336) | （1121,340) |



We use this set of calibration point to update the program file and control the robotic arm by program. When the target object is to the left of the manipulator (in the area surrounded by the calibration points), the error is small. When the target object is away from the calibration point area, the error will gradually increase. When the target object is on the right side of the manipulator (far from the calibration region), the error of the manipulator is exceedingly large.



![img](https://bionicdl.feishu.cn/space/api/box/stream/download/asynccode/?code=Nzg1MjkwMDY0OGZhYWU5Y2IzMWY4N2JkNDQ3ZWQwZWJfZUF1bldyc2RObGNwNkZqZUFCYVMwYXRwSXFFaEhQTDZfVG9rZW46Ym94Y245WVpTTWVVN0NRQTNVS0V6VDFxdm1mXzE2MTY5MDUzMzk6MTYxNjkwODkzOV9WNA)



Four objects on the right side of the picture represent the actual calibration points we chose,

​     The object on the left side of the picture represents the position of the target object.





![img](https://bionicdl.feishu.cn/space/api/box/stream/download/asynccode/?code=MTY0MzllMDk5NTA4MjFiZDgyOTYyZjNjNzhiNzZlNTZfaENMSkpMeWVMQ1A4cWhZQkJJNDZ1OGQ1ZnQ2MjJtTlhfVG9rZW46Ym94Y24yUkdPVGZoUFBTb09zRzNUZkM5MFdnXzE2MTY5MDUzMzk6MTYxNjkwODkzOV9WNA)

​                                The result of the manipulator grasping, the deviation is large.





Secondly, we select a set of points that are far apart from each other and form a larger area.





| Calibration point      | 1           | 2             | 3           | 4             |
| ---------------------- | ----------- | ------------- | ----------- | ------------- |
| Manipulator coordinate | （0.6,0.4） | （0.6，-0.4） | （0.2,0.4） | （0.2，-0.4） |
| Camera space           | （941,483） | （338，454）  | （969,184） | （371,146）   |

​       

​       

​       

 



Using this set of calibration point to control the robotic arm, we found that deviation decreased greatly. 



We carried out several tests, and results are satisfying when the target object landed in the area surrounded by the calibration point.



![img](https://bionicdl.feishu.cn/space/api/box/stream/download/asynccode/?code=ODBkYzA5OWZhZmMxZThhMTZjOWRjNWFmYWNhYmMzZTlfSXlxVUg2S2RGWDh3QkpuUFB5M0pRWnhFejFxWXlvb1RfVG9rZW46Ym94Y252NjM5Tm1neWQzcWc5RWFMalV6cVllXzE2MTY5MDUzMzk6MTYxNjkwODkzOV9WNA)



The position of the calibration points in the second trial.

The four points are widely distributed and form a larger rectangle.







![img](https://bionicdl.feishu.cn/space/api/box/stream/download/asynccode/?code=MWE5ZjBmYzAxZjhhODU0YTI1MTc0NmQ5YWUzZGRjNjdfWEFycUJCWXlSTEhubWZFT25JU2RSTHB5S0FZU0lRdWRfVG9rZW46Ym94Y24xYTlESVZVb2M1YWlQMFdEZVdLZENkXzE2MTY5MDUzMzk6MTYxNjkwODkzOV9WNA)

The result of the manipulator grasping in the second trial.



Through this experiment, we claim that the deviation will be much smaller when the target object falls inside the area surrounded by four initial calibration points. Furthermore, we can select the calibration points as widely distributed as possible (within the working range of the manipulator) in the subsequent experiments, in order to expand the range of the actual grasping accuracy of the manipulator.



# Improvement

The accuracy of the hand-eye calibration plays an important role in the grasping process. Therefore, we are committed to understand the principle of 2D calibration and try to improve the accuracy.

There are two cases of hand-eye calibration depending on if the camera is fixed or connected to the robot arm. In this project, our camera is fixed corresponding to the ground, that is eye-to-hand.

![img](https://bionicdl.feishu.cn/space/api/box/stream/download/asynccode/?code=MjljZTJkMDIwYjYxMGUyNTcxODk4ZWM4NjhkZjJmODNfbHNQSmU4bmdwQ0hJTU04cGVDRjQ5aGd1MHJCaUlUdDFfVG9rZW46Ym94Y25Fb29OUnJKSlBOakRhTXFDUzRZeXZTXzE2MTY5MDUzMzk6MTYxNjkwODkzOV9WNA)

  In this case, the aim of hand-eye calibration is to calculate the transfer matrix T between robot arm coordinate X1 and camera coordinate X2: 

$$X_1 =TX_2 $$


  T is a homogeneous matrix, so the formula can also be written in form of rotation matrix R and translation matrix C: 

$$X_1  =RX_2+C$$

$$[x_1, y_1]=R[x_2,y_2]+C$$

  After derivation: 

$$x_1=r_1x_1+r_2y_2+c_1$$

$$y_1=r_3x_1+r_4y_2+c_2$$

  There are 6 unknown variables, so at least 3 points are needed to calculate the transfer matrix. Take the error of points’ coordination and camera distortion into consideration, we can reduce the measure error by sampling more points and take average whe. In practice, the nine-points calibration which requires 9 sample points is frequently used to get a better accuracy. Besides, sample points are better to be scattered over a large area in order to reduce the impact of measurement error.

  


 
