# ME336_Yellow_Team_Project_2_6D_Picking

# Instruction

In the industrial field, controlling the 6D position and orientation of the end-effector of the manipulator arm is required in many scenarios, such as picking target objects in 3D workplace. This 3D-picking project is to reproduce the whole procedure of picking objects with the manipulator arm(Franka) in 3D workspace, basing on the result of visual observation(RGB images) and object detection(YOLO5).

![img](images/1.png)

Firstly, in this work, we need to complete the hand-eye calibration. With the eye-to-hand camera fixed, the calibration plate is connected on the end of the arm. When the calibration plate moves with the arm, the position and orientation will be recorded. Then, we can get the hand-eye transform matrix. In this way, coordinate transformation can be achieved. 

Secondly, to get relatively accurate point cloud information of the target object, we also need to determine the work plane and truncate needless point cloud.

Finally, we can control the arm to pick the target with the point cloud information and transform matrix.

# Demo video

<img src="images/demo.gif" width="700px" height="400px" controls="controls"></img>

The demo video can be seen in [feishu doc](https://bionicdl.feishu.cn/docs/doccnIlIrbwQbURpeiJHcO4uuYN).

# Contact

- SUSTech me336 yellow team

# Installation

This 3D-Picking project requires the same environments as what has been shown in our 2D-Picking project (Required System: Ubuntu 18.04), which will be demonstrated again as following: 

1. Anaconda Installation and Environment Configuration

- Use this link [Anaconda](https://repo.anaconda.com/archive/Anaconda3-2020.11-Linux-x86_64.sh) to download sh. File and run to install

```
cd Downloads
ls
bash Anaconda3-2020.11-Linux-x86_64.sh
```

- Set the environment configuration for Anaconda

- Creative a ***me336*** conda environment (python 3.6)

```
conda create -n me336 python=3.6
```



1. Pycharm-Community Installation

- Open File at home/user/Me336/ME336-2021Spring and choose the conda environment with Interpreter me336



1. Conda me336 Environment Configuration

- Change the Download source to mirror.tuna.tsinghua.edu.cn

- Enter the environment to install requirements

```
conda activate me336
cd Me336
git clone https://github.com/bionicdl-sustech/ME336-2021Spring.git
cd ME336-2021Spring
pip install -r requirements.txt
```



1. Franka FCI Installation

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



1. Realsense SDK Installation

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



1. PCL Installation

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



1. pybind11 Environment Installation

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



1. Franka Driver Installation (Ignored if using the file of teaching)

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

# Experiment procedure

## Getting started

1. Check out this repository and download our source code

```
git clone git@github.com:1079931505/ME336-Yellow-Team-SUSTech.git
cd ME336-Yellow-Team-SUSTech 
```

1. Install the required python modules

```
pip install -r requirements.txt
```

1. Update weights of yolo v5

```
sh ./deepclaw/modules/end2end/yolov5/weights/download_weights.sh
```

## 3D Calibration

This procedure will complete the 3D hand-eye calibration between the camera and the robot arm. In particular, we will get a hand-eye transform matrix which gives the relative pose of robot arm$$p'(x',y',z')$$ to camera$$p(x,y,z)$$.

![img](images/2.png)

![img](images/3.png)

1. Modify the configuration file
    1. Open the file ME336Spring/configs/basic_config/cail3D.yaml
    2. Modify the parameters of offset in E_T_F values. Please pay attention to the status of the robot arm. (The gripper will make the coordinate direction change.)

![img](images/4.png)

1. Run the hand eye calibration
    1. Open this  file in PyCharm ME336-2021Spring/deepclaw/modules/calibration/EyeOnBase.py . It also needs to change the saving path for calibration data and calculation outcoming. If you need to collect the calibration data, change collect_flag =True .

![img](images/5.png)

1. Run the EyeOnBase.py file. The robot arm will move on the generating mesh according to the given home position and step length. You can get the camera information by realsense in time.

![img](images/6.png)

1. After running, the robot arm ends with completing the calibration. If you choose to collect data, you will obtain .npz and .npy files in the appointed saving path.

1. Plane calculation
    1. Open realsense-viewer, and then turn Stereo Module on.

![img](images/7.png)

1. Choose four random points on the conveyor belt, and obtain the coordinate values$$(x,y,z)$$. Then close RealSense.
2. Open the file plane_calculate.py and fill the ***xyzs*** with four point coordinate values. Then run this file. 
3. We can obtain the output as model para, then copy this data to main.py in plane_model.

![img](images/8.png)

## 6-D picking

According to the vision recognition algorithm and the Point Cloud data, we calculate the picking attitude. In this experiment, we will use the GeoGrasp algorithm to obtain the best attitude in the camera coordinate and use the hand-eye matrix transformation to obtain the best attitude in the mechanical arm base coordinate, which is suitable to grasp the waste.

1. Coordinate transformation

Measure the distance between the end effector and the center of the claw and set the measure_z as below. Firstly, we set a larger z in case that the claw will hit the convey belt, then decrease it step by step until the claw can grasp the waste in the proper attitude.

```
measure_z = -0.06  # meter
temp_pose = [transfer[0], transfer[1], transfer[2] + measure_z, rot[0], rot[1], rot[2]]
```

1. Open the realsense-viewer and choose a rectangle as the recognition region for the camera.

![img](images/9.png)

1. Record the (x1,y1) of the up-left point and (x2,y2) of the down-right point then modify the value of the crop_bounding  =[y1,y2,x1,x2].

![img](images/10.png)

1. Place a bottle on the convey belt and ensure that there is no other object in the view.
2. Run the main.py and observe the robot arm on how to collect the waste.



# Evaluation

According to the steps in the experimental manual, we can complete the 3D calibration and the subsequent 6D grasping process. However, at the same time, during the 6D grasping, large errors often occur when the manipulator moves to grasp, and even lead to the failure of grasping.It is difficult for us to improve the hardware of the manipulator at the present stage, so we hope to improve the accuracy of grasping movement by improving the 3D calibration method and modifying the camera to reduce the errors caused by the camera itself. 

![img](images/11.png)


We assume that the idea of improving the mark is based on the fact that changing the calibration area can have a great impact on the grasping accuracy of the manipulator during 2D calibration and grabbing. Therefore, it is speculated that improvement can be made in this aspect in 3D calibration.In addition, since the manipulator arm needs to move in a 4x4x4 grid in 3D calibration, it can be considered to change the step of each movement to test whether the grasping accuracy is affected.





# Improvement

We found out that the picking accuracy was high when the target lies near to the center of the calibration space. Nevertheless, the accuracy declines sharply as the target moves away from the center.

![img](images/12.png)

![img](images/13.png)

![img](images/14.png)

![img](images/15.png)

After excluding the problem of mis- detection or path planning, we narrow down the problem to the camera.

First, we tried to enlarge the calibration space to improve  the performance. We increased the stride of calibration to expand the space. However, the accuracy did not increase significantly.

![img](images/16.png)

![img](images/17.png)


Then, through reading the source codes, we discovered that the problem might result from the given camera intrinsic matrix since our camera could be different.

After the correct intrinsic matrix was calculated, we updated the focal length parameters fx, fy, cx, cy used in calibration.

![img](images/18.png)

The picking accuracy was highly improved.

![img](images/19.png)

![img](images/20.png)
