# MIT License.
# Copyright (c) 2021 by BioicDL. All rights reserved.
# Created by LiuXb on 2021/1/31
# -*- coding:utf-8 -*-

"""
@Modified: 
@Description:
"""
import sys
import os
# append the DeepClawDev directory to python path and set it as working directory
_root_path = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.append(_root_path)
os.chdir(_root_path)
print('work_dir: ', _root_path)

import time
import yaml
import numpy as np
import matplotlib.pyplot as plt
import cv2
from scipy.spatial.transform import Rotation as R

from deepclaw.driver.arms.ArmController import ArmController
from deepclaw.modules.end2end.yolov5.YOLO5 import Yolo5
from deepclaw.driver.sensors.camera.Realsense_L515 import Realsense

from deepclaw.modules.calibration.Calibration2D import Calibration2D
from deepclaw.driver.arms.franka.FrankaController import FrankaController


def pick_place(robot_server: ArmController, gripper_server: object, home_joint, pick_xyzrt, place_xyzrt):
    # go to pick above
    robot_server.gripperOpen()
    up_pos = pick_xyzrt.copy()
    up_pos[2] = up_pos[2] + 0.1
    robot_server.move_p(up_pos, 0.5, 0.5)
    time.sleep(0.1)
    # go to pick
    robot_server.move_p(pick_xyzrt, 0.5, 0.5)
    time.sleep(0.1)
    # pick
    robot_server.gripperGrasp()
    time.sleep(1)
    # go up
    robot_server.move_p(up_pos, 0.5, 0.5)
    time.sleep(0.1)
    # go to release
    robot_server.move_p(place_xyzrt, 0.5, 0.5)
    # release
    robot_server.gripperOpen()
    time.sleep(0.8)
    # go back home
    robot_server.move_j(home_joint, 0.5, 0.5)



def detectObject(detector_algo: Yolo5, crop_bounding=[ 200, 460,450, 950]):
    region_class = detector_algo.detect(color)
    if region_class is None:
        return False, None, None, None
    print(crop_bounding)
    # object on the tray
    uv_roi = []
    cla_roi = []
    cfi_roi = []
    for i in range(len(region_class)):
        uv_temp = np.array(region_class[i][0:4], np.int16)
        cla_temp = int(region_class[i][5])
        cfi_temp = region_class[i][4]
        # col_min, row_min, col_max, row_max
        uv_mid_c = (uv_temp[0]+uv_temp[2])/2.0
        uv_mid_r = (uv_temp[1]+uv_temp[3])/2.0
        if uv_mid_c < crop_bounding[2] - 20 or uv_mid_c > crop_bounding[3]+10 or uv_mid_r < crop_bounding[0] - 20 or uv_mid_r > crop_bounding[1] + 10:
            continue
        else:
            uv_roi.append(uv_temp)
            cla_roi.append(cla_temp)
            cfi_roi.append(cfi_temp)

    if len(uv_roi) == 0:
        return False, None, None, None

    # pick one object once
    uv = uv_roi[0]
    cla = cla_roi[0]
    cfi = cfi_roi[0]
    return True, uv, cla, cfi


if __name__ == '__main__':
    """ Initialization """
    # camera and robot driver
    robot = FrankaController('./configs/basic_config/franka.yaml')
    robot.gripperGrasp()
    """
    camera = Realsense('./configs/basic_config/camera_rs_d435.yaml')
    object_detector = Yolo5 ('./configs/basic_config/yolov5_cfg.yaml')

    home_joints = [-0.172143, -1.37546, -0.23092, -2.81902, -0.258333, 1.72771, 0.53906]
    robot.gripperOpen()
    robot.move_j(home_joints, 1.5, 1.5)

    # start picking loop
    place_xyzrt = [0.3, 0.0, 0.4, 3.14, 0.0, 0.0]
    crop_bounding = [200, 460, 450, 950]
    cali_path = './configs/basic_config/cali2D.yaml'

    #robot.gripperGrasp()
    while 1:
        frame = camera.get_frame()
        color = frame.color_image[0]
        # 识别物体
        crop_image = color[crop_bounding[0]:crop_bounding[1], crop_bounding[2]:crop_bounding[3]]
        cv2.imshow('sss', crop_image)
        cv2.waitKey()
        # exit()
        #region_class = object_detector.detect(color)
        ret, uv, cla, cfi = detectObject(object_detector, crop_bounding)
        print('uv: ', uv)
        if not ret:
            continue
        if cla not in [0, 1, 2, 3]:
            print('\033[1;35m Error Category \033[0m!')
            continue

        # generate grasping pose
        hand_eye = Calibration2D(cali_path)
        print("Calibration2D complete")
        # col
        ux = (uv[0] + uv[2]) / 2.0
        # row
        vy = (uv[1] + uv[3]) / 2.0
        temp = hand_eye.cvt(ux, vy)
        z = 0.03
        if abs(uv[2] - uv[0]) >= abs(uv[3] - uv[1]):
            angle = -1.57
        else:
            angle = 0

        cv2.circle(color, (int(ux), int(vy)), 2, (255, 0, 0), 2)
        cv2.imshow('aa', color)
        cv2.waitKey()
        # grasp pose in euler angle
        temp_pose = [temp[0], temp[1], z, 3.14, 0, angle]
        print('pick pose: ', temp_pose)
        # transfer to rotation vector
        # r = R.from_euler('xyz', temp_pose[3:6], degrees=False)
        # rvc = r.as_rotvec()
        # pick_pose = [temp_pose[0], temp_pose[1], temp_pose[2], rvc[0], rvc[1], rvc[2]]
        pick_pose = temp_pose
        # grasping
        pick_place(robot, robot, home_joints, pick_pose, place_xyzrt)

"""
