from threading import Thread
# from pyquaternion import Quaternion
# from filterpy.kalman import KalmanFilter

import cv2
import numpy as np
import pyk4a
from pyk4a import Config, PyK4A

class Kinect:
    def __init__(self):
        self.k4a = None

        self.parameters = None

        self.intrinsic_color = None
        self.intrinsic_depth = None

        self.dist_coef_color = None
        self.dist_coef_depth = None

        # self.pose_filter = KalmanFilter(dim_x=6, dim_z=6)
        # # 초기 상태 및 측정 노이즈 설정
        # self.pose_filter.x = np.zeros(6)  # 초기 상태
        # self.pose_filter.P = np.eye(6)  # 초기 공분산 행렬
        # self.pose_filter.R *= 0.1  # 측정 노이즈 행렬
        # self.pose_filter.Q *= 0.01  # 프로세스 노이즈 행렬
        #
        #
        # # self.init_pose = Quaternion()
        # self.pose      = []
        # self.imu_Hz    = 0.0006        # 208Hz ( 1 / 208 )
        """
        동작 센서(IMU)
        내장형 IMU(관성 측정 장치)는 LSM6DSMUS이며, 가속도계와 자이로스코프를 모두 포함하고 있습니다. 
        가속도계와 자이로스코프는 1.6kHz로 동시에 샘플링됩니다. 샘플은 208Hz로 호스트에 보고됩니다.
        """

        self.isReady = False

    def start(self, size=720):
        if size == 720:
            resolution = pyk4a.ColorResolution.RES_720P
        elif size == 1080:
            resolution = pyk4a.ColorResolution.RES_1080P
        elif size == 1536:
            resolution = pyk4a.ColorResolution.RES_1536P
        else:
            raise ValueError

        try:
            self.k4a = PyK4A(Config(color_resolution=resolution,
                                    depth_mode=pyk4a.DepthMode.WFOV_2X2BINNED,
                                    camera_fps=pyk4a.FPS.FPS_30))
            self.k4a.start()
            self.set_params()

            self.isReady = True

        except Exception as e:
            print(e)

    def set_params(self):
        self.intrinsic_color = self.k4a.calibration.get_camera_matrix(pyk4a.CalibrationType.COLOR)
        self.intrinsic_depth = self.k4a.calibration.get_camera_matrix(pyk4a.CalibrationType.DEPTH)

        self.dist_coef_color = self.k4a.calibration.get_distortion_coefficients(pyk4a.CalibrationType.COLOR)
        self.dist_coef_depth = self.k4a.calibration.get_distortion_coefficients(pyk4a.CalibrationType.DEPTH)

    def get_data(self):
        return self.get_color(), self.get_depth()

    def get_color(self):
        return self.k4a.get_capture().color[:, :, :3][:, :, ::-1]

    def get_depth(self):
        return self.k4a.get_capture().transformed_depth

    def get_imu(self):
        imu = []
        imu.extend(self.k4a.get_imu_sample().pop("acc_sample"))
        imu.extend(self.k4a.get_imu_sample().pop("gyro_sample"))

        return imu

    # def get_pose(self):
    #     acc  = np.array(self.k4a.get_imu_sample().pop("acc_sample"))
    #     gyro = np.array(self.k4a.get_imu_sample().pop("gyro_sample"))
    #
    #     # q_acc  = Quaternion(axis=acc, radians=self.imu_Hz)
    #     # q_gyro = Quaternion(axis=gyro, radians=self.imu_Hz)
    #     #
    #     # self.init_pose = (q_acc * self.init_pose * q_gyro).normalised
    #
    #     # rot = self.init_pose.rotation_matrix
    #     # trans = -self.init_pose.axis * self.init_pose.angle
    #     # print(rot)
    #     # print(trans)
    #     # print()
    #     # self.pose = self.init_pose.transformation_matrix
    #
    #     self.pose_filter.predict()
    #     self.pose_filter.update(np.concatenate((acc, gyro)))
    #     print(self.pose_filter.H.copy())
    #     pose = self.pose_filter.x.copy()
    #     # print(pose)
    #     return pose
