import os
import sys
DIR = os.path.dirname(os.path.realpath(__file__))
sys.path.append(DIR)
os.chdir(DIR)

from imaging import scanner
from localization import localize
import argparse
import numpy as np

import math
from os import TMP_MAX
import numpy as np
from mpl_toolkits.mplot3d import Axes3D
from scipy.linalg import norm
from scipy.spatial.transform import Rotation as R
from tf.transformations import euler_from_quaternion
import cv2

import tf_conversions.posemath as pm

class Operator:
    def __init__(self):
        self.cap = scanner.ImageCapturer()
        self.matcher = scanner.StereoMatcher()

    def request(self, slot_index=0):
        # Trigger image capturing 
        cap_token = self.cap.capture()

        # Compute the point cloud
        cloud_arr = self.matcher.match(cap_token, slot_index)
        print(cloud_arr.shape)

        if cloud_arr.size == 0:
            return None

        # Localize the glassware and return target gripping pose
        midpt_rxry = localize.cylinder_seg(cloud_arr)
        np.set_printoptions(suppress=True)
        print(midpt_rxry)


        # Camera pose
        fcp = np.array([-0.99755257, -1.21059407, 1.42173296,  -0.11893276681,  -0.15414543234,  0.0276920035])
        fmp = midpt_rxry

        a = np.matrix([0,0,0,1])

        cpr = cv2.Rodrigues(fcp[0:3])
        mpr = fmp[3:]

        cpmtx = np.c_[cpr[0],fcp[3:]]
        cpmtx2 = np.r_[cpmtx,a]


        E1 = R.from_rotvec(mpr)
        mprvec = fmp[3:]


        mpr3x3 = cv2.Rodrigues(mprvec)
        mpmtx = np.c_[mpr3x3[0],fmp[0:3]]
        mpmtx2 = np.r_[mpmtx,a]

        pose4x4 = cpmtx2@mpmtx2
        pose4x4 = pose4x4.squeeze()
        pose_rvec = cv2.Rodrigues(pose4x4[0:3,0:3])

        # print(pose4x4[0:3,3])
        # print(pose4x4)

        r_euler = R.from_rotvec(pose_rvec[0].squeeze())
        e = r_euler.as_euler('xyz', degrees=True)

        # print(e)
        loc_vec = np.array(pose4x4[0:3,3]).squeeze()
        pose = list(loc_vec)+list([-90,0,115])

        print(pose)
        return pose

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Localize glassware on the stand')
    parser.add_argument('slot_index', type=int, help='index of slot')
    args = parser.parse_args()

    operator = Operator()
    operator.request(args.slot_index)
