import numpy as np
from numpy.ctypeslib import ndpointer
import ctypes
from ctypes import cdll
import os
from datetime import datetime

curr_dir = os.path.dirname(os.path.abspath(__file__))

lib_capturer = cdll.LoadLibrary(curr_dir + '/../cpp/build/libCapturer.so')
lib_matcher = cdll.LoadLibrary(curr_dir + '/../cpp/build/libMatcher.so')

# Set FOV
IMG_W = 2448
IMG_H = 2048

class ImageCapturer:
    def __init__(self):
        lib_capturer.ImageCapturer_new.restype = ctypes.c_void_p

        lib_capturer.ImageCapturer_capture.argtypes = [ctypes.c_void_p]
        lib_capturer.ImageCapturer_capture.restype = ctypes.c_longlong

        self.instance = lib_capturer.ImageCapturer_new()
        
    def capture(self):
        return lib_capturer.ImageCapturer_capture(self.instance)

class StereoMatcher:
    def __init__(self, group='vial'):

        lib_matcher.StereoMatcher_new.argtypes = [ctypes.c_int, ctypes.c_int, ctypes.c_char_p]
        lib_matcher.StereoMatcher_new.restype = ctypes.c_void_p

        lib_matcher.StereoMatcher_match.argtypes = \
            [ctypes.c_longlong, ctypes.c_int]
        lib_matcher.StereoMatcher_match.restype = \
            ndpointer(dtype=ctypes.c_double, shape=(IMG_W * IMG_H *3,))

        g = ctypes.c_char_p(group.encode('utf-8'))
        self.instance = lib_matcher.StereoMatcher_new(IMG_W, IMG_H, g)

        ### Read 3D bounds
        bounds_filepath = 'calibration/' + group + '/CalibPic/roi/bounds.txt'
        self.bounds = []
        try:
            with open(bounds_filepath, 'r') as f_bounds:
                lines = f_bounds.readlines()
                for i in range(1, 6, 2):
                    self.bounds.append([float(v) for v in lines[i].strip().split()])
        except:
            print('Cannot parse 3D bounds')

        ### Read camera extrinsics
        calib_filepath = 'calibration/' + group + '/CalibPic/camera1pic/calibration_result.txt'
        self.rt_mtx = None
        try:
            with open(calib_filepath, 'r') as f_calib:
                lines = f_calib.readlines()
                ### Take the parameter of the first calibration board
                for i in range(len(lines)):
                    if lines[i].startswith('---'):
                        break

                self.rt_mtx = np.array([[float(x) for x in l.split()] for l in lines[i+1:i+4]])
                # print(self.rt_mtx)
        except:
            print('Cannot parse camera extrinsics')

    def match(self, cap_token, index=0):
        flatten_xyz = lib_matcher.StereoMatcher_match(self.instance, cap_token, index)
        xyz = flatten_xyz.reshape(-1, 3)

        ### Filter with nan
        xyz = xyz[~np.isnan(xyz).any(axis=1)]
        print('Original =>', xyz.shape)

        ### Filter by given 3D bounds
        if len(self.bounds) >= 3:
            xyz = xyz[\
                (xyz[:, 0] >= self.bounds[0][0]) & (xyz[:, 0] <= self.bounds[0][1]) &
                (xyz[:, 1] >= self.bounds[1][0]) & (xyz[:, 1] <= self.bounds[1][1]) &
                (xyz[:, 2] >= self.bounds[2][0]) & (xyz[:, 2] <= self.bounds[2][1]) \
            ]
            print('Filtered by XYZ([{:6.3f},{:6.3f}][{:6.3f},{:6.3f}][{:6.3f},{:6.3f}]) =>'.format(\
                self.bounds[0][0], self.bounds[0][1], 
                self.bounds[1][0], self.bounds[1][1], 
                self.bounds[2][0], self.bounds[2][1]), xyz.shape)

        ### Write point cloud file
        timestamp = datetime.now().astimezone().strftime('%Y%m%d_%H%M%S')
        data_filepath = 'dynamic/result3d_{}.xyz'.format(timestamp)
        with open(data_filepath, 'w') as f_data:
            for p in xyz:
                f_data.write('{:.6f} {:.6f} {:.6f}\n'.format(p[0]*1000, p[1]*1000, p[2]*1000))
            print('Point cloud: {}'.format(data_filepath))
        
        ### Convert points from world coordinates (defined by the first chessboard) to camera coordinates
        if self.rt_mtx is not None:
            ## Multiply each point (1x4) by the RT matrix (4x4)
            xyz_ones = np.c_[ xyz, np.ones(xyz.shape[0]) ]
            xyz_camera = np.array([ np.matmul(self.rt_mtx, xyz_one.T)[:3] for xyz_one in xyz_ones ])
            
            ### Write transformed point cloud file
            data_filepath = 'dynamic/result3d_{}_camcoords.xyz'.format(timestamp)
            with open(data_filepath, 'w') as f_data:
                for p in xyz_camera:
                    f_data.write('{:.6f} {:.6f} {:.6f}\n'.format(p[0]*1000, p[1]*1000, p[2]*1000))
                print('Point cloud in camera coords: {}'.format(data_filepath))
            return xyz_camera

        # return xyz
