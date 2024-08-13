
import numpy as np
import math

def transform_2d(src_pts, x, y, angle):
    '''Apply 2D transformation
    Args:
        src_pts: a 2D Numpy array of 2D vectors
        x: shift in x direction
        y: shift in y direction
        angle: rotation angle around the center (in radians)

    Returns:
        A 2D Numpy array for the 2D vectors
    '''
    r_mtx = np.matrix([
        [ math.cos(angle),-math.sin(angle) ],
        [ math.sin(angle), math.cos(angle) ]])
    t_mtx = np.array([[x], [y]])
    return np.array((r_mtx * src_pts.T + t_mtx).T)


def transform_3d(src_pts, x, y, z, rot_x, rot_y, rot_z):
    '''Apply 3D transformation
    
    Args:
        src_pts: a 2D Numpy array of 3D vectors
        x: shift in x direction
        y: shift in y direction
        z: shift in z direction
        rot_x: rotation angle around x axis (in radians)
        rot_y: rotation angle around y axis (in radians)
        rot_z: rotation angle around z axis (in radians)

    Returns:
        A 2D Numpy array for the 3D vectors
    '''

    # Construct the rotation matrices for each Euler angles
    r_x = np.matrix([
        [ 1, 0, 0 ],
        [ 0, math.cos(rot_x),-math.sin(rot_x)],
        [ 0, math.sin(rot_x), math.cos(rot_x)]])
    r_y = np.matrix([
        [ math.cos(rot_y), 0, math.sin(rot_y)],
        [ 0, 1, 0 ],
        [-math.sin(rot_y), 0, math.cos(rot_y)]])
    r_z = np.matrix([
        [ math.cos(rot_z),-math.sin(rot_z), 0 ],
        [ math.sin(rot_z), math.cos(rot_z), 0 ],
        [ 0, 0, 1 ]])

    # Rotation followed by translation
    r_mtx = r_z * r_y * r_x
    t_mtx = np.array([[x], [y], [z]])
    return np.array((r_mtx * src_pts.T + t_mtx).T)
