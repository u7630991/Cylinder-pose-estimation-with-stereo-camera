import pcl
import numpy as np
import math
from mpl_toolkits.mplot3d import Axes3D
from scipy.linalg import norm
from scipy.spatial.transform import Rotation as R

    
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
        
def seg(src_pts):

        src_pts = src_pts*1000
        src_pts = src_pts[:,:3]
        src_pts = np.float32(src_pts)

        if (src_pts.size)<50000:
            src_pts = src_pts[::10]
            cloud = pcl.PointCloud(src_pts)


        while (src_pts.size)>=50000:
            src_pts = src_pts[::5]
            cloud = pcl.PointCloud(src_pts)

        print('PointCloud has: ' + str(cloud.size) + ' data points.')
        print(str(cloud[0]))
        pcl.save(cloud, 'input.pcd')
        
        #Segmentation of points of the cylinder curve surface 
        seg = cloud.make_segmenter_normals(ksearch=50)
        seg.set_optimize_coefficients(False)
        seg.set_model_type(pcl.SACMODEL_CYLINDER)
        seg.set_normal_distance_weight(0.01)
        seg.set_method_type(pcl.SAC_RANSAC)
        seg.set_max_iterations(10000)
        seg.set_distance_threshold(0.01)
        seg.set_radius_limits(0,100)


        [inliers_cylinder, coefficients_cylinder] = seg.segment()

        cloud_cylinder = cloud.extract(inliers_cylinder, False)
        print(coefficients_cylinder)

        if cloud_cylinder.size == 0:
            print("Can't find the cylindrical component.")
        else:
            print("PointCloud representing the cylindrical component: " +
                str(cloud_cylinder.size) + " data points.")
        
        return np.asarray(cloud_cylinder), coefficients_cylinder

def cylinder_seg(src_pts):
        #Convert orientation axes from camera space into point cloud space
        data_pts = transform_3d(src_pts, 0, 0, 0, 0, 0, 0)
        out_arr, coefficients_cylinder = seg(data_pts)  
        print('coefficients_cylinder=', coefficients_cylinder)
        #print(out_arr)

        # if coefficients_cylinder[3] < 0:
        #     data_pts = transform_3d(src_pts, 0, 0, 0, np.pi/2, 0, 0)
        #     out_arr, coefficients_cylinder = seg(data_pts)

        # rxyzi = np.array([ coefficients_cylinder[3] , coefficients_cylinder[4],coefficients_cylinder[5]])
        # E1 = R.from_rotvec(rxyzi)
        # e_rxyz = E1.as_euler('xyz', degrees=True)
        # print('e_rxyz1:',e_rxyz)  

        # if (coefficients_cylinder[3]<0 and coefficients_cylinder[4] < 0) or (coefficients_cylinder[5] > 0):
            # e_rxyz = -e_rxyz
            # print(e_rxyz)
            # R1 = R.from_euler('xyz', [e_rxyz], degrees=True)
            # v1 = R1.as_rotvec()
            # print(v1)
            # print(v1[0][1])
            # coefficients_cylinder[3] = v1[0][0]
            # coefficients_cylinder[4] = v1[0][1]
            # coefficients_cylinder[5] = v1[0][2]



        #Generate the rotaion of x and y axis
        # if coefficients_cylinder[4] < 0:
        #     coefficients_cylinder[4] = -coefficients_cylinder[4]
        # if coefficients_cylinder[3] > 0:
            # coefficients_cylinder[3] = -coefficients_cylinder[3]
        ry = - np.arcsin(coefficients_cylinder[3])
        rx =  np.arcsin(coefficients_cylinder[4]/np.cos(ry))

        if coefficients_cylinder[5] > 0:
    
            rx = -rx
            ry = -ry

        print(rx,ry)

        # while rx < 0:
        #     e_rxyz = -e_rxyz
        #     print(e_rxyz)
        #     R1 = R.from_euler('xyz', [e_rxyz], degrees=True)
        #     v1 = R1.as_rotvec()
        #     print(v1)
        #     print(v1[0][1])
        #     coefficients_cylinder[3] = v1[0][0]
        #     coefficients_cylinder[4] = v1[0][1]
        #     coefficients_cylinder[5] = v1[0][2]
        #     ry = - np.arcsin(coefficients_cylinder[3])
        #     rx =  np.arcsin(coefficients_cylinder[4]/np.cos(ry))
        #     print(rx,ry)


        rxyz = np.array([ rx , ry,0  ])
        E2 = R.from_rotvec(rxyz)
        e_rxyz2 = E2.as_euler('xyz', degrees=True)
        print('e_rxyz:',e_rxyz2)




        #Finding the height, base and top point of the center axis 
        axis_arr = transform_3d(out_arr, 0, 0, 0, -rx, -ry, 0)

        max_xyz = np.max(axis_arr,0)
        min_xyz = np.min(axis_arr,0)

        max_z = max_xyz[2]
        min_z = min_xyz[2]

        h = max_z - min_z
        

        p0 = np.array([coefficients_cylinder[0], coefficients_cylinder[1], coefficients_cylinder[2]])
        uv = np.array([coefficients_cylinder[3], coefficients_cylinder[4], coefficients_cylinder[5]])

        min_xyzo_index = np.where(out_arr == np.amin(out_arr[:,2]))

        min_xyzo = np.array([out_arr[min_xyzo_index[0][0]][0], out_arr[min_xyzo_index[0][0]][1], out_arr[min_xyzo_index[0][0]][2]])

        base_pt = p0

        for i in range(100):
            dist = np.linalg.norm(base_pt - min_xyzo)
            axis_pt = p0 - uv * i
            if (coefficients_cylinder[3]<0 and coefficients_cylinder[4] < 0) or (coefficients_cylinder[5] < 0):
                axis_pt = p0 + uv * i

            dist2 = np.linalg.norm(axis_pt - min_xyzo)
            if dist2 < dist:
                base_pt = axis_pt
            
        midpt = base_pt + uv * h/2
        if (coefficients_cylinder[3]<0 and coefficients_cylinder[4] < 0) or (coefficients_cylinder[5] < 0):
            midpt = base_pt - uv * h/2


        #Return the mid-point, rotation of x and y axis a
        return np.array([midpt[0]/1000, midpt[1]/1000, midpt[2]/1000, e_rxyz2[0] , e_rxyz2[1], e_rxyz2[2]])
