# -*- coding: utf-8 -*-
# Cylinder model segmentation
# http://pointclouds.org/documentation/tutorials/cylinder_segmentation.php#cylinder-segmentation
# dataset : https://raw.github.com/PointCloudLibrary/data/master/tutorials/table_scene_mug_stereo_textured.pcd

import pcl
import open3d as o3d
import utils
import operation
import matplotlib.pyplot as plt
import numpy as np
from mpl_toolkits.mplot3d import Axes3D
from scipy.linalg import norm


def main():
    # typedef pcl::PointXYZ PointT;
    # int main (int argc, char** argv)
    # // All the objects needed
    # pcl::PCDReader reader;
    # pcl::PassThrough<PointT> pass;
    # pcl::NormalEstimation<PointT, pcl::Normal> ne;
    # pcl::SACSegmentationFromNormals<PointT, pcl::Normal> seg;
    # pcl::PCDWriter writer;
    # pcl::ExtractIndices<PointT> extract;
    # pcl::ExtractIndices<pcl::Normal> extract_normals;
    # pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT> ());
    #
    # // Datasets
    # pcl::PointCloud<PointT>::Ptr cloud (new pcl::PointCloud<PointT>);
    # pcl::PointCloud<PointT>::Ptr cloud_filtered (new pcl::PointCloud<PointT>);
    # pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);
    # pcl::PointCloud<PointT>::Ptr cloud_filtered2 (new pcl::PointCloud<PointT>);
    # pcl::PointCloud<pcl::Normal>::Ptr cloud_normals2 (new pcl::PointCloud<pcl::Normal>);
    # pcl::ModelCoefficients::Ptr coefficients_plane (new pcl::ModelCoefficients), coefficients_cylinder (new pcl::ModelCoefficients);
    # pcl::PointIndices::Ptr inliers_plane (new pcl::PointIndices), inliers_cylinder (new pcl::PointIndices);
    #
    #Convert the scanned ply file into pcd file

    data_pt = np.genfromtxt('static/vial_1213_calibL.xyz', delimiter=' ', dtype=np.float32)

    data = operation.transform_3d(data_pt, 0, 0, 0, -np.pi/2, 0, 0)

    data = np.float32(data)
    #print(data.dtype)

    # // Read in the cloud data
    # reader.read ("table_scene_mug_stereo_textured.pcd", *cloud);
    # std::cerr << "PointCloud has: " << cloud->points.size () << " data points." << std::endl;

    if (data.size)<50000:
        data = data[::10]
        cloud = pcl.PointCloud(data)

    while (data.size)>=50000:
        data = data[::5]
        cloud = pcl.PointCloud(data)
    #cloud = operation.transform_3d(cloud, 0, 0, 0, np.pi/2, 0, 0)
    print('PointCloud has: ' + str(cloud.size) + ' data points.')
    

    '''
    # Build a passthrough filter to remove spurious NaNs
    # pass.setInputCloud (cloud);
    # pass.setFilterFieldName ("z");
    # pass.setFilterLimits (0, 1.5);
    # pass.filter (*cloud_filtered);
    # std::cerr << "PointCloud after filtering has: " << cloud_filtered->points.size () << " data points." << std::endl;
    
    passthrough = cloud.make_passthrough_filter()
    passthrough.set_filter_field_name('z')
    passthrough.set_filter_limits(0, 100)
    cloud_filtered = passthrough.filter()
    #cloud_filtered = cloud
    print('PointCloud has: ' + str(cloud_filterevial_1213_sampling2.xyzd.size) + ' data points.')
    

    
    # Estimate point normals
    # ne.setSearchMethod (tree);
    # ne.setInputCloud (cloud_filtered);
    # ne.setKSearch (50);
    # ne.compute (*cloud_normals);
    ne = cloud_filtered.make_NormalEstimation()
    tree = cloud_filtered.make_kdtree()
    ne.set_SearchMethod(tree)
    ne.set_KSearch(100)
    # cloud_normals = ne.compute ()

    # Create the segmentation object for the planar model and set all the parameters
    # seg.setOptimizeCoefficients (true);
    # seg.setModelType (pcl::SACMODEL_NORMAL_PLANE);
    # seg.setNormalDistanceWeight (0.1);
    # seg.setMethodType (pcl::SAC_RANSAC);
    # seg.setMaxIterations (100);
    # seg.setDistanceThreshold (0.03);
    # seg.setInputCloud (cloud_filtered);
    # seg.setInputNormals (cloud_normals);
    # // Obtain the plane inliers and coefficients
    # seg.segment (*inliers_plane, *coefficients_plane);
    # std::cerr << "Plane coefficients: " << *coefficients_plane << std::endl;

    # SACSegmentationFromNormals
    # seg = cloud_filtered.make_segmenter_normals(ksearch=50)
    seg = cloud_filtered.make_segmenter_normals(ksearch=50)
    seg.set_optimize_coefficients(True)
    seg.set_model_type(pcl.SACMODEL_NORMAL_PLANE)
    seg.set_normal_distance_weight(0.1)
    seg.set_method_type(pcl.SAC_RANSAC)
    seg.set_max_iterations(100)
    seg.set_distance_threshold(0.03)
    # seg.set_InputNormals (cloud_normals)
    [inliers_plane, coefficients_plane] = seg.segment()

    # // Extract the planar inliers from the input cloud
    # extract.setInputCloud (cloud_filtered);
    # extract.setIndices (inliers_plane);
    # extract.setNegative (false);
    #
    # // Write the planar inliers to disk
    # pcl::PointCloud<PointT>::Ptr cloud_plane (new pcl::PointCloud<PointT> ());
    # extract.filter (*cloud_plane);
    # std::cerr << "PointCloud representing the planar component: " << cloud_plane->points.size () << " data points." << std::endl;
    # writer.write ("table_scene_mug_stereo_textured_plane.pcd", *cloud_plane, false);
    cloud_plane = cloud_filtered.extract(inliers_plane, False)
    print('PointCloud representing the planar component: ' +
          str(cloud_plane.size) + ' data points.\n')
    pcl.save(cloud_plane, 'half_cylinder_plane.pcd')
    

    # // Remove the planar inliers, extract the rest
    # extract.setNegative (true);
    # extract.filter (*cloud_filtered2);
    cloud_filtered2 = cloud_filtered.extract(inliers_plane, True)

    # extract_normals.setNegative (true);
    # extract_normals.setInputCloud (cloud_normals);
    # extract_normals.setIndices (inliers_plane);
    # extract_normals.filter (*cloud_normals2);
    # cloud_normals2 = cloud_normals.extract(inliers_plane, True)
    

    
    #
    # // Create the segmentation object for cylinder segmentation and set all the parameters
    # seg.setOptimizeCoefficients (true);
    # seg.setModelType (pcl::SACMODEL_CYLINDER);
    # seg.setMethodType (pcl::SAC_RANSAC);
    # seg.setNormalDistanceWeight (0.1);
    # seg.setMaxIterations (10000);
    # seg.setDistanceThreshold (0.05);
    # seg.setRadiusLimits (0, 0.1);
    # seg.setInputCloud (cloud_filtered2);
    # seg.setInputNormals (cloud_normals2);
    #
    '''
    # // Obtain the cylinder inliers and coefficients
    # seg.segment (*inliers_cylinder, *coefficients_cylinder);
    # std::cerr << "Cylinder coefficients: " << *coefficients_cylinder << std::endl;
    seg = cloud.make_segmenter_normals(ksearch=50)
    seg.set_optimize_coefficients(False)
    seg.set_model_type(pcl.SACMODEL_CYLINDER)
    seg.set_normal_distance_weight(0.1)
    seg.set_method_type(pcl.SAC_RANSAC)
    seg.set_max_iterations(10000)
    seg.set_distance_threshold(0.01)
    seg.set_radius_limits(0,100)
    #seg.set_InputNormals (cloud_normals2)
    [inliers_cylinder, coefficients_cylinder] = seg.segment()
    print(coefficients_cylinder)
    

    #   // Write the cylinder inliers to disk
    #   extract.setInputCloud (cloud_filtered2);
    #   extract.setIndices (inliers_cylinder);
    #   extract.setNegative (false);
    #   pcl::PointCloud<PointT>::Ptr cloud_cylinder (new pcl::PointCloud<PointT> ());
    #   extract.filter (*cloud_cylinder);
    cloud_cylinder = cloud.extract(inliers_cylinder, False)

    #   if (cloud_cylinder->points.empty ())
    #     std::cerr << "Can't find the cylindrical component." << std::endl;
    #   else
    #   {
    #     std::cerr << "PointCloud representing the cylindrical component: " << cloud_cylinder->points.size () << " data points." << std::endl;
    #     writer.write ("table_scene_mug_stereo_textured_cylinder.pcd", *cloud_cylinder, false);
    #   }
    #
    if cloud_cylinder.size == 0:
        print("Can't find the cylindrical component.")
    else:
        print("PointCloud representing the cylindrical component: " +
              str(cloud_cylinder.size) + " data points.")
        

    
    out_arr = np.asarray(cloud_cylinder)  
    #print (out_arr)  

    # Convert direction unit vector into rotation of x and y axis
    ry = - np.arcsin(coefficients_cylinder[3])
    rx = np.arcsin(coefficients_cylinder[4]/np.cos(ry))


    
    if rx < 0:
        data = operation.transform_3d(data_pt, 0, 0, 0, np.pi/2, 0, 0)

        data = np.float32(data)

        if (data.size)<=50000:
            data = data[::10]
            cloud = pcl.PointCloud(data)

        while (data.size)>50000:
            data = data[::5]
            cloud = pcl.PointCloud(data)
   
        print('PointCloud has: ' + str(cloud.size) + ' data points.')
    

        seg = cloud.make_segmenter_normals(ksearch=50)
        seg.set_optimize_coefficients(False)
        seg.set_model_type(pcl.SACMODEL_CYLINDER)
        seg.set_normal_distance_weight(0.1)
        seg.set_method_type(pcl.SAC_RANSAC)
        seg.set_max_iterations(10000)
        seg.set_distance_threshold(0.01)
        seg.set_radius_limits(0,100)

        [inliers_cylinder, coefficients_cylinder] = seg.segment()

        

        cloud_cylinder = cloud.extract(inliers_cylinder, False)

     
        if cloud_cylinder.size == 0:
            print("Can't find the cylindrical component.")
        else:
            print("PointCloud representing the cylindrical component: " +
                str(cloud_cylinder.size) + " data points.")
            

        
        out_arr = np.asarray(cloud_cylinder)  
        #print (out_arr)  

        # Convert direction unit vector into rotation of x and y axis
        ry = - np.arcsin(coefficients_cylinder[3])
        rx = np.arcsin(coefficients_cylinder[4]/np.cos(ry))
    



    # Find the maximum and minimun points along the central axis of the cylinder
    axis_arr = operation.transform_3d(out_arr, 0, 0, 0, -rx, -ry, 0)


    #min_zo = out_arr[0][2]
    #min_xyzo = np.array([out_arr[0][0], out_arr[0][1], out_arr[0][2]])

    max_xyz = np.max(axis_arr,0)
    min_xyz = np.min(axis_arr,0)

    max_z = max_xyz[2]
    min_z = min_xyz[2]

    h = max_z - min_z
    

    p0 = np.array([coefficients_cylinder[0], coefficients_cylinder[1], coefficients_cylinder[2]])
    uv = np.array([coefficients_cylinder[3], coefficients_cylinder[4], coefficients_cylinder[5]])

    min_xyzo_index = np.where(out_arr == np.amin(out_arr[:,2]))
    #print(min_xyzo_index[0][0])
    min_xyzo = np.array([out_arr[min_xyzo_index[0][0]][0], out_arr[min_xyzo_index[0][0]][1], out_arr[min_xyzo_index[0][0]][2]])

    base_pt = p0


    for i in range(100):
        dist = np.linalg.norm(base_pt - min_xyzo)
        axis_pt = p0 + uv * i
        dist2 = np.linalg.norm(axis_pt - min_xyzo)
        if dist2 < dist:
            base_pt = axis_pt
        
    p1 = base_pt - uv * h
    midpt = base_pt - uv * h/2

    print('Midpoint of the cylinder:', midpt)
    print('Rotation of x-axis:', rx)
    print('Rotation of y-axis:', ry)
    print('Radius:',coefficients_cylinder[6])
    print('Height:',h)
    #plot axis and point clouds
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    ax.plot(*zip(base_pt,p1), color = 'red')
    cyli = utils.gen_cylinder_circumference_points(h,coefficients_cylinder[6])
    cyli2 = operation.transform_3d(utils.gen_cylinder_circumference_points(h,coefficients_cylinder[6]), base_pt[0], base_pt[1], base_pt[2], rx, ry, 0)
    #result1 = utils.compare_cylinders(out_arr, cyli2)
    #print('result1:', result1)
    #ax.scatter(cyli[:,0], cyli[:,1], cyli[:,2])
    ax.scatter(cyli2[:,0], cyli2[:,1], cyli2[:,2])
    ax.scatter(out_arr[:,0], out_arr[:,1], out_arr[:,2])
    #ax.scatter(axis_arr[:,0], axis_arr[:,1], axis_arr[:,2])
    plt.show()

    
    


if __name__ == "__main__":
    # import cProfile
    # cProfile.run('main()', sort='time')
    main()