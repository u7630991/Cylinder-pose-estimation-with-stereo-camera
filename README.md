# Cylinder localization using Stereo camera and phase-shift pattern projection

## Overview
* Estimate the pose of the center of a cylinder in the base frame coordinates of the robot arm
  * The labeled region of a test tube (or "sample vial") in our application
* C++ library for 3D scanning
  * Trigger phase-shift pattern projection
  * Trigger image capturing from stereo camera (with 1P2C)
  * Do stereo matching with the pairs of images of patterns
  * Reconstruct the point cloud (filtered by ROI)
  * Transform to camera coordinates
* Point cloud handling
  * Cylinder segmentation
  * Pose estimation
  * Transform to robot arm coordinates

## System requirments & hardware setup
* 1P2C
  * a pair of cameras
    * Camera: Basler acA2440-75μm
    * Lens: C23-1620-5M-P f16mm
    * Baseline distance: 280mm
    * Working distance: 400mm
  * Projector
    * Projection: 280mmx160mm at 400mm
* Computer
  * USB 3.0 ports x2
  * 64GB RAM
  * 12-core CPU
  * NVIDIA GeForce RTX 2080Ti
* Target scene
  * A vial stand is placed at the stated working distance and within the projection area
  * A sample vial is placed at one of the slot
    * having an observable surface (*height* > 20mm; *arc length* > 15mm) that coated by a white label

## Calibration
* Single camera calibration for both camera
* Stereo camera calibration
* Eye-to-hand calibration with the left camera

## Configurations
* Camera matrix, distortion coefficients and R|T matrices for each calibration images
  * Left camera: `calibration/vial/CalibPic/camera1pic/calibration_result.txt`
  * Right camera: `calibration/vial/CalibPic/camera2pic/calibration_result.txt`
* Fundamental matrix
  * Left to right: `calibration/vial/fundamental_matrix1/fundamental_matrix1.txt`
  * Right to left: `calibration/vial/fundamental_matrix2/fundamental_matrix2.txt` *(not used)*
* ROIs
  * Global 3D bounds: `calibration/vial/CalibPic/roi/bounds.txt`
  * Left camera 2D mask: `calibration/vial/CalibPic/roi/camera1.png`
  * Right camera 2D mask: `calibration/vial/CalibPic/roi/camera2.png`
* Camera pose
  * `operator.py` => **`fcp`**
* Capturer setting
  * Camera S/N, IP addr and port: `cpp/capturer_setting.txt`

## Compilation
* Enable projection/camera trigger and GPU computation on dynamic data *(Normal usage)*
  ```
  cd cpp
  mkdir build
  cd build
  rm -rf *
  cmake -D ENABLE_TRIGGER=ON -D ENABLE_COMPUTATION=ON ..
  make
  ```

## Execution
* To get the pose of the vial at slot `index` (*integer*: [0..4])
  ```
  cd ../../
  python3 operator.py <index>
  ```
* Result:
  ```
  coefficients_cylinder= [x, y, z, rotvec_x, rotvec_y, rotvec_z, radius]
  ...
  ...
  [X, Y, Z, Rx_deg, Ry_deg, Rz_deg]
  [X_target, Y_target, Z_target, Rx_deg_target, Ry_deg_target, Rz_deg_target]
  ```
  * The coefficients of the segmented cylinder
  * The second last array: the pose of the center of the cylinder in camera coordinates
  * The last array: the target gripping pose in robot arm coordinates

## Setting up ROS node
* Make the ROS package
  ```
  cd ros_ws
  catkin_make
  ```
* Run the server node with `roslaunch`
  ```
  source devel/setup.bash
  roslaunch stereovision server.launch
  ```
* Run the demo client (in another terminal)
  * Make sure the robot arm controller is running
  ```
  source devel/setup.bash
  rosrun stereovision client_vial_gripping_demo.py
  ```
  * It demostrates dynamic vial localization and gripping
  * In a run:
    * Makes a request to the server node and call the *operator* as done from plain script
    * Trigger robot movements via preset path relative to given dynamic pose 
      so that the vial on the vial stand can be gripped