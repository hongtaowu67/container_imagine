# Container Imagination
This repository is for the imagination of cup with real robots.

# Installation
The project has been tested on python 2.7.

Pybullet

[TSDF Fusion](https://github.com/andyzeng/tsdf-fusion): Need to compile this code.


# Module
## Camera Calibration
We provide a simple calibration process for hand-eye calibration. The calibration is an eye-on-hand calibration (see more explanation [here](https://github.com/jaydenwu17/camera_calibration)). The provided method aims to get the pose of the camera frame in the robot base frame. To do so, the robot moves to 20 pre-defined configurations and record the robot's end-effector pose and the ArUco tag pose for calibration. The robot's end-effector pose in w.r.t. to the robot base frame. The ArUco tag pose is w.r.t. the camera frame.

Before you start, please mount the camera sturdily at the end-effector of the robot. And make sure that you have already conducted the camera calibration to get the camera intrinsic (see how to do camera intrinsic calibration [here](https://github.com/jaydenwu17/camera_calibration)).

In this project, we are using the PrimeSense Carmine 1.09 and a UR5 robot.

### Instruction
1. Print an ArUco Tag from [here](http://chev.me/arucogen/). Mark down the marker ID and the dictionary it belongs to. Also, measure the marker size (in meter).

2. Roslaunch the camera:
```shell
roslaunch openni2_launch openni2.launch
```
3. Roslaunch the aruco_ros:
```shell
roslaunch aruco_ros single.launch markerId:=<marker ID> markerSize:=<marker size in meter>
```

4. Move the robot to 20 configurations. Use the following to make sure that the marker is detected:
```shell
rosrun image_view image_view image:=/aruco_single/result
```

### Real Robot Experiment
TODO: Calibration (can test the chessboard calibration method)

The real robot experiment use *containability_3_1.py* to imagine the containability (**not tested yet**). To run the real robot experiment, run
```shell
python main.py
```
Specify the data directory (directory to save the data), the content urdf and the data name in *main.py*. A directory with the object name will be created in the data directory. The RGB images, depth images, scanned 3D model file (obj), object urdf file, open containability imagination visualization (mp4), and the containability imagination results (txt) will be saved in this directory. 

### Containability Imagination Benchmark
The objects are saved in `test_set_all/` which contains 99 objects at the moment. To run the containability imagination benchmark, run
```shell
python containability_imagination.py
```
Set the content urdf, object directory, and result directory in *containability_imagination.py*. The imagination result (txt) for each objects will be saved in the result directory.

If you want to enable the pouring imagination, then run
```shell
python containability_pour.py
```
Set the content urdf, object directory, result directory, and the bottle urdf in *containability_pour.py*. The imagination result (txt) for each objects will be saved in the result directory.

To benchmark the result with human annotation, run
```shell
python benchmark_human.py
```
Specify the data directory of the imagination result txt, annotation data directory, and the corresponding annotation files in *benchmark_human.py*. The imagination result (containability and/or pouring) will be displayed.
