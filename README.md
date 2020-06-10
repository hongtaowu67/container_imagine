# Container Imagination
This repository is for the imagination of cup with real robots.

# Installation

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

