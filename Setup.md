# Setup
The setup for dependency is split into to two sections. The first section is the requirement for imagination only. The second section installs the required package if you want to do real robot experiments.

## Imagination
If you only need the imagination module, please install the following packages.
* [Pybullet](https://pybullet.org/wordpress/): the simulation engine used for the robot imagination. It can be installed by
    ```
    pip install pybullet
    ```
    
* scikit-learn: Principal Component Analysis (PCA) for the pouring imagination
  ```
  pip install scikit-learn
  ```

* [V-HACD](https://github.com/kmammou/v-hacd): convex decomposition of the mesh for pybullet simulation.
  ```
  git clone https://github.com/kmammou/v-hacd
  cd v-hacd/src/
  mkdir build && cd build
  cmake ..
  make
  ```

* ffmpeg: saving the video of the imagination process.
  ```
  sudo apt install ffmpeg
  ```

## Real Robot Experiment
If you want to include the real robot experiments (e.g., robot scanning and robot pouring), please also install the following packages. The experiment has been tested on a UR5 robot.
* [OpenCV](https://opencv.org/): We used OpenCV 3.3 in this project. Please download the source code from the official website and install.

* [TSDF Fusion](https://github.com/hongtaowu67/TSDFfusion-cpu): 3D mesh reconstruction from the depth images capture from the camera. In this repo, we provide the code for installation.
  ```
  cd processing/TSDFfusion
  mkdir build && cd build
  cmake ..
  make
  ```
* [python-pcl](https://python-pcl-fork.readthedocs.io/en/rc_patches4/install.html#install-python-pcl): PCL function to work with point clouds. It is a python wrapper of the PCL library.
  ```
  pip install python-pcl
  ```

* [python-urx](https://github.com/SintefManufacturing/python-urx): a handful python wrapper to interact with UR robots.
  ```
  pip install urx
  ```

* [pyyaml](https://github.com/yaml/pyyaml): a python yaml reader package
  ```
  pip install pyyaml
  ```

* scikit-image: using the marching-cube algorithm to reconstruct the mesh. For python 2.7, we use version 0.14.5.
  ```
  pip install scikit-image
  ```

* plyFile
  ```
  pip install plyfile
  ```

* [ROS](http://wiki.ros.org/): follow the [installation guide](http://wiki.ros.org/kinetic/Installation/Ubuntu) to install ROS.

* openni2: ROS package to interact with PrimeSense camera
  ```
  sudo apt-get install ros-kinetic-openni2-launch
  sudo apt-get install ros-kinetic-openni2-camera
  ```

* [aruco_ros](https://github.com/pal-robotics/aruco_ros): ROS package to detect the pose of ArUco tag. This package is for calibrating the robot arm. To download ArUco tag, see [here](https://chev.me/arucogen/). More details can be found [here](https://github.com/hongtaowu67/engineering_note). To install, clone the repo into your catkin workspace and catkin_make
  ```
  cd ~/catkin_ws/src
  git clone https://github.com/pal-robotics/aruco_ros
  cd ..
  catkin_make
  source devel/setup.bash
  ```