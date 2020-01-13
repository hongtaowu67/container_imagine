#! /usr/bin/env python

"""
Processing the raw model for convex decomposition and writing the URDF file.
The processed model and the URDF file will be saved in ../model
@author: Hongtao Wu
Nov 05, 2019
"""
import os
import time
import subprocess

#// --input camel.off --output camel_acd.wrl --log log.txt --resolution 1000000 --depth 20 --concavity 0.0025 --planeDownsampling 4 --convexhullDownsampling 4 --alpha 0.05 --beta 0.05 --gamma 0.00125 
# --pca 0 --mode 0 --maxNumVerticesPerCH 256 --minVolumePerCH 0.0001 --convexhullApproximation 1 --oclDeviceID 2


def run_vhacd(vhacd_dir, input_file, output_file, log='log.txt', resolution=500000, depth=20, concavity=0.0025,planeDownsampling=4, 
    convexhullDownsampling=4, alpha=0.05, beta=0.05, gamma=0.00125,pca=0, mode=0, maxNumVerticesPerCH=256, 
    minVolumePerCH=0.0001, convexhullApproximation=1, oclDeviceID=2):
    """
    The wrapper function to run the vhacd convex decomposition.
    """

    vhacd_executable = os.path.join(vhacd_dir, 'testVHACD')
    if not os.path.isfile(vhacd_executable):
        print vhacd_executable
        raise ValueError('vhacd executable not found, have you compiled it?')

    cmd = "cd %s && %s --input %s --output %s --log %s --resolution %s --depth %s --concavity %s --planeDownsampling %s --convexhullDownsampling %s --alpha %s --beta %s --gamma %s \
        --pca %s --mode %s --maxNumVerticesPerCH %s --minVolumePerCH %s --convexhullApproximation %s --oclDeviceID %s" %(vhacd_dir, vhacd_executable, input_file, output_file, log, resolution,
        depth, concavity, planeDownsampling, convexhullDownsampling, alpha, beta, gamma, pca, mode, maxNumVerticesPerCH, minVolumePerCH, convexhullApproximation, oclDeviceID)

    print "cmd:\n", cmd

    start_time = time.time()
    process = subprocess.Popen(cmd, shell=True)
    print "started subprocess, waiting for V-HACD to finish"
    process.wait()
    elapsed = time.time() - start_time

    print "V-HACD took %d seconds" %(elapsed)


def write_urdf(urdf_path, obj_original_file, obj_vhacd_file, mass=0.0, origin_x=0.0, origin_y=0.0, origin_z=0.0,
    origin_roll=0.0, origin_pitch=0.0, origin_yaw=0.0, ixx=0.0, ixy=0.0, ixz=0.0, iyy=0.0, iyz=0.0, izz=0.0):
    """
    Writing the URDF file for simulation.
    Args:
    - obj_original_file: the file name of the original file, e.g., cup_0003.obj
    - obj_vhacd_file: the filename of the vhacd file, e.g., cup_0003_vhacd.obj
    """
    
    f = open(urdf_path, 'w+')
    obj_name = obj_original_file.split('.')[0]

    f.write(
    '<?xml version=\"1.0\" ?>\n'
    '<robot name=\"%s.urdf\">\n'
    '  <link name=\"baseLink\">\n'
    '    <contact>\n'
    '      <lateral_friction value=\"1.0\"/>\n'
    '      <inertia_scaling value=\"1.0\"/>\n'
    '    </contact>\n'
    '    <inertial>\n'
    '      <origin rpy=\"0 0 0\" xyz=\"%.6f %.6f %.6f\"/>\n'
    '      <mass value=\"%.6f\"/>\n'
    '      <inertia ixx=\"%.6f\" ixy=\"%.6f\" ixz=\"%.6f\" iyy=\"%.6f\" iyz=\"%.6f\" izz=\"%.6f\"/>\n'
    '    </inertial>\n'
    '    <visual>\n'
    '      <origin rpy=\"0 0 0\" xyz=\"0 0 0\"/>\n'
    '      <geometry>\n'
    '       <mesh filename=\"%s\" scale=\"1 1 1\"/>\n'
    '      </geometry>\n'
    '       <material name=\"white\">\n'
    '        <color rgba=\"1 1 1 1\"/>\n'
    '      </material>\n'
    '    </visual>\n'
    '    <collision>\n'
    '      <origin rpy=\"0 0 0\" xyz=\"0 0 0\"/>\n'
    '      <geometry>\n'
    '        <mesh filename=\"%s\" scale=\"1 1 1\"/>\n'
    '      </geometry>\n'
    '    </collision>\n'
    '  </link>\n'
    '</robot>\n' %(obj_name, origin_x, origin_y, origin_z, mass, ixx, ixy, ixz, iyy, iyz, izz, obj_original_file, obj_vhacd_file))

    f.close()

    


if __name__ == '__main__':
    start_time = time.time()
    
    root_dir = '/home/hongtao/src/cup_imagine'
    
    model_root_dir = "/home/hongtao/Dropbox/ICRA2021/data"

    object_subdir = "GripperTest_24view"

    object_name = object_subdir + "_mesh_debug_0"

    vhacd_dir = os.path.join(root_dir, 'processing')
    input_file = os.path.join(model_root_dir, object_subdir, object_name + '.obj') 
    output_file = os.path.join(model_root_dir, object_subdir, object_name + '_vhacd.obj')
    run_vhacd(vhacd_dir, input_file, output_file)

    urdf_path = os.path.join(model_root_dir, object_subdir, object_name + '.urdf')
    obj_original_file = object_name + '.obj'
    obj_vhacd_file = object_name + '_vhacd.obj'
    write_urdf(urdf_path, obj_original_file, obj_vhacd_file)
    process_time = time.time() - start_time
    print("Total process time: ", process_time)
