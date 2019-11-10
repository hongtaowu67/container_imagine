#! /usr/bin/env python

"""
Code for processing the raw model for convex decomposition and writing the URDF file
The processed model and the URDF file will be saved in ../model
@author: Hongtao
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


# TODO: write urdf file for simulation
def write_urdf():
    raise NotImplementedError


if __name__ == '__main__':
    root_dir = '/home/hongtao/src/cup_imagine'
    vhacd_dir = os.path.join(root_dir, 'processing/v-hacd/src/build/test/')
    input_file = os.path.join(root_dir, 'model/cup_0003.obj')
    output_file = os.path.join(root_dir,'model/cup_0003_vhacd.obj')
    run_vhacd(vhacd_dir, input_file, output_file)