'''
Tools for setting up the imaginebot.
@author: Hongtao Wu
Oct 05, 2019

'''

import pybullet as p
import numpy as np

def getMatrixFromQuaternion(quaternion):
    '''
    quaternion: a quaternion of (x, y, z, w), tuple. It is the original quaternion shape used in Pybullet
    
    Return: a 3x3 rotation matrix, numpy array.
    '''
    return np.reshape(np.array(p.getMatrixFromQuaternion(quaternion)), (3,3))

def array2vector(array):
    '''
    array: a (3, ) array

    Return: a 3x1 numpy vector
    '''
    return np.reshape(np.array(array), (3, 1))

def vector2array(vector):
    '''
    vector: a (3, 1) numpy array 

    Return: a (3, ) numpy array
    '''
    return vector.flatten()

def frobenius_norm(mat1, mat2):
    '''
    mat1, mat2: (3, 3)/(3, ) (orn/pos) numpy array

    Return: a float
    '''
    return np.sqrt(np.sum(np.multiply((mat1 - mat2), (mat1 - mat2))))

def exponential_angle_metric(mat1, mat2):
    '''
    The angle of rotation of between two rotation matrixs mat1, mat2.
    
    mat1, mat2: (3, 3) numpy array

    Return: a float
    '''
    mat = mat1 @ np.linalg.inv(mat2)
    if mat.shape != (3, 3):
        raise ValueError('The dimension of mat1*(mat2)^(-1) is not correct!')
    else:
        return np.arccos((np.trace(mat) -1) / 2) 

if __name__ == '__main__':
    quaternion = (9.55342815300414e-16, 4.371139017411705e-08, 2.185569508705854e-08, 0.9999999999999988)
    print('testing...')
    mat = getMatrixFromQuaternion(quaternion)
    print(frobenius_norm(mat, mat))