import numpy as np

def create_base_transformation(x_base, y_base, z_base, roll, pitch, yaw):
    rotxyz = create_rot_xyz(roll, pitch, yaw)
    Tbase = create_base_translation(x_base, y_base, z_base)
    Tm = np.matmul(rotxyz, Tbase)
    return Tm

def create_base_translation(x_base, y_base, z_base):
    Tbase = np.matrix([
        [1, 0, 0, x_base],
        [0, 1, 0, y_base],
        [0, 0, 1, z_base],
        [0, 0, 0, 1],
    ])
    return Tbase

def create_rot_xyz(roll, pitch, yaw):
    rotx = create_rotx(roll)
    roty = create_roty(pitch)
    rotz = create_rotz(yaw)
    rotxy = np.matmul(rotx,roty)
    rotxyz = np.matmul(rotxy,rotz)
    return rotxyz

def create_rotx(roll):
    rotx = np.matrix([
        [1, 0, 0, 0],
        [0, np.cos(roll), -np.sin(roll), 0],
        [0, np.sin(roll), np.cos(roll), 0],
        [0, 0, 0, 1],
    ])
    return rotx

def create_roty(pitch):
    roty = np.matrix([
        [np.cos(pitch), 0, -np.sin(pitch), 0],
        [0, 1, 0, 0],
        [np.sin(pitch), 0, np.cos(pitch), 0],
        [0, 0, 0, 1],
    ])
    return roty

def create_rotz(yaw):
    rotz = np.matrix([
        [np.cos(yaw), -np.sin(yaw), 0, 0],
        [np.sin(yaw), np.cos(yaw), 0, 0],
        [0, 0, 1, 0],
        [0, 0, 0, 1],
    ])
    return rotz