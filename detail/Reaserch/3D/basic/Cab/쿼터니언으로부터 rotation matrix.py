import numpy as np

def quaternion_to_rotation_matrix(quaternion):
    qw = quaternion[0]  #q0
    qx = quaternion[1]  #q1
    qy = quaternion[2]  #q2
    qz = quaternion[3]  #q3
    
    r00 = 2*(qw*qw + qx*qx) -1
    r01 = 2*(qx*qy - qw*qz)
    r02 = 2*(qx*qz + qw*qy)
    
    r10 = 2*(qx*qy + qw*qz)
    r11 = 2*(qw*qw + qy*qy) -1
    r12 = 2*(qy*qz - qw*qx)
    
    r20 = 2*(qx*qz - qw*qy)
    r21 = 2*(qy*qz + qw*qx)
    r22 = 2*(qw*qw + qz*qz) -1
    
    rot_matrix = np.array([[r00, r01, r02], [r10, r11, r12], [r20, r21, r22]])
    
    return rot_matrix 
