import numpy as np
def heading_err(R, R_ref):
    u = R[:, 0]
    #u[2] = 0 #project to xy plane
    v = R_ref[:, 0]
    CosTheta = np.dot(u, v) / (np.linalg.norm(u) * np.linalg.norm(v))
    err = np.arccos(CosTheta)
    xproduct = np.cross(u, v)
    if xproduct[2] > 0:
        err = -err
    return err

def rotx(alpha):
    R = np.matrix([[1, 0, 0], [0, np.cos(alpha), -np.sin(alpha)], [0, np.sin(alpha), np.cos(alpha)]])
    return R

def rotz(alpha):
    R = np.matrix([[np.cos(alpha), -np.sin(alpha), 0], [np.sin(alpha), np.cos(alpha), 0], [0, 0, 1]])
    return R