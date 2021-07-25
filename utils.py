import numpy as np
import pandas as pd
from scipy.linalg import pinv2

def build_matrix(start, goal, t):
    b = np.array([start.flatten(), goal.flatten()])

    A = np.array([
        [1, 0, 0,     0,     0,       0,       0,       0,       0,        0],
        [0, 1, 0,     0,     0,       0,       0,       0,       0,        0],
        [0, 0, 2,     0,     0,       0,       0,       0,       0,        0],
        [0, 0, 0,     6,     0,       0,       0,       0,       0,        0],
        [0, 0, 0,     0,     24,      0,       0,       0,       0,        0,],
        [1, t, t**2,   t**3,   t**4,     t**5,     t**6,     t**7,     t**8,      t**9],
        [0, 1, 2*t,   3*t**2, 4*t**3,   5*t**4,   6*t**5,   7*t**6,   8*t**7,    9*t**8],  
        [0, 0, 2,     6*t,   12*t**2,  20*t**3,  30*t**4,  42*t**5,  56*t**6,   72*t**7],
        [0, 0, 0,     6,     24*t,    60*t**2,  120*t**3, 210*t**4, 336*t**5,  504*t**6],
        [0, 0, 0,     0,     24,      120*t,   360*t**2, 840*t**3, 1680*t**4, 3024*t**5]
    ])

    return A,b



def trajectory(start, goal, time, t_step):
    A,b = build_matrix(start, goal, time)
    A_inv = np.linalg.inv(A)
    b = b.flatten()

    coeff_p = np.fliplr((A_inv@b).reshape(1,10)).flatten()
    t=  np.arange(0,time,t_step)

    coeff_v = np.polyder(coeff_p.flatten()) # coefficients of the velocity polynomial
    coeff_a = np.polyder(coeff_v.flatten()) # coefficients of the acceleration polynomial

    # coeff_j = np.polyder(coeff_a); # coefficients of the jerk polynomial (derivative of the acceleration)
    # coeff_s = np.polyder(coeff_j); # coefficients of the snap polynomial (derivative of the jerk)
    
    # Resulting trajectory
    traj = np.array([
        np.polyval(coeff_p,t),
        np.polyval(coeff_v,t),
        np.polyval(coeff_a,t)
    ])

    return traj