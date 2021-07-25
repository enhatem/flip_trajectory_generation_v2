import numpy as np
import matplotlib.pyplot as plt
from numpy.core.numeric import ones_like

from scipy import integrate

from utils import *

def truncate(n, decimals=0):
    multiplier = 10 ** decimals
    return int(n * multiplier) / multiplier

def obj(x):
    # Drone parameters
    m =  29.5e-3 / 2 #  mass
    Ixx =  1.657171e-05 # Inertia
    l = 0.046 # arm length

    # Constants
    g = 9.81 
    t_step = 0.01
    
    # Control bounds
    # Maximum thrust and torque reachable by the drone
    u1_max = 0.9 * ( ( 46e-3 * g ) / 2 ) # Maximum thrust 
    u1_min = 0
    u2_max = 0.1 * ( 1 / 2 * u1_max * l) # Maximum torque
    u2_min = -u2_max

    z1 = x[0]  # initial height of the reaching phase
    z2 = x[1]  # initial height of the flipping phase
    z3 = x[2]  # final   height of the flipping phase
    z4 = x[3]  # final   height of the recovery phase

    phi_start = x[4] # initial roll angle of the flipping phase
    phi_end   = x[5] # final roll angle of the flipping phase

    t1 = truncate(x[6], 2)
    t2 = truncate(x[7], 2)
    t3 = truncate(x[8], 2)

    ## Trajectory waypoints
    # For z
    Z1 = np.array([z1, 0, 0, 0, 0]) # initial position (start of the reaching phase)
    Z2 = np.array([z2, ((z3-z2)/t2 + g*t2/2), -g, 0, 0])
    Z3 = np.array([z3, ((z3-z2)/t2 - g*t2/2), -g, 0, 0])
    Z4 = np.array([z4, 0, 0, 0, 0])

    # For phi
    PHI1 = np.array([0, 0, 0, 0, 0])
    PHI2 = np.array([phi_start, (phi_end - phi_start)/t2, 0, 0, 0])
    PHI3 = np.array([phi_end, (phi_end - phi_start)/t2, 0, 0, 0])
    PHI4 = np.array([2*np.pi, 0, 0, 0, 0])

    ## Reaching phase
    # For Z
    traj1_z = trajectory(Z1,Z2,t1,t_step)

    # For phi
    traj1_phi = trajectory(PHI1,PHI2,t1,t_step)
    T1 = np.arange(0,t1,t_step)

    ## Flip phase
    # For z

    coeff_z2   = np.array([-g/2, ((z3-z2)/t2+g*t2/2), z2])
    coeff_zd2  = np.polyder(coeff_z2)
    coeff_zdd2 = np.polyder(coeff_zd2)

    traj_z2    = np.polyval(coeff_z2, np.arange(0,t2,t_step))
    traj_zd2   = np.polyval(coeff_zd2, np.arange(0,t2,t_step)) 
    traj_zdd2  = np.polyval(coeff_zdd2,np.arange(0,t2,t_step))

    traj2_z = np.array([
            traj_z2,
            traj_zd2,
            traj_zdd2])

    # For phi
    coeff_phi2 = np.array([(phi_end-phi_start)/t2, phi_start])
    coeff_phid2 = np.polyder(coeff_phi2)
    coeff_phidd2 = np.polyder(coeff_phid2)
    phi2 = np.polyval(coeff_phi2, np.arange(0,t2,t_step))
    phid2 = np.polyval(coeff_phid2, np.arange(0,t2,t_step))
    phidd2 = np.polyval(coeff_phidd2,np.arange(0,t2,t_step))
    traj2_phi =np.array([
                phi2,
                phid2,
                phidd2])

    T2 = np.arange(t1,t1+t2, t_step)

    ## Recovery phase
    # For z
    traj3_z = trajectory(Z3,Z4,t3,t_step)

    # For phi
    traj3_phi = trajectory(PHI3,PHI4,t3,t_step)

    T3 = np.arange(t1+t2, t1+t2+t3, t_step)

    t_tot = np.arange(0, t1+t2+t3, t_step)

    z = np.hstack([ traj1_z[0,:], traj_z2, traj3_z[0,:] ])
    phi = np.hstack([ traj1_phi[0,:], phi2, traj3_phi[0,:] ])

    ## Calculating the thrust u1 along the trajectory
    zdd = np.hstack([ traj1_z[2,:], traj_zdd2, traj3_z[2,:] ])
    phidd = np.hstack([ traj1_phi[2,:], phidd2, traj3_phi[2,:] ])

    gravity = g * ones_like(z)

    u1 = m * (zdd + gravity) / np.cos(phi)
    u2 = phidd * Ixx

    ## Calculating ydd 
    ydd = - np.tan(phi) * (zdd + gravity)

    while not len(t_tot)==len(zdd):
        if len(t_tot) > len(zdd):
            t_tot = np.delete(t_tot, -1)
        elif len(t_tot) < len(zdd):
            zdd = np.delete(zdd, -1)
            phidd = np.delete(phidd, -1)
            ydd = np.delete(ydd, -1)
            u1 = np.delete(u1, -1)
            u2 = np.delete(u2, -1)


    yd = integrate.cumtrapz(ydd, t_tot, initial=0)
    y  = integrate.cumtrapz(yd, t_tot, initial=0)

    J = np.trapz(u1, x=t_tot)

    return J