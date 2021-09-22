import numpy as np
import matplotlib.pyplot as plt
from numpy.core.numeric import ones_like

from scipy.optimize import minimize
from scipy import integrate
from utils import *

# Drone parameters
m =  29.5e-3 / 2 #  mass
Ixx =  1.657171e-05 # Inertia
l = 0.046 # arm length

# Constants
t_step = 0.01 # 100 Hz
g = 9.81 # m/s^2

## Trajectory parameters to be optimized

# For z
z1 = 1
z2 = 1.5
z3 = 1.3
z4 = 1

# For phi
phi_start = np.pi/2 - 0.1
phi_end = (3/2)* np.pi + 0.1

# For the time required during each trajectory
t1 = 0.3
t2 = 0.2
t3 = 0.5

## Trajectory waypoints for the z trajectory

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

T3 = np.arange(t1+t2,t1+t2+t3,t_step)





t_tot = np.arange(0,t1+t2+t3,t_step)

z = np.hstack([ traj1_z[0,:], traj_z2, traj3_z[0,:] ])
phi = np.hstack([ traj1_phi[0,:], phi2, traj3_phi[0,:] ])

# fig3, ax3 = plt.subplots()
# ax3.plot(t_tot,z)

# fig4, ax4 = plt.subplots()
# ax4.plot(t_tot,phi)

## Calculating the thrust u1 along the trajectory
zdd = np.hstack([ traj1_z[2,:], traj_zdd2, traj3_z[2,:] ])
phidd = np.hstack([ traj1_phi[2,:], phidd2, traj3_phi[2,:] ])

gravity = g * ones_like(z)

u1 = m * (zdd + gravity) / np.cos(phi)
u2 = phidd * Ixx

## Calculating ydd 
ydd = - np.tan(phi) * (zdd + gravity)

yd = integrate.cumtrapz(ydd, t_tot, initial=0)
y  = integrate.cumtrapz(yd, t_tot, initial=0)


# Z plot
fig1, ax1 = plt.subplots()

ax1.plot(T1,traj1_z[0,:],label='reaching phase')
ax1.plot(T2,traj_z2,     label='flip phase')
ax1.plot(T3,traj3_z[0,:],label='recovery phase')

ax1.set_title('Trajectory along $z$')
ax1.legend()

# Phi plot
fig2, ax2 = plt.subplots()

ax2.plot(T1,traj1_phi[0,:],label='reaching phase')
ax2.plot(T2,phi2,     label='flip phase')
ax2.plot(T3,traj3_phi[0,:],label='recovery phase')

ax2.set_title('Trajectory along $\phi$')
ax2.legend()

# Control inputs plots
fig3, (ax3,ax4) = plt.subplots(nrows=2, ncols=1, sharex=True)
ax3.plot(t_tot,u1,label='thrust')
ax3.set_ylabel('Thrust [N]')
ax3.set_title('Control inputs')

ax4.plot(t_tot,u2,label='torque')
ax4.set_xlabel('Time [s]')
ax4.set_ylabel('Torque [N.m]')

# Y plot

fig4, ax5 = plt.subplots()
ax5.plot(t_tot,y, label='y')

ax5.set_title('Trajectory along y')
ax5.set_xlabel('Time [s]')
ax5.set_ylabel('y [m]')
ax5.legend()

# Planar trajectory
fig5, ax6 = plt.subplots()
ax6.plot(y,z,label='traj')

ax6.set_title('Planar trajectory')
ax6.set_xlabel('y [m]')
ax6.set_ylabel('z [m]')
ax6.legend()

plt.show()