import numpy as np
import matplotlib.pyplot as plt
from numpy.core.numeric import ones_like
from numpy.linalg import solve

from scipy.optimize import minimize
from scipy.optimize import shgo
from scipy.optimize import Bounds
from scipy.optimize import NonlinearConstraint


from scipy import integrate

from nonlinear_bounds import nl_con
from objective_function import obj

from utils import *

# Drone parameters
m =  29.5e-3 / 2 #  mass
Ixx =  1.657171e-05 # Inertia
l = 0.046 # arm length

# Constants
t_step = 0.01 # 100 Hz
g = 9.81 # m/s^2

# Bounds on the z trajectory of the reaching phase
z1_min = 0.8
z1_max = 3

# Bounds on the z trajectory of the flipping phase
z2_min = 0.8
z2_max = 3

# Bounds on the z trajectory of the recovery phase
z3_min = 0.8
z3_max = 3

# Maximum thrust and torque reachable by the drone
u1_max = 0.9 * ( ( 46e-3 * g ) / 2 ) # Maximum thrust 
u2_max = 0.1 * ( 1 / 2 * u1_max * l) # Maximum torque

## Constraints applied on the optimization problem

# lower bound on the initial phi angle at the start of the reaching phase
phi_start_min = 0

# upper bound on the final phi angle at the end of the reaching phase
phi_start_max = np.pi/2

# lower bound on the initial phi angle at the end of the flipping phase (start of the recovery phase)
phi_end_min = 3/2*np.pi

# upper bound on the final phi angle at the end of the recovery phase
phi_end_max   = 2*np.pi

# bounds on t1 (time of the reaching phase trajectory)
t1_min = 0.1
t1_max = np.inf

# bounds on t2 (time of the flipping phase trajectory)
t2_min = 0.1
t2_max = np.inf

# bounds on t3 (time of the recovery phase trajectory)
t3_min = 0.1
t3_max = np.inf

# setting the lower and upper bounds for each vaiable

bnd_z1          = (z1_min, z1_max)
bnd_z2          = (z1_min, z1_max)
bnd_z3          = (z2_min, z2_max)
bnd_z4          = (z3_min, z3_max)
bnd_phi_start   = (phi_start_min, phi_start_max)
bnd_phi_end     = (phi_end_min, phi_end_max)
bnd_t1          = (t1_min, t1_max)
bnd_t2          = (t2_min, t2_max)
bnd_t3          = (t3_min, t3_max)

bnds = (bnd_z1, bnd_z2, bnd_z3, bnd_z4,
        bnd_phi_start, bnd_phi_end, 
        bnd_t1, bnd_t2, bnd_t3)

# bounds = Bounds (   [z1_min, z1_max],               # z1 bounds 
#                     [z1_min, z1_max],               # z2 bounds
#                     [z2_min, z2_max],               # z3 bounds
#                     [z3_min, z3_max],              # z4 bounds
#                     [phi_start_min, phi_start_max], # phi_start bounds
#                     [phi_end_min, phi_end_max],     # phi_end bounds
#                     [t1_min, t1_max],               # t1 bounds
#                     [t2_min, t2_max],               # t2 bounds
#                     [t3_min, t3_max])               # t3 bounds

nonlinear_constraint = NonlinearConstraint(nl_con, -np.inf, 0)

x0 = np.array([ 0.9, 2.5, 1.5, 0.9, np.pi/2-0.2, (3/2+0.2)*np.pi, 0.4, 0.3, 0.4 ])

sol = minimize(obj, x0, method = 'trust-constr',
                constraints=nonlinear_constraint,
                options={'verbose': 3, 'maxiter': 1e50, 'xtol': 1e-8}, bounds=bnds)

# print(sol)