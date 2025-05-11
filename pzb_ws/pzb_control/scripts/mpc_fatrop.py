#!/usr/bin/env python3

from rockit import *
from casadi import *
import rockit

import matplotlib.pyplot as plt
import numpy as np

from IPython import embed

from scipy.optimize import minimize

obs_n = (int)(10)                # Number of obstacles x 2 (2D position)
nx    = (int)(3 + (int)(obs_n)) # number of states
nu    = 2                       # the system has 2 inputs
dt    = 0.1                     # sample time

# Static avoidance - general processing
# Tf    = 1.0           # control horizon [s]
Tf    = 20.0           # control horizon [s]


Nhor  = (int)(Tf/dt)    # number of control intervals

initial_x = 0.01
initial_y = 0.01
initial_psi = 0.01

# Default initial target parameter: [x0,y0] -> [x1,y1, psi1]
initial_target = np.array([0.0, 0.0, 
1.0, 0.0, 0.0
])

# Initial obstacle register list
obs_regs_init = np.array([
        3.5, -3.0,
        9.0, -4.1,
        18.0, -2.0,
        19.5, 2.0,
        20.0, 8.1
])

dobs_init = np.zeros(obs_n) # Obstacles' dynamics

# Default Performance Indicators
# xe, ye, psi, u, r, ds
Qs_init = np.array([ 100.0, 70.0, 20.0, 1000.0, 10.0, 5.0 ])

# -------------------------------
# Set OCP
# -------------------------------
ocp = Ocp(T=Tf)

# Define states
x_state = ocp.state()
y_state = ocp.state()
psi_state = ocp.state()
obs = ocp.register_state(MX.sym("obs", obs_n))

# Define control inputs
u_cmd = ocp.control()
r_cmd = ocp.control()

# Initialize variables
ocp.set_initial(x_state,initial_x)
ocp.set_initial(y_state,initial_y)
ocp.set_initial(psi_state,initial_psi)
ocp.set_initial(obs, np.array([100.]*obs_n))

ocp.set_initial(u_cmd,0.0)
ocp.set_initial(r_cmd,0.0)

# Define initial states parameter
initial_states_X = np.zeros(nx)
initial_states_X[:3] = [initial_x,initial_y,initial_psi]
X_0 = ocp.register_parameter(MX.sym("X_0", nx))

# Define target parameter
tg = ocp.register_parameter(MX.sym("target", 5))
ocp.set_value(tg, initial_target)

# Define obstacles' dynamics parameter
dobs = ocp.register_parameter(MX.sym("dobs", obs_n))
ocp.set_value(dobs, dobs_init)

# Define obstacles' postions parameter
Qs = ocp.register_parameter(MX.sym("qs", 6))
ocp.set_value(Qs, Qs_init)

# ODE Variables
# Desired final psi
psi_d_final = tg[4]

# Virtual arm's length
l_dist = 0.0

# Virtual point's position
x_ = x_state + l_dist * cos(psi_state)
y_ = y_state + l_dist * sin(psi_state)

# Cross-track and along-track errors
dx = tg[2] - tg[0]
dy = tg[3] - tg[1]
gamma_p = atan2(dy + 1e-6, dx + 1e-6)

ye = -(x_-tg[2])*sin(gamma_p)+(y_-tg[3])*cos(gamma_p)
xe =  (x_-tg[2])*cos(gamma_p)+(y_-tg[3])*sin(gamma_p)

# Define ODE Equations
ocp.set_der(x_state, u_cmd * cos(psi_state))
ocp.set_der(y_state, u_cmd * sin(psi_state))
ocp.set_der(psi_state, r_cmd)
ocp.set_der(obs, dobs)

# Update PI's (Performance Indicators) weights
Qxe     = Qs[0]
Qye     = Qs[1]
Qpsi    = Qs[2]
Qu      = Qs[3]
Qr      = Qs[4]
Qds     = Qs[5]

# Path following objectives  
ocp.add_objective(ocp.sum  (Qye*((ye)**2) + 
                            Qxe*(xe)**2))
ocp.add_objective(ocp.at_tf(Qye*((ye)**2) + 
                            Qxe*(xe)**2))


# Control constraints
ocp.subject_to( ( -1.0 <= u_cmd ) <= 5.0 )
ocp.subject_to( (-20.0 <= r_cmd) <= 20.0 )

# Obstacle Avoidance objectives
l_list = [
  [0., 0.],
        #   [0., -0.4], [0., 0.4],
          # [0.55,-0.3], [0.55,0.3],
        #   [0.55,-0.2],[0.55,0.2],
          # [-0.35,-0.3], [-0.35,0.3],
        #   [0.55,0.]
        # [0.55,0.]
          ]
for i in range((int)(obs_n/2)):
    for l in l_list:
        x_virt = x_state + l[0]*cos(psi_state) - l[1]*sin(psi_state)
        y_virt = y_state + l[0]*sin(psi_state) + l[1]*cos(psi_state)
        obs_cost = Qds/((sqrt((obs[i*2]-x_virt)**2 + (obs[i*2+1]-y_virt)**2) / 3.)**1.5)
        ocp.add_objective(ocp.sum( obs_cost ))
        ocp.add_objective(ocp.at_tf( obs_cost ))
        if(i==0):
            obs_cost_sample = obs_cost

# Initial constraints
X = vertcat(x_state,y_state,psi_state,
  obs[0], obs[1], obs[2], obs[3], obs[4], 
  obs[5]
  , obs[6], obs[7], obs[8], obs[9]
  )
ocp.subject_to(ocp.at_t0(X)==X_0)

# Pick a solution method
options = {"ipopt": {"print_level": 5}}
options["expand"] = True
options["print_time"] = True
ocp.solver('ipopt',options)

# Make it concrete for this ocp
ocp.method(MultipleShooting(N=Nhor,M=1,intg='rk'))

# -------------------------------
# Solve the OCP wrt a parameter value (for the first time)
# -------------------------------
# Set initial value for parameters
ocp.set_value(X_0, initial_states_X)
# Solve
#sol = ocp.solve()

Sim_asv_dyn = ocp._method.discrete_system(ocp)
# Code Gen:
code_gen = True
if code_gen:
    ocp.method(external_method("fatrop", N=Nhor, intg="rk"))
    ocp._method.set_name("/code_gen")
    ocp._method.add_sampler("xe",xe)
    ocp._method.add_sampler("ye",ye)
    ocp._method.add_sampler("psie",sqrt((sin(psi_state)-sin(gamma_p))**2 + (cos(psi_state)-cos(gamma_p))**2))
    ocp._method.add_sampler("gamma_p",gamma_p)
    ocp._method.add_sampler("obs_cost",obs_cost_sample)
    ocp._method.add_sampler("n_horizon",Nhor)

sol = ocp.solve()