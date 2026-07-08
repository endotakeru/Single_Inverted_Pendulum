import numpy as np
#p = parameter

def accelerations(state, F, p):
    _, _, theta, theta_dot = state
    A, B, C, g = p.A, p.B, p.C, p.g
    sin, cos = np.sin(theta), np.cos(theta)

    # non-linearized (sin(theta) != theta, cos(theta) != 1)
    a = np.array([[A, B*cos], 
                  [B*cos, C]])
    b = np.array([F+B*sin*(theta_dot**2),
                  B*g*sin])
    x_ddot, theta_ddot = np.linalg.solve(a, b)
    return x_ddot, theta_ddot

def state_derivative(t, y, F, p): # t is used by solve_ivp later to solve increments in Ts
    x_ddot, theta_ddot = accelerations(y, F, p)
    return [y[1], x_ddot, y[3], theta_ddot]