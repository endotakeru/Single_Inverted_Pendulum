from dataclasses import dataclass
import numpy as np
from scipy.integrate import solve_ivp

from dynamics import state_derivative

@dataclass
class Results:
    t: np.ndarray
    x: np.ndarray
    x_dot: np.ndarray
    theta: np.ndarray
    theta_dot: np.ndarray
    F: np.ndarray
    saturated: np.ndarray
    trip_time: float = None # time of the safety trip
    rail_hits: int = 0

def simulate(p, controller):
    n = int(round(p.Tstop / p.Ts)) + 1 # simulation length
    t = np.zeros(n)
    X = np.zeros((4,n)) # 4 rows = [x, x_dot, theta, theta_dot]
    F = np.zeros(n)
    sat = np.zeros(n, dtype=bool)

    # initial:
    y = np.array([p.x0, p.x_dot0, p.theta0, p.theta_dot0], dtype=float) #state_derivative0
    X[:,0] = y # first row
    controller.reset(theta0=p.theta0, x0=p.x0) # function in CartPolePD class
    tripped, trip_time, rail_hits = False, None, 0

    # simulation loop:
    for k in range(1, n):
        # Record new theta
        theta_meas = y[2]

        # Calculate/Update PD Output Force (u) and Motor Saturation Bool (s) if pendulum not tripped
        if tripped:
            u, s = 0.0, False
        else:
            u, s = controller.compute(theta_meas, y[0], p.Ts)

        # Update y using state_derivative(args = t, y, F, p):
        sol = solve_ivp(state_derivative, (0.0, p.Ts), y, args=(u,p), method="RK45", rtol=1e-7, atol=1e-9, t_eval=[p.Ts])
        y = sol.y[:,-1].copy() # last row of the calculated value as RK45 integrator produces multiple results at increments of Ts

        # When hit rail end: Clamp position, Get rid of velocity
        if y[0] > p.max_x: # positive direction
            y[0], y[1], rail_hits = p.max_x, min(0.0, y[1]), rail_hits+1
        if y[0] < p.min_x: # negative direction
            y[0], y[1], rail_hits = p.min_x, max(0.0, y[1]), rail_hits+1

        # When theta exceed limit (-15 ~ 15 deg):
        if not tripped and abs(y[2]) >= p.max_theta_rad:
            tripped, trip_time = True, k*p.Ts

        # Record data
        t[k], X[:, k], F[k], sat[k] = k*p.Ts, y, u, s
    
    # return: **time**, **x**, x_dot, **theta**, theta_dot, **F**, sat, trip_time, rail_hits
    return Results(t, X[0], X[1], X[2], X[3], F, sat, trip_time, rail_hits)