from dataclasses import dataclass
import numpy as np

@dataclass # all the varibale without (ClassVar[]) becomes a instance attribute
class Params:
    # Cart + Pendulum Specs and Gravity
    g: float = 9.81
    M: float = 0.220 # [kg]
    m1: float = 0.042
    m2: float = 0.018
    l: float = 0.250 # [m]

    # Largest horizontal output force from motor
    max_cart_force: float = 60 # tentative

    # Rail and Cart Length
    rail_l: float = 0.540
    cart_l: float = 0.112

    # PD Gains
    kp_theta: float = 1.0
    kd_theta: float = 0.0

    kp_x: float = 0.0
    kd_x: float = 0.0

    # Simulation time step and stop time
    Ts: float = 0.005
    Tstop: float = 10.0

    # Initial Conditions
    theta0_deg: float = 5.0 
    x0: float = 0.0
    x_dot0: float = 0.0
    theta_dot0: float = 0.0

    # Safety Limit
    max_theta_deg: float = 15.0

    # Derived A,B,C,Delta values from state-space model
    @property
    def A(self):
        return self.M + self.m1 + self.m2
    
    @property
    def B(self):
        return self.l * (self.m1 + self.m2/2)
    
    @property
    def C(self):
        return self.l**2 * (self.m1 + self.m2/3)
    
    @property
    def Delta(self):
        return self.A * self.C - self.B**2
    
    @property
    def max_x(self):
        return (self.rail_l - self.cart_l)/2
    
    @property
    def min_x(self):
        return -self.max_x

    @property
    def max_theta_rad(self):
        return np.deg2rad(self.max_theta_deg)
    
    @property
    def theta0(self):
        return np.deg2rad(self.theta0_deg)
    
    @classmethod
    def balanced(cls, **kwargs): # (these variables to be used as balanced values)
        return cls(kp_theta=9.17, kd_theta=0.75, kp_x=1, kd_x=1.5, **kwargs)