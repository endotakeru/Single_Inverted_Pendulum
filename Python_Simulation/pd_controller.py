class PD:
    def __init__(self, kp, kd, tau_d=0.02): # initialization
        self.kp, self.kd, self.tau_d = kp, kd, tau_d
        self.reset(0.0)

    def reset(self, meas=0.0): # clear values before a run
        self.deriv = 0.0
        self.prev = meas # measured initial value (theta or x)

    def output(self, meas, dt, setpoint=0.0): # updates error and deriv
        err = meas - setpoint
        raw_deriv = (meas - self.prev)/dt #raw deriv
        self.deriv += (raw_deriv - self.deriv)*(dt/(self.tau_d+dt)) # deriv noise cancellation using formula
        self.prev = meas
        return self.kp*err + self.kd*self.deriv

class CartPolePD:
    def __init__(self, p):
        self.p = p
        self.angle = PD(p.kp_theta, p.kd_theta)
        self.pos = PD(p.kp_x, p.kd_x)

    def reset(self, theta0=0.0, x0=0.0):
        self.angle.reset(theta0)
        self.pos.reset(x0)

    def compute(self, theta_meas, x_meas, dt):
    # update output using PD
        F = self.angle.output(theta_meas, dt) + self.pos.output(x_meas, dt)

        Fmax = self.p.max_cart_force
        Fsat = max(-Fmax, min(Fmax, F)) # Clamp Output/Force
        saturated = abs(Fsat-F) > 1e-9 # is motor saturated?
        return Fsat, saturated
