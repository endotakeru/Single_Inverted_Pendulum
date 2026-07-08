import numpy as np
import matplotlib.pyplot as plt

def plot_results(res, p, title="Inverted Pendulum - Closed-Loop Response"):
    fig, ax = plt.subplots(4, 1, figsize=(9,9), sharex=True)
    fig.suptitle(title, fontsize=13, weight="bold")

    # Theta w/ Safety Trip Limits
    ax[0].plot(res.t, np.rad2deg(res.theta), color="tab:blue") # tab: blue = tableau: blue
    for lim in (p.max_theta_deg, -p.max_theta_deg):
        ax[0].axhline(lim, color="tab:red", ls="--", lw=1) # horizontal line
    ax[0].set_ylabel("theta [deg]")

    # X (Pos) w/ Rail End Stops
    ax[1].plot(res.t, res.x, color="tab:green")
    for lim in (p.max_x, p.min_x):
        ax[1].axhline(lim, color="tab:red", ls="--", lw=1)
    ax[1].set_ylabel("x [m]")

    # Force w/ Saturation Limits
    ax[2].plot(res.t, res.F, color="tab:purple")
    for lim in (p.max_cart_force, -p.max_cart_force):
        ax[2].axhline(lim, color="tab:orange", ls="--", lw=1)
    ax[2].set_ylabel("F [N]")

    # Velocity
    ax[3].plot(res.t, np.rad2deg(res.theta_dot), color="tab:blue", label="theta_dot [deg/s]")
    ax[3].plot(res.t, res.x_dot, color="tab:green", label="x_dot [m/s]")
    ax[3].set_ylabel("rates")
    ax[3].set_xlabel("time [s]")
    ax[3].legend(loc="upper right", fontsize=8)

    # Mark the safety trip (if exist) on panels
    if res.trip_time is not None:
        for a in ax:
            a.axvline(res.trip_time, color="k", ls=":", lw=1) # vertical line
        ax[0].annotate("trip", (res.trip_time, p.max_theta_deg), fontsize=8, ha="left", va="top")

    # Final Graph Formatting
    for a in ax:
        a.grid(alpha=0.3) # gridline opacity
    fig.tight_layout(rect=(0,0,1,0.97)) # adjust graph spacing (left, right, bottom, top) to be used
    return fig

# runs : list of (theta0_deg, Results) - one entry per starting angle
# plots a single graph with results from sweepiong
def plot_sweep(runs, p, title="STarting-angle sweep"):
    fig, ax = plt.subplots(2, 1, figsize = (9,8), sharex=True)
    fig.suptitle(title, fontsize=13, weight="bold")

    # res = results 
    # Adding one curve per starting angle (theta & x)
    for i, (a, res) in enumerate(runs):
        color = f"C{i}"
        label = f"theta0 = {a:+d} deg"
        ax[0].plot(res.t, np.rad2deg(res.theta), color=color, label=label) # theta
        ax[1].plot(res.t, res.x*100.0, color=color, label=label) # x

    # Top graph formatting (theta) w/ safety-trip limits
    for lim in (p.max_theta_deg, -p.max_theta_deg):
        ax[0].axhline(lim, color="0.6", ls="--", lw=1)
    ax[0].axhline(0, color="0.8", lw=0.8)
    ax[0].set_ylabel("theta [deg]")
    ax[0].set_title("Pendulum angle", fontsize=10)
    ax[0].legend(loc="upper right", fontsize=8, ncol=2)

    # X (Pos) w/ Rail End Stops
    for lim in (p.max_x*100.0, p.min_x*100.0):
        ax[1].axhline(lim, color="0.6", ls="--", lw=1)
    ax[1].axhline(0, color="0.8", lw=0.8)
    ax[1].set_ylabel("x [cm]")
    ax[1].set_xlabel("time [s]") #same x label used for theta
    ax[1].set_title("Cart position", fontsize=10)
    ax[1].legend(loc="upper right", fontsize=8, ncol=2)

    for a in ax:
        a.grid(alpha=0.3)
    fig.tight_layout(rect=(0, 0, 1, 0.96))
    return fig