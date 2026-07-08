import argparse # so that you can run sims from terminal
import numpy as np
import matplotlib.pyplot as plt

from params import Params
from linear_model import open_loop_poles
from pd_controller import CartPolePD
from simulation import simulate
from plot_results import plot_results, plot_sweep

# Sweep Mode:
def run_sweep(args): # sweep through angles [-15, -10, -5, 0, 5, 10, 15]
    angles = angles = list(range(-15, 16, 5))
    print(f"sweeping theta0 = {angles} deg   gains = {args.gains}")
    runs = []
    p_ref = None
    # Sweep through diff theta0 w/ same gains for every run
    for a in angles:
        p = Params.balanced(theta0_deg=a) if args.gains == "preset" else Params(theta0_deg=a) # if user type 'preset' run 'pre-tuned' gains
        p_ref = p
        controller = CartPolePD(p) # Create CartPolePD class object. Basically doing CartPolePD.__init__(p)
        res = simulate(p, controller) # run simulation and return object 'Results'

        # Print Results
        peak = np.max(np.abs(np.rad2deg(res.theta)))
        status = f"TRIPPED at t = {res.trip_time: .2f} s" if res.trip_time else "stayed upright"
        print(f" theta0 = {a:+3d} deg -> peak |theta| = {peak:6.2f} deg, "
              f"final x = {res.x[-1]*100:+6.2f} cm, {status}")
        runs.append((a,res)) # add results to runs
        
    # Plot Results (just theta and x)
    plot_sweep(runs, p_ref, title=f"Starting-angle sweep  (gains: {args.gains})") # plot sweep graph
    plt.show()

def main():
    parser = argparse.ArgumentParser(description="Single inverted pendulum cart-pole simulator")
    parser.add_argument("--gains", choices=["preset","spec"], default="preset", metavar='',
                        help="'preset' = tuned balancing gains, 'spec' = the listed defaults")
    parser.add_argument("--sweep", action="store_true", 
                        help="sweep starting angle -15..+15 deg and plot theta & x over t for each")
    args = parser.parse_args()

    # Run Sweep Mode if true:
    if args.sweep:
        run_sweep(args)
        return
    
    # Run Deault Mode else:
    p = Params.balanced() if args.gains == "preset" else Params()

    # Print Model Description
    print(f"total mass A = {p.A:.3f} kg   B = {p.B:.5f}   C = {p.C:.5f}   Delta = {p.Delta:.6f}")
    print(f"max cart force = {p.max_cart_force:.2f} N   rail x in [{p.min_x:+.3f}, {p.max_x:+.3f}] m")
    print(f"open-loop poles = {np.round(open_loop_poles(p), 3)}  (having one ~ +7 rad/s -> unstable)")
    print(f"min Kp_theta to balance = A*g = {p.A*p.g:.2f} N/rad   using Kp_theta = {p.kp_theta}")
    print(f"gains = {args.gains}")

    # Get PD Output + Result from Simulation
    controller = CartPolePD(p)
    res = simulate(p, controller)

    # Print Results
    peak = np.max(np.abs(np.rad2deg(res.theta)))
    status = f"TRIPPED at t = {res.trip_time:.2f} s" if res.trip_time else "stayed upright"
    print(f"\npeak |theta| = {peak:6.2f} def   final x = {res.x[-1]*100:.2f} cm   "
          f"rail hits = {res.rail_hits}   -> {status}")
    
    # Plot Results
    plot_results(res, p)
    plt.show()

# If file is run directly, run main()
if __name__ == "__main__":
    main()
    