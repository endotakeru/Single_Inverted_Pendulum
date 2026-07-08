import numpy as np

def state_space_matrices(p):
    A, B, C, Delta, g = p.A, p.B, p.C, p.Delta, p.g

    A_mat = np.array([[0, 1, 0, 0],
                      [0, 0, -(B**2 * g) / Delta, 0],
                      [0, 0, 0, 1],
                      [0, 0, (A * B * g) / Delta, 0]])
    B_mat = np.array([[0.0],
                      [C / Delta],
                      [0.0],
                      [-B / Delta]])         
    C_mat = np.array([[1, 0, 0, 0],  # x   
                      [0, 0, 1, 0]]) # theta
    D_mat = np.zeros((2, 1)) # y = C_mat*x + D_mat*u (x = [x, x', theta, theta'], u = Force)
    return A_mat, B_mat, C_mat, D_mat

def open_loop_poles(p):
    # Eigenvalues of A_mat = open-loop poles of uncontrolled system, indicate if system = stable.
    A_mat = state_space_matrices(p)[0]
    return np.linalg.eigvals(A_mat)