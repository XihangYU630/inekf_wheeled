import numpy as np
from scipy.linalg import block_diag, expm
from helper_func import skew, wedge
import pandas as pd


class Right_IEKF:
    def __init__(self, system):
        # Right_IEKF Construct an instance of this class
        #
        # Input:
        #   system:     system and noise models
        self.R = system.R  # error dynamics matrix
        self.v = system.v  # process model
        self.p = system.p  # measurement error matrix
        self.b_w = system.b_w  # input noise covariance
        self.b_a = system.b_a  # measurement noise covariance
        self.R_c = system.R_c  # state vector
        self.p_c = system.p_c  # state covariance

        self.error_estimated = system.error_estimated

        self.P = system.P

    def Ad(self):
        Ad_x = np.zeros((9, 9))
        Ad_x[0:3, 0:3] = self.R
        Ad_x[3:6, 0:3] = skew(self.v) @ self.R
        Ad_x[3:6, 3:6] = self.R
        Ad_x[6:9, 0:3] = skew(self.p) @ self.R
        Ad_x[6:9, 6:9] = self.R
        return Ad_x

    def compute_A(self, sys):
        A = np.zeros((21, 21))
        A[3:6, 0:3] = skew(sys.g)
        A[6:9, 3:6] = np.eye(3)
        A[0:3, 9:12] = -self.R
        A[3:6, 9:12] = -skew(self.v) @ self.R
        A[3:6, 12:15] = -self.R
        A[6:9, 9:12] = -skew(self.p) @ self.R
        return A

    def compute_B(self):
        B = np.zeros((21, 21))
        B[0:9, 0:9] = self.Ad()
        B[9:21, 9:21] = np.eye(12)
        return B

    def propagation_state_and_error(self, system):
        # w here is the estimated 
        R = self.R
        v = self.v
        p = self.p
        b_w = self.b_w
        b_a = self.b_a


        self.R = R @ expm(skew(system.w - b_w)*system.dt)
        self.v = v + R @ (system.a - b_a) * system.dt + system.g * system.dt
        self.p = p + v * system.dt + 0.5 * R @ (system.a - b_a) * system.dt * system.dt + 0.5 * system.g * system.dt * system.dt

        A = self.compute_A(system)
        self.error_estimated = self.error_estimated @ expm(A * system.dt)

    def propagation_covariance(self, system):
        PHI_k = expm(self.compute_A(system) * system.dt) # 21x21
        # print("PHI_k: ", np.shape(PHI_k))
        Q = self.compute_B() @ system.cov_w @ self.compute_B().T
        # print("system.cov_w: ", np.shape(system.cov_w))
        Q_k = PHI_k @ Q @ PHI_k.T * system.dt
        # print("Q_k: ", np.shape(Q_k))
        self.P = PHI_k @ self.P @ PHI_k.T + Q_k
    
    def compute_H(self):
        H = np.zeros((3, 21))
        H[0:3, 3:6] = np.eye(3)
        return H

    def compute_PI(self):
        PI = np.zeros((3, 5))
        PI[0:3, 0:3] = np.eye(3)
        return PI
    def compute_X(self):
        X = np.zeros((5, 5))
        X[0:3, 0:3] = self.R
        X[0:3, 3] = self.v
        X[0:3, 4] = self.P
        X[3, 3] = 1
        X[4, 4] = 1

    def measurement_model_encoder(self, sys):
        H = self.compute_H()
        N = self.R @ sys.nf_cov @ self.R.T
        S = H @ self.P @ H.T + N
        K = self.P @ H.T @ np.linalg.inv(S)
        PI = self.compute_PI()
        b = np.array([0, 0, 0, -1, 0])
        delta = K @ PI @ (self.compute_X() @ sys.y_encoder - b)
        delta_IMU = delta[0:9]
        delta_bw = delta[9:12]
        delta_ba = delta[12:15]
        delta_Rc = delta[15:18]
        delta_pc = delta[18:21]
        X = expm(wedge(delta_IMU)) @ self.compute_X()
        self.b_w = self.b_w + delta_bw
        self.b_a = self.b_a + delta_ba
        self.R_c = expm(skew(delta_Rc)) @ self.R_c
        self.p_c = self.p_c + delta_pc
        self.P = (np.eye(21) - K @ H) @ self.P @ (np.eye(21) - K @ H).T + K @ N @ K.T
        self.R = X[0:3, 0:3]
        self.v = X[0:3, 3]
        self.p = X[0:3, 4]
    
    def measurement_model_camera(self, sys):
        v_c_estimated = self.R_c.T @ self.R.T @ self.v + skew(sys.w_c_observation) @ self.R_c.T @ self.p_c
        inno_error = v_c_estimated - sys.v_c_observation
        ## Jacobina matrix H and G
        H = np.zeros((3, 21))
        H[:, 3:6] = self.R_c.T @ self.R.T
        H[:, 15:18] = -skew(self.R_c @ self.R.T @ self.v)-skew(sys.w_c_observation) @ skew(self.R_c.T @ self.p_c)
        H[:, 18:21] = skew(sys.w_c_observation) @ self.R_c.T
        G = np.zeros((3, 6))
        G[:, 0:3] = -skew(self.R_c.T @ self.p_c)
        G[:, 3:6] = -np.eye(3)
        ##
        S = H @ self.P @ H.T + G @ sys.N_camera @ G.T
        K = self.P @ H.T @ np.linalg.inv(S)
        error = self.error_estimated + K @ inno_error
        self.P = (np.eye(21) - K @ H) @ self.P

        error_R = error[0:3]
        error_v = error[3:6]
        error_p = error[6:9]
        error_b_w = error[9:12]
        error_b_a = error[12:15]
        error_R_c = error[15:18]
        error_p_c = error[18:21]

        R = self.R
        self.R = self.R @ expm(skew(error_R))
        self.v = self.R @ R.T @ (self.v - error_v)
        self.p = self.R @ R.T @ (self.p - error_p)
        self.b_w = self.b_w - error_b_w
        self.b_a = self.b_a - error_b_a
        self.R_c = self.R_c @ expm(skew(error_R_c))
        self.p_c = self.p_c - error_p_c





