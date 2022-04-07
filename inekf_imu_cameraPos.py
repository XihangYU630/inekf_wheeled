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
        self.p = p + v * system.dt + 0.5 * R @ (system.a - b_a) * system.dt ** 2 + 0.5 * system.g * system.dt ** 2

        # A = self.compute_A(system)
        # self.error_estimated = self.error_estimated @ expm(A * system.dt)

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
        X[0:3, 4] = self.p
        X[3, 3] = 1
        X[4, 4] = 1
        return X

    def measurement_model_encoder(self, sys):
        H = self.compute_H()
        N = self.R @ sys.nf_cov @ self.R.T
        S = H @ self.P @ H.T + N
        K = self.P @ H.T @ np.linalg.inv(S)
        PI = self.compute_PI()
        b = np.array([0, 0, 0, -1, 0])

        delta_1 = K @ PI @ (self.compute_X() @ sys.y_encoder_1 - b)

        delta_IMU = delta_1[0:9]
        delta_bw = delta_1[9:12]
        delta_ba = delta_1[12:15]
        delta_Rc = delta_1[15:18]
        delta_pc = delta_1[18:21]

        X = expm(wedge(delta_IMU)) @ self.compute_X()
        self.b_w = self.b_w + delta_bw
        self.b_a = self.b_a + delta_ba
        self.R_c = expm(skew(delta_Rc)) @ self.R_c
        self.p_c = self.p_c + delta_pc
        self.P = (np.eye(21) - K @ H) @ self.P @ (np.eye(21) - K @ H).T + K @ N @ K.T
        self.R = X[0:3, 0:3]
        self.v = X[0:3, 3]
        self.p = X[0:3, 4]

        delta_2 = K @ PI @ (self.compute_X() @ sys.y_encoder_2 - b)

        delta_IMU = delta_2[0:9]
        delta_bw = delta_2[9:12]
        delta_ba = delta_2[12:15]
        delta_Rc = delta_2[15:18]
        delta_pc = delta_2[18:21]
        X = expm(wedge(delta_IMU)) @ self.compute_X()
        self.b_w = self.b_w + delta_bw
        self.b_a = self.b_a + delta_ba
        self.R_c = expm(skew(delta_Rc)) @ self.R_c
        self.p_c = self.p_c + delta_pc
        self.P = (np.eye(21) - K @ H) @ self.P @ (np.eye(21) - K @ H).T + K @ N @ K.T
        self.R = X[0:3, 0:3]
        self.v = X[0:3, 3]
        self.p = X[0:3, 4]

    def measurement_model_position(self, sys):
        H = np.zeros((3, 9))
        H[0:3, 6:9] = np.eye(3)
        N = np.linalg.inv(self.R) @ sys.N_pos @ np.linalg.inv(self.R.T) ## nf_cov is the noise for measurements of position

        P_l = np.linalg.inv(self.Ad()) @ self.P[0:9, 0:9] @ np.linalg.inv(self.Ad().T)

        S = H @ P_l @ H.T + N ## S is 3x3

        K = P_l @ H.T @ np.linalg.inv(S) ## 9x3
        b = np.array([0, 0, 0, 0, 1]) ## b is 5x1
        print("sys.y_pos: ", sys.y_pos)
        print("self.compute_X()) @ sys.y_pos: ", np.linalg.inv(self.compute_X()) @ sys.y_pos)
        delta = K @ (np.linalg.inv(self.compute_X()) @ sys.y_pos - b)[0:3] ## delta is 9x1

        X = self.compute_X() @ expm(wedge(delta))
        self.R = X[0:3, 0:3]
        print("self.R @ self.R.T: ", self.R @ self.R.T)

        self.v = X[0:3, 3]
        self.p = X[0:3, 4]

        P_l = (np.eye(9) - K @ H) @ P_l  @ (np.eye(9) - K @ H).T + K @ N @ K.T
        self.P[0:9, 0:9] = self.Ad() @ P_l @ self.Ad().T

    def measurement_model_camera(self, sys):
        v_c_estimated = self.R_c.T @ self.R.T @ self.v + skew(sys.w_c_observation) @ self.R_c.T @ self.p_c
        print("v_c_estimated: ", v_c_estimated)
        print("sys.v_c_observation: ", sys.v_c_observation)
        inno_error = v_c_estimated - sys.v_c_observation
        ## Jacobina matrix H and G
        H = np.zeros((3, 21))
        H[:, 3:6] = -self.R_c.T @ self.R.T
        H[:, 15:18] = -self.R_c.T @ skew(self.R.T @ self.v)-skew(-sys.w_c_observation) @ self.R_c.T @ skew(self.p_c)
        print("sys.w_c_observation: ", )
        H[:, 18:21] = -skew(sys.w_c_observation) @ self.R_c.T
        # G = np.zeros((3, 6))
        # G[:, 0:3] = -skew(self.R_c.T @ self.p_c)
        # G[:, 3:6] = -np.eye(3)
        ##

        N = sys.N_camera[0:3, 0:3] + skew(self.R_c.T @ self.p_c) * sys.N_camera[3:6, 3:6] @ skew(self.R_c.T @ self.p_c).T
        N = self.R @ self.R_c @ N @ (self.R @ self.R_c).T

        S = H @ self.P @ H.T + N  ## S is different
        print("H: ", H)
        # S = H @ self.P @ H.T + G @ sys.N_camera @ G.T
        K = self.P @ H.T @ np.linalg.inv(S)
        error = K @ inno_error

        self.P = (np.eye(21) - K @ H) @ self.P
        # self.P = (np.eye(21) - K @ H) @ self.P @ (np.eye(21) - K @ H).T + K @ G @ sys.N_camera @ G.T @ K.T
        print("error: ", error)
        delta_IMU = error[0:9]
        delta_bw = error[9:12]
        delta_ba = error[12:15]
        delta_Rc = error[15:18]
        delta_pc = error[18:21]
        print("delta_pc: ", delta_pc)
        X = expm(wedge(delta_IMU)) @ self.compute_X()
        self.b_w = self.b_w + delta_bw
        self.b_a = self.b_a + delta_ba
        self.R_c = expm(skew(delta_Rc)) @ self.R_c
        self.p_c = self.p_c + delta_pc
        self.R = X[0:3, 0:3]
        self.v = X[0:3, 3]
        self.p = X[0:3, 4]

