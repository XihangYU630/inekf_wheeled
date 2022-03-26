import numpy as np
from scipy.linalg import block_diag, expm
from helper_func import skew, wedge
import pandas as pd

# state variable: (R, v, p, b_w, b_a, R_c, p_c)
g = np.array([0, 0, -9.8067])
dt = 0.1 # this should be retrieved from data
w = np.array([0.1, 0.1, 0.1]) # this should be retrieved from data
a = np.array([0.1, 0.1, 0.1]) # this should be retrieved from data
y = np.array([0.1, 0.1, 0.1, -1, 0]) # this is the encoder data for left wheel
v_c_observation = np.array([1, 1, 1]) # this is camera data for velocity
w_c_observation = np.array([1, 1, 1]) # this is camera data for angular velocity
error_estimated = np.array([0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1,
                            0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1])

# data processing
Encoder = pd.read_csv('Encoder_velocities.csv')
Sec_Enc = Encoder.iloc[:, 0]
Sec_Nano_Enc = Encoder.iloc[:, 1]
Left_wheel_ang = Encoder.iloc[:, 3]
Right_wheel_ang = Encoder.iloc[:, 4]
Filtered_imu = pd.read_csv('Filtered_imu_data.csv')
Sec_IMU = Filtered_imu.iloc[:, 0]
Sec_Nano_IMU = Filtered_imu.iloc[:, 1]
a_x_IMU = Filtered_imu.iloc[:, 3]
a_y_IMU = Filtered_imu.iloc[:, 4]
a_z_IMU = Filtered_imu.iloc[:, 5]
w_x_IMU = Filtered_imu.iloc[:, 6]
w_y_IMU = Filtered_imu.iloc[:, 7]
w_z_IMU = Filtered_imu.iloc[:, 8]

# initial guess of state variables
R = 
v
p
b_w
b_a
R_c
p_c





# initial noise variable
Q = np.eye(3) # Q is 3*3 matrix
nf_cov = np.eye(3) # nf_cov is 3*3 matrix
N = np.eye(6) # N is covariance for camera data

class Right_IEKF:
    def __init__(self, system):
        # Right_IEKF Construct an instance of this class
        #
        # Input:
        #   system:     system and noise models
        self.R = R  # error dynamics matrix
        self.v = v  # process model
        self.p = p  # measurement error matrix
        self.b_w = b_w  # input noise covariance
        self.b_a = b_a  # measurement noise covariance
        self.R_c = R_c  # state vector
        self.p_c = p_c  # state covariance

        self.error_estimated = error_estimated

        self.P = P

    def Ad(self):
        Ad_x = np.zeros((9, 9))
        Ad_x[0:3, 0:3] = self.R
        Ad_x[3:6, 0:3] = skew(self.v) @ self.R
        Ad_x[3:6, 3:6] = self.R
        Ad_x[6:9, 0:3] = skew(self.p) @ self.R
        Ad_x[6:9, 6:9] = self.R
        return Ad_x

    def compute_A(self):
        A = np.zeros((21, 21))
        A[3:6, 0:3] = skew(g)
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

    def propagation_state_and_error(self, system):
        # w here is the estimated 
        R = self.R
        v = self.v
        p = self.p
        b_w = self.b_w
        b_a = self.b_a

        self.R = R @ expm(skew(w - b_w)*system.dt)
        self.v = v + R @ (system.a - b_a) * system.dt + system.g * system.dt
        self.p = p + v * system.dt + 0.5 * R @ (system.a - b_a) * system.dt * system.dt + 0.5 * system.g * system.dt * system.dt

        A = self.compute_A()
        self.error_estimated = self.error_estimated @ expm(A * system.dt)

    def propagation_covariance(self, system):
        PHI_k = expm(self.compute_A() * system.dt)
        Q_k = PHI_k @ Q @ PHI_k.T * system.dt
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

    def measurement_model_encoder(self):
        H = self.compute_H()
        N = self.R @ nf_cov @ self.R.T
        S = H @ self.P @ H.T + N
        K = self.P @ H.T @ np.linalg.inv(S)
        PI = self.compute_PI()
        b = np.array([0, 0, 0, -1, 0])
        delta = K @ PI @ (self.compute_X() @ y - b)
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
    
    def measurement_model_camera(self):
        v_c_estimated = self.R_c.T @ self.R.T @ self.v + skew(self.w_c) @ self.R_c.T @ self.p_c
        inno_error = v_c_estimated - v_c_observation
        ## Jacobina matrix H and G
        H = np.zeros((3, 21))
        H[:, 3:6] = self.R_c.T @ self.R.T
        H[:, 15:18] = -skew(self.R_c @ self.R.T @ self.v)-skew(w_c_observation) @ skew(self.R_c.T @ self.p_c)
        H[:, 18:21] = skew(w_c_observation) @ self.R_c.T
        G = np.zeros((3, 6))
        G[:, 0:3] = -skew(self.R_c.T @ self.p_c)
        G[:, 3:6] = -np.eye(3)
        ##
        S = H @ self.P @ H.T + G @ N @ G.T
        K = self.P @ H.T @ np.linalg.inv(S)
        error = self.error_estimated + K @ inno_error
        self.P = (np.eye(3) - K @ H) @ self.P

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





