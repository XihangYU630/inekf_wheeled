
import numpy as np
from scipy.linalg import block_diag, expm
import helper_func
import pandas as pd
import inekf_imu_camera



# state variable: (R, v, p, b_w, b_a, R_c, p_c)

# global variable
g = np.array([0, 0, -9.8067])
r1 = 
r2 = 

dt = 0.1 # this should be retrieved from data
w = np.array([0.1, 0.1, 0.1]) # this should be retrieved from data
a = np.array([0.1, 0.1, 0.1]) # this should be retrieved from data
y = np.array([0.1, 0.1, 0.1, -1, 0]) # this is the encoder data for left wheel
v_c_observation = np.array([1, 1, 1]) # this is camera data for velocity
w_c_observation = np.array([1, 1, 1]) # this is camera data for angular velocity
error_estimated = np.array([0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1,
                            0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1])

# data processing
Camera_vdom = pd.read_csv("Camera_data.csv")

Camera_vdom.iloc[:, 0]
Encoder = pd.read_csv('Encoder_velocities.csv')
Sec_Enc = Encoder.iloc[:, 0]
Sec_Nano_Enc = Encoder.iloc[:, 1]
Sec_Enc_total = Sec_Enc + Sec_Nano_Enc
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



class System:
    def __init__(self):

        #initial guess of state variables
        self.R = np.eyes(3)  # error dynamics matrix
        self.v = np.array([0, 0, 0])  # process model
        self.p = np.array([67, -14, 8])  # measurement error matrix
        self.b_w = np.array([0, 0, 0])  # input noise covariance
        self.b_a = np.array([0, 0, 0])  # measurement noise covariance
        self.R_c = np.eyes(3)  # state vector
        self.p_c = np.array([0, 0, 0])  # state covariance
        self.error_estimated = error_estimated
        self.P = np.eyes(3)


def main():

    system = System()
    husky = Right_IEKF(system)
    # number of steps
    n = 1000
    for i in range(n):
        # updata dt, g, a, w
        system.updata()

        husky.propagation_state_and_error(system)

        husky.propagation_covariance()
        husky.measurement_model_encoder()
        husky.measurement_model_camera()

if __name__ == "__main__":
    main()