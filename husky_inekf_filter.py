
import numpy as np
from scipy.linalg import block_diag, expm
import helper_func
import pandas as pd
# import inekf_imu_camera

# state variable: (R, v, p, b_w, b_a, R_c, p_c)

# global variable
# g = np.array([0, 0, -9.8067])
# r1 = 
# r2 = 

dt = 0.1 # this should be retrieved from data
w = np.array([0.1, 0.1, 0.1]) # this should be retrieved from data
a = np.array([0.1, 0.1, 0.1]) # this should be retrieved from data
y = np.array([0.1, 0.1, 0.1, -1, 0]) # this is the encoder data for left wheel
v_c_observation = np.array([1, 1, 1]) # this is camera data for velocity
w_c_observation = np.array([1, 1, 1]) # this is camera data for angular velocity
error_estimated = np.array([0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1,
                            0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1])

# data processing
Visual_dom = pd.read_csv("Visual_odometry_v_w.csv")
Sec = Visual_dom.iloc[50:100, 0]

print("Sec: ", Sec)

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

def find_nearest(array, value):
    array = np.asarray(array)
    idx = (np.abs(array - value)).argmin()
    return idx, array[idx]

class System:
    def __init__(self):

        #initial guess of state variables
        self.R = np.eyes(3)  # error dynamics matrix
        self.v = np.array([0.1, 0.1, 0.1])  # process model
        self.p = np.array([67, -14, 8])  # measurement error matrix
        self.b_w = np.array([0.1, 0.1, 0.1])  # input noise covariance
        self.b_a = np.array([0.1, 0.1, 0.1])  # measurement noise covariance
        self.R_c = np.eyes(3)  # state vector
        self.p_c = np.array([0.1, 0.1, 0.1])  # state covariance
        self.error_estimated = np.array([0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1,
                            0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1])
        self.P = np.eyes(3)

        # hard code calibration
        g = np.array([0, 0, -9.8067])
        ## vector of wheel radius
        r1 = np.array([0, 0, -0.5])
        ## vector of distance between two wheels
        r2 = np.array([1, 0, 0])
    
    def update(self, i):
        # update dt, a, w
        self.dt = Sec_IMU[i] + Sec_Nano_IMU[i] * 10 ** (-9) - Sec_IMU[i-1] + Sec_Nano_IMU[i-1] * 10 ** (-9)
        self.a = np.array([a_x_IMU[i], a_y_IMU[i], a_z_IMU[i]])
        self.w = np.array([w_x_IMU[i], w_y_IMU[i], w_z_IMU[i]])
    
    def choose_measurement(self, i):
        t = Sec_IMU[i] + Sec_Nano_IMU[i] * 10 ** (-9)
        idx_1, t1 = find_nearest(Sec_Enc_total, t)
        idx_2, t2 = find_nearest(Sec_Cameta_total, t)
        if np.abs(t1 - t) < np.abs(t2 - t):
            return "encoder", idx_1
        else:
            return "camera", idx_2


def main():

    system = System()
    husky = Right_IEKF(system)
    # number of steps
    for i in range(1, len(Sec_IMU)):
        # updata dt, g, a, w
        system.updata(i)

        # propagate between steps
        husky.propagation_state_and_error(system)
        husky.propagation_covariance(system)

        # correction step
        ## check the nearest measurement
        flag, idx = system.choose_measurement(i)
        ## the nearest time stamp is encoder
        if(flag == "encoder"):
            husky.measurement_model_encoder()
        else:
            husky.measurement_model_camera()

if __name__ == "__main__":
    main()