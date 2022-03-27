
import numpy as np
from scipy.linalg import block_diag, expm
import helper_func
import pandas as pd
import inekf_imu_camera


# data processing
Visual_dom = pd.read_csv("Visual_odometry_v_w.csv")
Sec_visdom = Visual_dom.iloc[:, 0]
Sec_Nano_visdom = Visual_dom.iloc[:, 1]
Sec_visdom_total = Sec_visdom + Sec_Nano_visdom
# print("type(Sec_visdom_total): ", type(Sec_visdom_total))
v_x_visdom = Visual_dom.iloc[:, 2]
v_y_visdom = Visual_dom.iloc[:, 3]
v_z_visdom = Visual_dom.iloc[:, 4]
w_x_visdom = Visual_dom.iloc[:, 5]
w_y_visdom = Visual_dom.iloc[:, 6]
w_z_visdom = Visual_dom.iloc[:, 7]

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
        self.R = np.eye(3)  # error dynamics matrix
        self.v = np.array([0.1, 0.1, 0.1])  # process model
        self.p = np.array([67, -14, 8])  # measurement error matrix
        self.b_w = np.array([0.1, 0.1, 0.1])  # input noise covariance
        self.b_a = np.array([0.1, 0.1, 0.1])  # measurement noise covariance
        self.R_c = np.eye(3)  # state vector
        self.p_c = np.array([0.1, 0.1, 0.1])  # state covariance
        self.error_estimated = np.array([0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1,
                            0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1])
        self.P = np.eye(21)

        # hard code calibration
        self.g = np.array([0, 0, -9.8067])
        ## vector of wheel radius
        self.r1 = np.array([0, 0, -0.5])
        ## vector of distance between two wheels
        self.r2 = np.array([1, 0, 0])

        # hard coded noise covariance
        ## these parameters are used for tuning
        self.cov_w = np.eye(21) # Q is 3*3 matrix
        self.nf_cov = np.eye(3) # nf_cov is 3*3 matrix
        self.N_camera = np.eye(6) # N is covariance for camera data

    
    def update_IMU(self, i):
        # update dt, a, w
        self.dt = Sec_IMU[i] + Sec_Nano_IMU[i] * 10 ** (-9) - Sec_IMU[i-1] + Sec_Nano_IMU[i-1] * 10 ** (-9)
        self.a = np.array([a_x_IMU[i], a_y_IMU[i], a_z_IMU[i]])
        self.w = np.array([w_x_IMU[i], w_y_IMU[i], w_z_IMU[i]])
    
    def choose_measurement(self, i):
        t = Sec_IMU[i] + Sec_Nano_IMU[i] * 10 ** (-9)
        idx_1, t1 = find_nearest(Sec_Enc_total, t)
        idx_2, t2 = find_nearest(Sec_visdom_total, t)
        if np.abs(t1 - t) < np.abs(t2 - t):
            return "encoder", idx_1
        else:
            return "camera", idx_2
    def update_measurement(self, flag, idx):
        
        if(flag == "encoder"):
            w_l = np.array([-Left_wheel_ang[idx], 0, 0])
            w = np.array([w_x_IMU[idx], w_y_IMU[idx], w_z_IMU[idx]])
            y = helper_func.skew(w_l) @ self.r1 + 0.5 @ helper_func.skew(w) @ self.r2
            self.y_encoder = np.array([y[0], y[1], y[2], -1, 0])
        else:
            self.v_c_observation = np.array([v_x_visdom[idx], v_y_visdom[idx], v_z_visdom[idx]])
            self.w_c_observation = np.array([w_x_visdom[idx], w_y_visdom[idx], w_z_visdom[idx]])


def main():

    system = System()
    husky = inekf_imu_camera.Right_IEKF(system)
    # number of steps
    for i in range(1, len(Sec_IMU)):
        # updata dt, g, a, w
        system.update_IMU(i)

        # propagate between steps
        husky.propagation_state_and_error(system)
        husky.propagation_covariance(system)

        # correction step
        ## check the nearest measurement
        flag, idx = system.choose_measurement(i)
        
        # update measurement data
        system.update_measurement(flag, idx)
        ## the nearest time stamp is encoder
        if(flag == "encoder"):
            husky.measurement_model_encoder(system)
        else:
            husky.measurement_model_camera(system)
    
    plot()

if __name__ == "__main__":
    main()