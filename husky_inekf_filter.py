
import numpy as np
from scipy.linalg import block_diag, expm
import helper_func
import pandas as pd
import inekf_imu_camera
import matplotlib.pyplot as plt


# data processing
Visual_dom = pd.read_csv("Visual_odometry_v_w.csv")
Sec_visdom = Visual_dom.iloc[:, 0]
Sec_Nano_visdom = Visual_dom.iloc[:, 1]
Sec_visdom_total = Sec_visdom + Sec_Nano_visdom * 10 ** (-9)
Sec_visdom_total = Sec_visdom_total - Sec_visdom_total[0]
# print("type(Sec_visdom_total): ", type(Sec_visdom_total))
v_x_visdom = Visual_dom.iloc[:, 2]
v_y_visdom = Visual_dom.iloc[:, 3]
v_z_visdom = Visual_dom.iloc[:, 4]
w_x_visdom = Visual_dom.iloc[:, 5]
w_y_visdom = Visual_dom.iloc[:, 6]
w_z_visdom = Visual_dom.iloc[:, 7]
print("Sec_visdom: ", Sec_visdom[50:100])
print("Sec_Nano_visdom: ", Sec_Nano_visdom[50:100])
print("Sec_visdom_total: ", Sec_visdom_total[50:100])

Encoder = pd.read_csv('Encoder_velocities.csv')
Sec_Enc = Encoder.iloc[:, 0]
Sec_Nano_Enc = Encoder.iloc[:, 1]
Sec_Enc_total = Sec_Enc + Sec_Nano_Enc * 10 ** (-9)
Sec_Enc_total = Sec_Enc_total - Sec_Enc_total[0]
Left_wheel_ang = Encoder.iloc[:, 3]
Right_wheel_ang = Encoder.iloc[:, 4]
print("Sec_Enc: ", Sec_Enc[50:100])
print("Sec_Enc_total: ", Sec_Enc_total[50:100])
print("Sec_Nano_Enc: ", Sec_Nano_Enc[50:100])

Filtered_imu = pd.read_csv('Filtered_imu_data_corr.csv')
Sec_IMU = Filtered_imu.iloc[:, 0]
Sec_Nano_IMU = Filtered_imu.iloc[:, 1]
Sec_IMU_total = Sec_IMU + Sec_Nano_IMU * 10 ** (-9)
Sec_IMU_total = Sec_IMU_total - Sec_IMU_total[0]
a_x_IMU = Filtered_imu.iloc[:, 3]
a_y_IMU = Filtered_imu.iloc[:, 4]
a_z_IMU = Filtered_imu.iloc[:, 5]
w_x_IMU = Filtered_imu.iloc[:, 6]
w_y_IMU = Filtered_imu.iloc[:, 7]
w_z_IMU = Filtered_imu.iloc[:, 8]
print("Sec_IMU: ", Sec_IMU[50:100])
print("Sec_Nano_IMU: ", Sec_Nano_IMU[50:100])
print("Sec_IMU_total: ", Sec_IMU_total[50:100])

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
        self.r1 = np.array([0, 0, -0.154])
        ## vector of distance between two wheels
        self.r2 = np.array([0.56, 0, 0])

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
        # print("t1: ", t1)
        # print("t2: ", t2)
        # print("t: ", t)
        threshold = 1000
        if np.abs(t1 - t) < threshold or np.abs(t2 - t) < threshold:
            if np.abs(t1 - t) < np.abs(t2 - t):
                return "encoder", idx_1, t
            else:
                return "camera", idx_2, t
        else:
            return None, None, None

    def update_measurement(self, flag, idx):
        
        if(flag == "encoder"):
            w_l = np.array([-Left_wheel_ang[idx], 0, 0])
            w = np.array([w_x_IMU[idx], w_y_IMU[idx], w_z_IMU[idx]])
            y = helper_func.skew(w_l) @ self.r1 + 0.5 @ helper_func.skew(w) @ self.r2
            self.y_encoder = np.array([y[0], y[1], y[2], -1, 0])
        elif (flag == " camera"):
            self.v_c_observation = np.array([v_x_visdom[idx], v_y_visdom[idx], v_z_visdom[idx]])
            self.w_c_observation = np.array([w_x_visdom[idx], w_y_visdom[idx], w_z_visdom[idx]])


def plot(v, p, t):
    fig, axs = plt.subplots(6)
    fig.suptitle('Vertically stacked subplots')
    print("shape(v): ", np.shape(v))
    print("shape(t): ", np.shape(t))
    axs[0].plot(v[:, 0], t)
    axs[1].plot(v[:, 1], t)
    axs[2].plot(v[:, 2], t)
    axs[3].plot(p[:, 0], t)
    axs[4].plot(p[:, 1], t)
    axs[5].plot(p[:, 2], t)

def main():

    system = System()
    husky = inekf_imu_camera.Right_IEKF(system)

    
    v = husky.v
    p = husky.p
    t = np.zeros(1)

    # number of steps
    for i in range(1, len(Sec_IMU)):
        # updata dt, g, a, w
        system.update_IMU(i)

        # propagate between steps
        husky.propagation_state_and_error(system)
        husky.propagation_covariance(system)

        # correction step
        ## check the nearest measurement
        flag, idx, t_correct = system.choose_measurement(i)
        
        # update measurement data
        system.update_measurement(flag, idx)
        ## the nearest time stamp is encoder
        # print(flag)
        if(flag == "encoder"):
            husky.measurement_model_encoder(system)
            v = np.vstack((v,husky.v))
            p = np.vstack((p,husky.p))
            t = np.hstack((t, t_correct))
        elif (flag == "camera"):
            husky.measurement_model_camera(system)
            v = np.vstack((v,husky.v))
            p = np.vstack((p,husky.p))
            t = np.hstack((t, t_correct))
        # print("v: ", v)
    print("v: ", v)
    plot(v, p, t)
    

if __name__ == "__main__":
    main()