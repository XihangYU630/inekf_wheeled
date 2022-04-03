
import numpy as np
from scipy.linalg import block_diag, expm
import helper_func
import pandas as pd
import inekf_imu_camera
import matplotlib.pyplot as plt
from scipy.spatial.transform import Rotation


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

Visual_dom_gt = pd.read_csv("Camera_ground_truth.csv")
Sec_camera_pos = Visual_dom_gt.iloc[:, 0]
Sec_Nano_camera_pos = Visual_dom_gt.iloc[:, 1]
Sec_camera_pos_total = Sec_camera_pos + Sec_Nano_camera_pos * 10 ** (-9)
Sec_camera_pos_total = Sec_camera_pos_total - Sec_camera_pos_total[0]
# print("type(Sec_visdom_total): ", type(Sec_visdom_total))
p_x_visdom = Visual_dom_gt.iloc[:, 3]
p_y_visdom = Visual_dom_gt.iloc[:, 4]
p_z_visdom = Visual_dom_gt.iloc[:, 5]
q_x_visdom = Visual_dom_gt.iloc[:, 6]
q_y_visdom = Visual_dom_gt.iloc[:, 7]
q_z_visdom = Visual_dom_gt.iloc[:, 8]
q_w_visdom = Visual_dom_gt.iloc[:, 9]


Encoder = pd.read_csv('Encoder_velocities_corr.csv')
Sec_Enc = Encoder.iloc[:, 0]
Sec_Nano_Enc = Encoder.iloc[:, 1]
Sec_Enc_total = Sec_Enc + Sec_Nano_Enc * 10 ** (-9)
Sec_Enc_total = Sec_Enc_total - Sec_Enc_total[0]
Left_wheel_ang = Encoder.iloc[:, 3]
Right_wheel_ang = Encoder.iloc[:, 4]

Filtered_imu = pd.read_csv('Unfiltered_imu_data.csv')
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

def find_nearest(array, value):
    array = np.asarray(array)
    idx = (np.abs(array - value)).argmin()
    return idx, array[idx]

class System:
    def __init__(self):

        #initial guess of state variables
        self.R = np.array([[1, 0, 0],
                           [0, 0.99, 0.1],
                           [0, -0.1, 0.97]])  # error dynamics matrix
        # self.R = np.array([[1, 0, 0],
        #                    [0, 1, 0],
        #                    [0, 0, 1]])
        self.v = np.array([0.01, 0.01, 0.01])  # process model
        self.p = np.array([67, -14, 8])  # measurement error matrix
        self.b_w = np.array([0.01, 0.01, 0.01])  # measurement bias
        self.b_a = np.array([0.01, 0.01, 0.01])  # measurement bias
        self.R_c = np.array([[0.95, 0.034, -0.32],
                           [-0.002, 0.99, 0.01],
                           [0.32, -0.09, 0.94]])  # state vector
        self.p_c = np.array([0.01, 0.01, 0.01])  # state covariance

        self.P = np.eye(21)*0.000001
        # self.P[0, 0] = 0.0001
        # self.P[1, 1] = 0.0001
        # self.P[2, 2] = 0.0001
        self.P[3, 3] = 0.001
        self.P[4, 4] = 0.001
        self.P[5, 5] = 0.001
        # hard code calibration
        self.g = np.array([0, 0, 9.8067])
        ## vector of wheel radius
        self.r1 = np.array([0, 0, 0.154])
        ## vector of distance between two wheels
        self.r2 = np.array([0.56, 0, 0])

        # hard coded noise covariance
        ## these parameters are used for tuning
        self.cov_w = np.eye(21)*0.1 # cov_w is 21*21 matrix
        ## w = (w_w, w_a, 0, w_ba, w_bw, w_Rc, w_pc)
        self.cov_w[6, 6] = 0
        self.cov_w[7, 7] = 0
        self.cov_w[8, 8] = 0
        self.nf_cov = np.eye(3)*1 # nf_cov is 3*3 matrix
        self.N_camera = np.eye(6)*0.01 # N is covariance for camera data
        self.v_c_observation = np.zeros(3)
        self.w_c_observation = np.zeros(3)

    
    def update_IMU(self, i):
        # update dt, a, w
        self.dt = Sec_IMU_total[i] - Sec_IMU_total[i-1]
        self.a = np.array([a_x_IMU[i-1], a_y_IMU[i-1], a_z_IMU[i-1]])
        self.w = np.array([w_x_IMU[i-1], w_y_IMU[i-1], w_z_IMU[i-1]])
    
    def choose_measurement(self, i):
        t = Sec_IMU_total[i]
        idx_1, t1 = find_nearest(Sec_Enc_total, t)
        idx_2, t2 = find_nearest(Sec_visdom_total, t)
        threshold = 0.005
        if np.abs(t1 - t) < threshold or np.abs(t2 - t) < threshold:
            if np.abs(t1 - t) < np.abs(t2 - t):
                return "encoder", idx_1, t
            else:
                return "camera", idx_2, t
        else:
            return None, None, None

    def update_measurement(self, flag, idx, i):

        if(flag == "encoder"):
            w_l = np.array([Left_wheel_ang[idx], 0, 0])
            w = np.array([w_x_IMU[i], w_y_IMU[i], w_z_IMU[i]])
            y_1 = helper_func.skew(w_l) @ self.r1 - 0.5 * helper_func.skew(w) @ self.r2
            self.y_encoder_1 = np.array([y_1[0], y_1[1], y_1[2], -1, 0])

            w_r = np.array([Right_wheel_ang[idx], 0, 0])
            y_2 = helper_func.skew(w_r) @ self.r1 + 0.5 * helper_func.skew(w) @ self.r2
            self.y_encoder_2 = np.array([y_2[0], y_2[1], y_2[2], -1, 0])
        elif (flag == "camera"):

            # self.v_c_observation = np.array([0, 0, 0])
            # self.w_c_observation = np.array([0, 0, 0])

            self.v_c_observation = np.array([v_x_visdom[idx], v_y_visdom[idx], v_z_visdom[idx]])
            self.w_c_observation = np.array([w_x_visdom[idx], w_y_visdom[idx], w_z_visdom[idx]])


def plot(v, p, b_w, b_a, R_quat, p_c, R_c_quat, y_encoder_vel, y_encoder_vel_z, t):

    fig, axs = plt.subplots(4, 3, figsize=(15, 10))
    axs[0, 0].plot(t, v[:, 0])
    axs[0, 0].plot(Sec_visdom_total[0:700], v_x_visdom[0:700])
    axs[0, 0].plot(t, y_encoder_vel[:, 0])
    axs[0, 0].set_title("v_x")
    axs[0, 1].plot(t, v[:, 1])
    axs[0, 1].plot(Sec_visdom_total[0:700], v_y_visdom[0:700])
    axs[0, 1].plot(t, y_encoder_vel[:, 1])
    axs[0, 1].set_title("v_y")
    axs[0, 2].plot(t, v[:, 2])
    axs[0, 2].plot(Sec_visdom_total[0:700], v_z_visdom[0:700])
    axs[0, 2].plot(t, y_encoder_vel[:, 2])
    axs[0, 2].plot(t, y_encoder_vel_z)
    axs[0, 2].set_title("v_z")


    axs[1, 0].plot(t, p[:, 0])
    axs[1, 0].plot(Sec_camera_pos_total[0:700], p_x_visdom[0:700])
    axs[1, 0].set_title("p_x")
    axs[1, 1].plot(t, p[:, 1])
    axs[1, 1].plot(Sec_camera_pos_total[0:700], p_y_visdom[0:700])
    axs[1, 1].set_title("p_y")
    axs[1, 2].plot(t, p[:, 2])
    axs[1, 2].plot(Sec_camera_pos_total[0:700], p_z_visdom[0:700])
    axs[1, 2].set_title("p_z")

    axs[2, 0].plot(t, b_w[:, 0])
    axs[2, 0].set_title("bw_x")
    axs[2, 1].plot(t, b_w[:, 1])
    axs[2, 1].set_title("bw_y")
    axs[2, 2].plot(t, b_w[:, 2])
    axs[2, 2].set_title("bw_z")

    axs[3, 0].plot(t, b_a[:, 0])
    axs[3, 0].set_title("b_a_x")
    axs[3, 1].plot(t, b_a[:, 1])
    axs[3, 1].set_title("b_a_y")
    axs[3, 2].plot(t, b_a[:, 2])
    axs[3, 2].set_title("b_a_z")

    plt.show()
    fig, axs = plt.subplots(2, 4, figsize=(15, 5))
    print("R_quat: ", R_quat)
    print("t: ", t)
    axs[0, 0].plot(t, R_quat[:, 0])
    axs[0, 0].plot(Sec_camera_pos_total[0:700], q_x_visdom[0:700])
    axs[0, 0].set_title("R_quat_x")
    axs[0, 1].plot(t, R_quat[:, 1])
    axs[0, 1].plot(Sec_camera_pos_total[0:700], q_y_visdom[0:700])
    axs[0, 1].set_title("R_quat_y")
    axs[0, 2].plot(t, R_quat[:, 2])
    axs[0, 2].plot(Sec_camera_pos_total[0:700], q_z_visdom[0:700])
    axs[0, 2].set_title("R_quat_z")
    axs[0, 3].plot(t, R_quat[:, 3])
    axs[0, 3].plot(Sec_camera_pos_total[0:700], q_w_visdom[0:700])
    axs[0, 3].set_title("R_quat_w")


    # axs[1, 0].plot(t, p_c[:, 0])
    # axs[1, 0].set_title("p_c_x")
    # axs[1, 1].plot(t, p_c[:, 1])
    # axs[1, 1].set_title("p_c_y")
    # axs[1, 2].plot(t, p_c[:, 2])
    # axs[1, 2].set_title("p_c_z")
    #
    # axs[2, 0].plot(t, R_c_quat[:, 0])
    # axs[2, 0].set_title("R_c_quat_x")
    # axs[2, 1].plot(t, R_c_quat[:, 1])
    # axs[2, 1].set_title("R_c_quat_y")
    # axs[2, 2].plot(t, R_c_quat[:, 2])
    # axs[2, 2].set_title("R_c_quat_z")
    # axs[2, 3].plot(t, R_c_quat[:, 3])
    # axs[2, 3].set_title("R_c_quat_w")

    # plt.show()
    # fig, axs = plt.subplots(3, 7)
    # axs[0, 0].plot(t, error_estimated[:, 0])
    # axs[0, 0].set_title("error_estimated_x")
    # axs[0, 1].plot(t, p_c[:, 0])
    # axs[0, 1].set_title("p_c_x")
    # axs[0, 2].plot(t, p_c[:, 1])
    # axs[0, 2].set_title("p_c_x")
    #
    # axs[0, 3].set_title("p_c_y")
    # axs[0, 4].plot(t, p_c[:, 2])
    # axs[0, 5].set_title("p_c_z")
    # axs[0, 6].plot(t, R_c_quat[:, 0])
    #
    # axs[1, 0].set_title("R_c_quat_x")
    # axs[1, 1].plot(t, R_c_quat[:, 1])
    # axs[1, 2].set_title("R_c_quat_y")
    # axs[1, 3].plot(t, R_c_quat[:, 2])
    # axs[1, 4].set_title("R_c_quat_z")
    # axs[1, 5].plot(t, R_c_quat[:, 3])
    # axs[1, 6].set_title("R_c_quat_w")
    #
    # axs[1, 0].set_title("R_c_quat_x")
    # axs[1, 1].plot(t, R_c_quat[:, 1])
    # axs[1, 2].set_title("R_c_quat_y")
    # axs[1, 3].plot(t, R_c_quat[:, 2])
    # axs[1, 4].set_title("R_c_quat_z")
    # axs[1, 5].plot(t, R_c_quat[:, 3])
    # axs[1, 6].set_title("R_c_quat_w")

    fig.tight_layout()


    # fig, axs = plt.subplots(1, 3)
    # axs[0].plot(t, v[:, 0])
    # axs[0].set_title("v_x")
    # axs[1].plot(t, v[:, 1])
    # axs[1].set_title("v_y")
    # # axs[0, 2].sharex(axs[0, 0])
    # axs[2].plot(t, v[:, 2])
    # axs[2].set_title("v_z")
    #
    # fig.tight_layout()

    plt.show()

def main():

    system = System()
    husky = inekf_imu_camera.Right_IEKF(system)

    
    v = husky.v
    p = husky.p
    b_w = husky.b_w
    b_a = husky.b_a
    p_c = husky.p_c
    y_encoder_vel = np.zeros(3)
    y_encoder_vel_z = np.zeros(1)

    t = np.zeros(1)

    R = husky.R
    r = Rotation.from_matrix(R)
    R_quat = r.as_quat()

    R_c = husky.R_c
    r = Rotation.from_matrix(R_c)
    R_c_quat = r.as_quat()
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
        print("idx: ", idx)
        # update measurement data
        system.update_measurement(flag, idx, i)
        ## the nearest time stamp is encoder
        # print(flag)
        if i==2500:
            break

        if(flag == "encoder"):
            husky.measurement_model_encoder(system)
            v = np.vstack((v,husky.v))
            p = np.vstack((p,husky.p))
            b_w = np.vstack((b_w, husky.b_w))
            b_a = np.vstack((b_a, husky.b_a))
            p_c = np.vstack((p_c, husky.b_a))
            # error_estimated = np.vstack((error_estimated, system.error_estimated))
            t = np.hstack((t, t_correct))

            print("system.y_encoder[2]: ", system.y_encoder_1[2])
            y_encoder_vel = np.vstack((y_encoder_vel, husky.R @ system.y_encoder_1[0:3]))
            y_encoder_vel_z = np.vstack((y_encoder_vel_z, system.y_encoder_1[2]))

            R = husky.R
            r = Rotation.from_matrix(R)
            temp = r.as_quat()
            R_quat = np.vstack((R_quat, temp))


            R_c = husky.R_c
            r = Rotation.from_matrix(R_c)
            temp = r.as_quat()
            R_c_quat = np.vstack((R_c_quat, temp))

        # if (flag == "camera"):
        #     husky.measurement_model_camera(system)
        #     v = np.vstack((v,husky.v))
        #     p = np.vstack((p,husky.p))
        #     b_w = np.vstack((b_w, husky.b_w))
        #     b_a = np.vstack((b_a, husky.b_a))
        #     p_c = np.vstack((p_c, husky.b_a))
        #     # error_estimated = np.vstack((error_estimated, system.error_estimated))
        #     t = np.hstack((t, t_correct))
        #
        #     R = husky.R
        #     r = Rotation.from_matrix(R)
        #     temp = r.as_quat()
        #     R_quat = np.vstack((R_quat, temp))
        #
        #     R_c = husky.R_c
        #     r = Rotation.from_matrix(R_c)
        #     temp = r.as_quat()
        #     R_c_quat = np.vstack((R_c_quat, temp))

    plot(v, p, b_w, b_a, R_quat, p_c, R_c_quat, y_encoder_vel, y_encoder_vel_z, t)
    

if __name__ == "__main__":
    main()