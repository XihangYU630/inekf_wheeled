import numpy as np
from scipy.linalg import block_diag, expm
import helper_func
import pandas as pd
import inekf_imu_cameraPos
import matplotlib.pyplot as plt
from scipy.spatial.transform import Rotation



measurements = pd.read_csv("sorted_sensor_dataset.csv")

Visual_dom_gt = pd.read_csv("Camera_ground_truth_parking.csv")
Sec_camera_pos = Visual_dom_gt.iloc[:, 0]
Sec_Nano_camera_pos = Visual_dom_gt.iloc[:, 1]
Sec_camera_pos_total = Sec_camera_pos + Sec_Nano_camera_pos * 10 ** (-9)
Sec_camera_pos_total = Sec_camera_pos_total - Sec_camera_pos_total[0]
p_x_visdom = Visual_dom_gt.iloc[:, 3]
p_y_visdom = Visual_dom_gt.iloc[:, 4]
p_z_visdom = Visual_dom_gt.iloc[:, 5]
q_x_visdom = Visual_dom_gt.iloc[:, 6]
q_y_visdom = Visual_dom_gt.iloc[:, 7]
q_z_visdom = Visual_dom_gt.iloc[:, 8]
q_w_visdom = Visual_dom_gt.iloc[:, 9]

Encoder = pd.read_csv('Encoder_data_parking.csv')
Sec_Enc = Encoder.iloc[:, 0]
Sec_Nano_Enc = Encoder.iloc[:, 1]
Sec_Enc_total = Sec_Enc + Sec_Nano_Enc * 10 ** (-9)
Sec_Enc_total = Sec_Enc_total - Sec_Enc_total[0]
Left_wheel_ang = Encoder.iloc[:, 3]
Right_wheel_ang = Encoder.iloc[:, 4]

GPS_parking = pd.read_csv("GPS_parking.csv")
Sec_GPS = GPS_parking.iloc[:, 0]
Sec_Nano_GPS = GPS_parking.iloc[:, 1]
Sec_GPS_total = Sec_GPS + Sec_Nano_GPS * 10 ** (-9)
Sec_GPS_total = Sec_GPS_total - Sec_GPS_total[0]
Lat_GPS = GPS_parking.iloc[:, 3]
Long_GPS = GPS_parking.iloc[:, 4]
Alt_GPS = GPS_parking.iloc[:, 5]

Filtered_imu = pd.read_csv('gx5_0_IMU_data_parking.csv')
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

Filtered_imu_1 = pd.read_csv('gx5_1_IMU_data_parking.csv')
Sec_IMU_1 = Filtered_imu_1.iloc[:, 0]
Sec_Nano_IMU_1 = Filtered_imu_1.iloc[:, 1]
Sec_IMU_total_1 = Sec_IMU_1 + Sec_Nano_IMU_1 * 10 ** (-9)
Sec_IMU_total_1 = Sec_IMU_total_1 - Sec_IMU_total_1[0]
a_x_IMU_1 = Filtered_imu_1.iloc[:, 3]
a_y_IMU_1 = Filtered_imu_1.iloc[:, 4]
a_z_IMU_1 = Filtered_imu_1.iloc[:, 5]
w_x_IMU_1 = Filtered_imu_1.iloc[:, 6]
w_y_IMU_1 = Filtered_imu_1.iloc[:, 7]
w_z_IMU_1 = Filtered_imu_1.iloc[:, 8]

def find_nearest(array, value):
    array = np.asarray(array)
    idx = (np.abs(array - value)).argmin()
    return idx, array[idx]

class System:
    def __init__(self):

        # initial guess of state variables
        self.R = np.array([[1, 0, -0.04],
                           [0, 0.99, -0.04],
                           [0.04, 0.04, 0.97]])  # error dynamics matrix
        # self.R = np.array([[0, 1, 0],
        #                    [-1, 0, 0],
        #                    [0, 0, 1]])  # error dynamics matrix
        # self.R = np.array([[1, 0, 0],
        #                    [0, 1, 0],
        #                    [0, 0, 1]])
        self.v = np.array([0.01, 0.01, 0.01])  # process model
        self.p = np.array([0, 0, 0])  # measurement error matrix
        self.b_w = np.array([0.01, 0.01, 0.01])  # measurement bias
        self.b_a = np.array([0.01, 0.01, 0.01])  # measurement bias
        # self.R_c = np.array([[0.95, 0.034, -0.32],
        #                      [-0.002, 0.99, 0.01],
        #                      [0.32, -0.09, 0.94]])  # state vector
        self.R_c = np.eye(3)
        self.p_c = np.array([0, 0, 0])  # state covariance

        self.P = np.eye(21) * 0.000000001
        self.P[15:21, 15:21] = np.zeros((6, 6))
        # hard code calibration
        self.g = np.array([0, 0, 9.8067])
        ## vector of wheel radius
        self.r1 = np.array([0, 0, 0.154])
        ## vector of distance between two wheels
        self.r2 = np.array([0.56, 0, 0])

        self.y_pos = np.zeros(5)
        self.quat = np.array([0, 0, 0, 1])

        # hard coded noise covariance
        ## these parameters are used for tuning
        self.cov_w = np.eye(21) * 1  # cov_w is 21*21 matrix
        ## w = (w_w, w_a, 0, w_ba, w_bw, w_Rc, w_pc)
        self.cov_w[6, 6] = 0
        self.cov_w[7, 7] = 0
        self.cov_w[8, 8] = 0
        self.nf_cov = np.diag([5, 5, 1])  # nf_cov is 3*3 matrix
        self.N_camera = np.eye(3) * 2.5  # N is covariance for camera data
        self.v_c_observation = np.zeros(3)
        self.w_c_observation = np.zeros(3)

def plot(v, p, b_w, b_a, R_quat, p_c, R_c_quat, t, measurement_pos, v_body, q_visdom):

    fig, axs = plt.subplots(4, 3, figsize=(15, 7))
    axs[0, 0].plot(t, v[:, 0])
    axs[0, 0].plot(t, v_body[:, 0])
    axs[0, 0].set_title("v_x")
    axs[0, 1].plot(t, v[:, 1])
    axs[0, 1].plot(t, v_body[:, 1])
    axs[0, 1].set_title("v_y")
    axs[0, 2].plot(t, v[:, 2])
    axs[0, 2].plot(t, v_body[:, 2])
    axs[0, 2].set_title("v_z")

    idx = 1000

    axs[1, 0].plot(t, p[:, 0])
    # axs[1, 0].plot(Sec_camera_pos_total[0:idx], p_x_visdom[0:idx])
    axs[1, 0].plot(t, measurement_pos[:, 0])
    axs[1, 0].set_title("p_x")
    axs[1, 1].plot(t, p[:, 1])
    # axs[1, 1].plot(Sec_camera_pos_total[0:idx], p_y_visdom[0:idx])
    axs[1, 1].plot(t, measurement_pos[:, 1])
    axs[1, 1].set_title("p_y")
    axs[1, 2].plot(t, p[:, 2])
    # axs[1, 2].plot(Sec_camera_pos_total[0:idx], p_z_visdom[0:idx])
    axs[1, 2].plot(t, measurement_pos[:, 2])
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
    fig, axs = plt.subplots(3, 4, figsize=(15, 5))
    axs[0, 0].plot(t, R_quat[:, 0])
    axs[0, 0].plot(t, q_visdom[:, 0])
    axs[0, 0].set_title("R_quat_x")
    axs[0, 1].plot(t, R_quat[:, 1])
    axs[0, 1].plot(t, q_visdom[:, 1])
    axs[0, 1].set_title("R_quat_y")
    axs[0, 2].plot(t, R_quat[:, 2])
    axs[0, 2].plot(t, q_visdom[:, 2])
    axs[0, 2].set_title("R_quat_z")
    axs[0, 3].plot(t, R_quat[:, 3])
    axs[0, 3].plot(t, q_visdom[:, 3])
    axs[0, 3].set_title("R_quat_w")

    axs[1, 0].plot(t, R_c_quat[:, 0])
    axs[1, 0].set_title("R_c_quat_x")
    axs[1, 1].plot(t, R_c_quat[:, 1])
    axs[1, 1].set_title("R_c_quat_y")
    axs[1, 2].plot(t, R_c_quat[:, 2])
    axs[1, 2].set_title("R_c_quat_z")
    axs[1, 3].plot(t, R_c_quat[:, 3])
    axs[1, 3].set_title("R_c_quat_w")

    axs[2, 0].plot(t, p_c[:, 0])
    axs[2, 0].set_title("p_c_x")
    axs[2, 1].plot(t, p_c[:, 1])
    axs[2, 1].set_title("p_c_y")
    axs[2, 2].plot(t, p_c[:, 2])
    axs[2, 2].set_title("p_c_z")



    fig.tight_layout()

    plt.show()


def main():
    system = System()
    husky = inekf_imu_cameraPos.Right_IEKF(system)

    v = husky.v
    p = husky.p
    b_w = husky.b_w
    b_a = husky.b_a
    p_c = husky.p_c
    v_body = np.zeros(3)

    t_stack = np.zeros(1)

    R = husky.R
    r = Rotation.from_matrix(R)
    R_quat = r.as_quat()

    R_c = husky.R_c
    r = Rotation.from_matrix(R_c)
    R_c_quat = r.as_quat()

    imu_measurement = np.zeros(6)
    imu_measurement_prev = np.zeros(6)
    t = 0
    t_prev = 0

    q_visdom = np.array([0, 0, 0, 1])

    measurement_pos = np.zeros(3)

    for idx, measurement in measurements.iterrows():
        if idx == 5000:
            break
        if measurement[0] == "i":
            t = measurement[3]
            system.dt = t - t_prev
            ## read system acceleration and angular velocity
            imu_measurement = measurement[4:10]
            system.a = imu_measurement_prev[0:3]
            system.w = imu_measurement_prev[3:6]
            # propagate between steps
            husky.propagation_state_and_error(system)
            husky.propagation_covariance(system)
        elif measurement[0] == "e":
            t = measurement[3]
            w_l = np.array([measurement[10], 0, 0])
            w = imu_measurement_prev[3:6]
            y_1 = helper_func.skew(w_l) @ system.r1 - 0.5 * helper_func.skew(w) @ system.r2
            system.y_encoder_1 = np.array([y_1[0], y_1[1], y_1[2], -1, 0])

            w_r = np.array([measurement[11], 0, 0])
            y_2 = helper_func.skew(w_r) @ system.r1 + 0.5 * helper_func.skew(w) @ system.r2
            system.y_encoder_2 = np.array([y_2[0], y_2[1], y_2[2], -1, 0])

            husky.measurement_model_encoder(system)
        elif measurement[0] == "p":
            t = measurement[3]

            pos = np.array([measurement[12], measurement[13], measurement[14]])
            # pos = pos - np.array([-0.15823796, 0.155218643, 0.080507172])
            r = np.array([[0, 1, 0],
                          [-1, 0, 0],
                          [0, 0, 1]])
            pos = r @ pos
            system.y_pos = np.array([pos[0], pos[1], pos[2], 0, 1])
            system.quat = np.array([measurement[15], measurement[16], measurement[17], measurement[18]])
            # husky.measurement_model_position(system)
        print("t: ", t)
        q_visdom = np.vstack((q_visdom, system.quat))
        measurement_pos = np.vstack((measurement_pos, system.y_pos[0:3]))

        t_prev = t
        imu_measurement_prev = imu_measurement
        '''collect data for plotting  '''
        v = np.vstack((v, husky.v))
        p = np.vstack((p, husky.p))
        b_w = np.vstack((b_w, husky.b_w))
        b_a = np.vstack((b_a, husky.b_a))
        p_c = np.vstack((p_c, husky.p_c))
        t_stack = np.hstack((t_stack, t))

        v_body = np.vstack((v_body, husky.R.T @ husky.v))

        R = husky.R
        r = Rotation.from_matrix(R)
        temp = r.as_quat()
        R_quat = np.vstack((R_quat, temp))

        R_c = husky.R_c
        r = Rotation.from_matrix(R_c)
        temp = r.as_quat()
        R_c_quat = np.vstack((R_c_quat, temp))

    ## transform the visdom to specified world frame

    # for i in range(len(measurement_pos)):
    #     r = np.array([[0, 1, 0],
    #                   [-1, 0, 0],
    #                   [0, 0, 1]])
    # #     measurement_pos[i, :] = r @ measurement_pos[i, :]
    #
    #     Rot = Rotation.from_quat(q_visdom[i, :])
    #     Rot_matrix = Rot.as_matrix()
    #     Rot_matrix_total = r @ Rot_matrix
    #     Rot_1 = Rotation.from_matrix(Rot_matrix_total)
    #     Rot_quat_total = Rot_1.as_quat()
    #     q_visdom[i, :] = Rot_quat_total

    plot(v, p, b_w, b_a, R_quat, p_c, R_c_quat, t_stack, measurement_pos, v_body, q_visdom)


if __name__ == "__main__":
    main()