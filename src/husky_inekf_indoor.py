import numpy as np
import helper_func
import pandas as pd
import inekf_imu_cameraPos
import matplotlib.pyplot as plt
from scipy.spatial.transform import Rotation



measurements = pd.read_csv("data/sorted_sensor_dataset_new.csv")

imu = pd.read_csv("data/Mat_IMU_indoor.csv")
encoder = pd.read_csv("data/Mat_Encoder_indoor.csv")
position_Camera_SLAM = pd.read_csv("data/Mat_Camera_Path_indoor.csv")
position_odom = pd.read_csv("data/Mat_Odom_indoor.csv")

merged = pd.merge(imu, encoder, on=['t'], how='outer')
merged = pd.merge(merged, position_odom, on=['t'], how='outer')
merged.sort_values(["t"],
                    axis=0,
                    inplace=True)
merged.to_csv("merged_indoor.csv", index=False)



class System:
    def __init__(self):

        # initial guess of state variables
        # self.R = np.array([[1, 0, -0.04],
        #                    [0, 0.99, -0.04],
        #                    [0.04, 0.04, 0.97]])  # error dynamics matrix
        # self.R = np.array([[0.1542515,  0.0000000, -0.9880316],
        #                    [0.9762065,  0.1542515,  0.1524053],
        #                    [0.1524053, -0.9880316,  0.0237935]])
        self.R = np.array([[1, 0, 0],
                           [0, 1, 0],
                           [0, 0, 1]])  # error dynamics matrix
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
        # self.R_c = np.eye(3)
        self.R_c = np.array([[0, 1, 0],
                          [-1, 0, 0],
                          [0, 0, 1]])
        self.p_c = np.array([0, 0, 0])  # state covariance

        self.P = np.eye(21) * 1
        self.P[3:6, 3:6] = np.eye(3)
        # self.P[15:21, 15:21] = np.zeros((6, 6))
        # hard code calibration
        self.g = np.array([0, 0, 9.8067])
        ## vector of wheel radius
        self.r1 = np.array([0, 0, 0.33/2])
        ## vector of distance between two wheels
        self.r2 = np.array([0.556, 0, 0]) ## encoder original
        # self.r2 = np.array([0, 0.556, 0])

        self.y_pos = np.zeros(5)
        self.quat = np.array([0, 0, 0, 1])

        # hard coded noise covariance
        ## these parameters are used for tuning
        self.cov_w = np.eye(21) * 1  # cov_w is 21*21 matrix
        self.cov_w[15, 15] = 5
        self.cov_w[16, 16] = 5
        self.cov_w[17, 17] = 5
        ## w = (w_w,w_a, 0, w_ba, w_bw, w_Rc, w_pc)
        self.cov_w[18, 18] = 5
        self.cov_w[19, 19] = 5
        self.cov_w[20, 20] = 5
        self.nf_cov = np.diag([10, 10, 10])  # nf_cov is 3*3 matrix
        self.N_pseudo = np.diag([5, 5])
        self.N_camera = np.diag([100, 100, 100, 10, 10, 10])  # N is covariance for camera data
        self.N_pos = np.eye(3) * 5
        self.v_c_observation = np.zeros(3)
        self.w_c_observation = np.zeros(3)

        # self.n = 5 #number in the tuning window

    # def compute_camera_covariance(self, measurements_camera):
    #
    #     self.N_camera = np.zeros((6, 6))
    #     for i in range(self.n):
    #         sum = np.zeros(6)
    #         for measurement in measurements_camera:
    #             sum = sum + measurement
    #             print("sum: ", sum)
    #         e_i = measurements_camera[i] - sum / self.n
    #         print("e: ", e_i)
    #         self.N_camera = self.N_camera + e_i.reshape(6, 1) @ e_i.reshape(1, 6)
    #         # print("N_camera: ", self.N_camera)
    #     self.N_camera = self.N_camera / self.n
    #     print("N_camera: ", self.N_camera)

    # def compute_encoder_covariance(self, measurements_encoder):
    #
    #     self.nf_cov = np.zeros((3, 3))
    #     for measurement in measurements_encoder:
    #         e = measurement - np.sum(measurement) / self.n
    #         print("encoder.shape(e): ", np.shape(e))
    #         self.nf_cov = self.nf_cov + e.reshape(3, 1) @ e.reshape(1, 3)
    #     self.nf_cov = self.nf_cov / self.n

def plot(v, p, b_w, b_a, R_quat, p_c, R_c_quat, t, measurement_pos, v_body, q_visdom, v_body_camera):


    ## plot 2d trajectory
    fig = plt.figure(figsize=(16,8))
    plt.plot(measurement_pos[:, 0], measurement_pos[:, 1], linewidth=1)
    plt.plot(p[:, 0], p[:, 1],linewidth=1)
    plt.legend(("visual_odometry", "proposed"), loc="center left")
    # plt.set_xlabel("x-pos (m)")
    # plt.set_ylabel("y-pos (m)")
    plt.show()


    ## plotting trajectory
    fig = plt.figure(figsize=(16,8))
    ax = plt.axes(projection='3d')
    ax.plot3D(measurement_pos[:, 0], measurement_pos[:, 1], measurement_pos[:, 2], linewidth=1)
    ax.plot3D(p[:, 0], p[:, 1], p[:, 2],linewidth=1)
    plt.legend(("visual_odometry", "proposed"), loc="center left")
    ax.set_xlabel("x-pos (m)")
    ax.set_ylabel("y-pos (m)")
    ax.set_zlabel("z-pos (m)")
    plt.show()



    ## plot against time t
    fig, axs = plt.subplots(4, 3, figsize=(30, 7))
    fig.subplots_adjust(hspace=1.0)
    fig.subplots_adjust(wspace=0.2)
    axs[0, 0].plot(t, v[:, 0], linewidth=1)
    axs[0, 0].plot(t, v_body[:, 0], linewidth=1)
    axs[0, 0].set_xlabel('time (s)')
    axs[0, 0].set_title('velocity-x (m/s)')
    axs[0, 0].xaxis.set_label_coords(.9, -.3)

    axs[0, 1].plot(t, v[:, 1], linewidth=1)
    axs[0, 1].plot(t, v_body[:, 1], linewidth=1)
    axs[0, 1].set_xlabel('time (s)')
    axs[0, 1].set_title('velocity-y (m/s)')
    axs[0, 1].xaxis.set_label_coords(.9, -.3)

    axs[0, 2].plot(t, v[:, 2], linewidth=1)
    axs[0, 2].plot(t, v_body[:, 2], linewidth=1)
    axs[0, 2].set_xlabel('time (s)')
    axs[0, 2].set_title('velocity-z (m/s)')
    axs[0, 2].xaxis.set_label_coords(.9, -.3)

    axs[1, 0].plot(t, p[:, 0], linewidth=1)
    axs[1, 0].plot(t, measurement_pos[:, 0], linewidth=1)
    axs[1, 0].set_xlabel('time (s)')
    axs[1, 0].set_title('position-x (m)')
    axs[1, 0].xaxis.set_label_coords(.9, -.3)

    axs[1, 1].plot(t, p[:, 1], linewidth=1)
    # axs[1, 1].plot(Sec_camera_pos_total[0:idx], p_y_visdom[0:idx])
    axs[1, 1].plot(t, measurement_pos[:, 1], linewidth=1)
    axs[1, 1].set_xlabel('time (s)')
    axs[1, 1].set_title('position-y (m)')
    axs[1, 1].xaxis.set_label_coords(.9, -.3)

    axs[1, 2].plot(t, p[:, 2], linewidth=1)
    # axs[1, 2].plot(Sec_camera_pos_total[0:idx], p_z_visdom[0:idx])
    axs[1, 2].plot(t, measurement_pos[:, 2], linewidth=1)
    axs[1, 2].set_xlabel('time (s)')
    axs[1, 2].set_title('position-z (m)')
    axs[1, 2].xaxis.set_label_coords(.9, -.3)


    axs[2, 0].plot(t, b_w[:, 0], linewidth=1)
    axs[2, 0].set_xlabel('time (s)')
    axs[2, 0].set_title('bias-w-x (rad)')
    axs[2, 0].xaxis.set_label_coords(.9, -.3)

    axs[2, 1].plot(t, b_w[:, 1], linewidth=1)
    axs[2, 1].set_xlabel('time (s)')
    axs[2, 1].set_title('bias-w-y (rad)')
    axs[2, 1].xaxis.set_label_coords(.9, -.3)

    axs[2, 2].plot(t, b_w[:, 2], linewidth=1)
    axs[2, 2].set_xlabel('time (s)')
    axs[2, 2].set_title('bias-w-z (rad)')
    axs[2, 2].xaxis.set_label_coords(.9, -.3)

    axs[3, 0].plot(t, b_a[:, 0], linewidth=1)
    axs[3, 0].set_xlabel('time (s)')
    axs[3, 0].set_title('bias-a-x (m/s)')
    axs[3, 0].xaxis.set_label_coords(.9, -.3)

    axs[3, 1].plot(t, b_a[:, 1], linewidth=1)
    axs[3, 1].set_xlabel('time (s)')
    axs[3, 1].set_title('bias-a-y (m/s)')
    axs[3, 1].xaxis.set_label_coords(.9, -.3)

    axs[3, 2].plot(t, b_a[:, 2], linewidth=1)
    axs[3, 2].set_xlabel('time (s)')
    axs[3, 2].set_title('bias-a-z (m/s)')
    axs[3, 2].xaxis.set_label_coords(.9, -.3)

    plt.show()
    fig, axs = plt.subplots(3, 4, figsize=(30, 5))
    fig.subplots_adjust(hspace=0.8)
    fig.subplots_adjust(wspace=0.4)
    axs[0, 0].plot(t, R_quat[:, 0], linewidth=1)
    axs[0, 0].plot(t, q_visdom[:, 0], linewidth=1)
    axs[0, 0].set_xlabel('time (s)')
    axs[0, 0].set_title('quaternion-x (R)')
    axs[0, 0].xaxis.set_label_coords(.9, -.3)
    axs[0, 0].tick_params(axis='y', labelsize=8)

    axs[0, 1].plot(t, R_quat[:, 1], linewidth=1)
    axs[0, 1].plot(t, q_visdom[:, 1], linewidth=1)
    axs[0, 1].set_xlabel('time (s)')
    axs[0, 1].set_title('quaternion-y (R)')
    axs[0, 1].xaxis.set_label_coords(.9, -.3)
    axs[0, 1].tick_params(axis='y', labelsize=8)

    axs[0, 2].plot(t, R_quat[:, 2], linewidth=1)
    axs[0, 2].plot(t, q_visdom[:, 2], linewidth=1)
    axs[0, 2].set_xlabel('time (s)')
    axs[0, 2].set_title('quaternion-z (R)')
    axs[0, 2].xaxis.set_label_coords(.9, -.3)
    axs[0, 2].tick_params(axis='y', labelsize=8)

    axs[0, 3].plot(t, R_quat[:, 3], linewidth=1)
    axs[0, 3].plot(t, q_visdom[:, 3], linewidth=1)
    axs[0, 3].set_xlabel('time (s)')
    axs[0, 3].set_title('quaternion-w (R)')
    axs[0, 3].xaxis.set_label_coords(.9, -.3)
    axs[0, 3].tick_params(axis='y', labelsize=8)

    axs[1, 0].plot(t, R_c_quat[:, 0], linewidth=1)
    axs[1, 0].set_xlabel('time (s)')
    axs[1, 0].set_title('quaternion-x (Rc)')
    axs[1, 0].xaxis.set_label_coords(.9, -.3)
    axs[1, 0].tick_params(axis='y', labelsize=7)

    axs[1, 1].plot(t, R_c_quat[:, 1], linewidth=1)
    axs[1, 1].set_xlabel('time (s)')
    axs[1, 1].set_title('quaternion-y (Rc)')
    axs[1, 1].xaxis.set_label_coords(.9, -.3)
    axs[1, 1].tick_params(axis='y', labelsize=7)

    axs[1, 2].plot(t, R_c_quat[:, 2], linewidth=1)
    axs[1, 2].set_xlabel('time (s)')
    axs[1, 2].set_title('quaternion-z (Rc)')
    axs[1, 2].xaxis.set_label_coords(.9, -.3)
    axs[1, 2].tick_params(axis='y', labelsize=7)

    axs[1, 3].plot(t, R_c_quat[:, 3], linewidth=1)
    axs[1, 3].set_xlabel('time (s)')
    axs[1, 3].set_title('quaternion-w (Rc)')
    axs[1, 3].xaxis.set_label_coords(.9, -.3)
    axs[1, 3].tick_params(axis='y', labelsize=7)

    axs[2, 0].plot(t, p_c[:, 0], linewidth=1)
    axs[2, 0].set_xlabel('time (s)')
    axs[2, 0].set_title('position-camera-x (Rc)')
    axs[2, 0].xaxis.set_label_coords(.9, -.3)
    axs[2, 0].tick_params(axis='y', labelsize=8)

    axs[2, 1].plot(t, p_c[:, 1], linewidth=1)
    axs[2, 1].set_xlabel('time (s)')
    axs[2, 1].set_title('position-camera-y (Rc)')
    axs[2, 1].xaxis.set_label_coords(.9, -.3)
    axs[2, 1].tick_params(axis='y', labelsize=8)

    axs[2, 2].plot(t, p_c[:, 2], linewidth=1)
    axs[2, 2].set_xlabel('time (s)')
    axs[2, 2].set_title('position-camera-z (Rc)')
    axs[2, 2].xaxis.set_label_coords(.9, -.3)
    axs[2, 2].tick_params(axis='y', labelsize=8)

    axs[2, 3].tick_params(axis='y', labelsize=8)

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
    v_body_camera = np.zeros(3)

    t_stack = np.zeros(1)

    R = husky.R
    r = Rotation.from_matrix(R)
    R_quat = r.as_quat()

    R_c = husky.R_c
    r = Rotation.from_matrix(R_c)
    R_c_quat = r.as_quat()

    imu_measurement = np.zeros(6)
    imu_measurement_prev = np.zeros(6)
    # measurements_camera = np.zeros((system.n, 6))
    # measurements_encoder = np.zeros((20, 2))

    t = 0
    t_prev = 0

    q_visdom = np.array([0, 0, 0, 1])

    measurement_pos = np.zeros(3)

    count = 0


    for idx, measurement in merged.iterrows():
        if idx == 15000:
            break
        if measurement[0] == "i":
            t = measurement[1]
            system.dt = t - t_prev
            ## read system acceleration and angular velocity
            imu_measurement = measurement[2:8]
            system.a = imu_measurement_prev[0:3]
            system.w = imu_measurement_prev[3:6]
            # propagate between steps
            husky.propagation_state_and_error(system)
            husky.propagation_covariance(system)



        if measurement[12] == "e":
            t = measurement[1]
            w_l = np.array([measurement[13], 0, 0]) ## encoder original
            w_r = np.array([measurement[14], 0, 0]) ## encoder original
            # w_l = np.array([0, measurement[10], 0])
            # w_r = np.array([0, measurement[11], 0])

            # w_wheel = np.array([measurement[10], measurement[11]])
            # measurements_encoder = np.append(measurements_encoder, [w_wheel], axis=0)
            # measurements_encoder = np.delete(measurements_encoder, (0), 0)
            # if count >= system.n:
            #     system.compute_encoder_covariance(measurements_encoder)

            w = imu_measurement_prev[3:6]
            print("w: ", w)
            y_1 = helper_func.skew(w_l) @ system.r1 - 0.5 * helper_func.skew(w) @ system.r2
            system.y_encoder_1 = np.array([y_1[0], y_1[1], y_1[2], -1, 0]) ## encoder original
            # system.y_encoder_1 = y_1

            y_2 = helper_func.skew(w_r) @ system.r1 + 0.5 * helper_func.skew(w) @ system.r2
            system.y_encoder_2 = np.array([y_2[0], y_2[1], y_2[2], -1, 0]) ## encoder original
            # system.y_encoder_2 = y_2

            husky.measurement_model_encoder(system)

            system.y_pseudo = np.array([0, 0])
            husky.measurement_pseudo(system)

            # system.y_pseudo_1 = np.array([0, y_1[1], 0, -1, 0])
            #
            # system.y_pseudo_2 = np.array([0, y_2[1], 0, -1, 0])
            # husky.measurement_pseudo(system)
        if measurement[15] == "p":
            t = measurement[1]

            pos = np.array([measurement[16], measurement[17], measurement[18]])
            # pos = pos - np.array([-0.15823796, 0.155218643, 0.080507172])


            r = np.array([[0, 1, 0],
                          [-1, 0, 0],
                          [0, 0, 1]])
            pos = r @ pos
            system.y_pos = np.array([pos[0], pos[1], pos[2], 0, 1])
            system.quat = np.array([measurement[20], measurement[21], measurement[22], measurement[19]])


            system.v_c_observation = np.array([measurement[26], measurement[27], measurement[28]])
            system.w_c_observation = np.array([measurement[23], measurement[24], measurement[25]])

            # v_plus_w = np.array([measurement[19], measurement[20], measurement[21], measurement[7], measurement[8], measurement[9]])
            # measurements_camera = np.append(measurements_camera, [v_plus_w], axis=0)
            # measurements_camera = np.delete(measurements_camera, (0), 0)
            # if count >= system.n and t > 20:
            #     system.compute_camera_covariance(measurements_camera)

            # system.v_c_observation = np.array([0, 0, 0])
            # husky.measurement_model_camera(system)
            # husky.measurement_model_position(system)

        count = count + 1

        print("t: ", t)
        q_visdom = np.vstack((q_visdom, system.quat))
        measurement_pos = np.vstack((measurement_pos, system.y_pos[0:3]))
        v_body_camera = np.vstack((v_body_camera, system.v_c_observation))

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

    plot(v, p, b_w, b_a, R_quat, p_c, R_c_quat, t_stack, measurement_pos, v_body, q_visdom, v_body_camera)


if __name__ == "__main__":
    main()