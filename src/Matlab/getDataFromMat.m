function [GPS,IMU,CAMERA,ENCODER] = getDataFromMat()
    %   GETDATAFROMCSV function reads the csv files of the gps, camera, imu and encoder data
    %   and returns the stored data
    %
    %   Author: Theodor Chakhachiro
    %   Date:   04/02/2022
    
    %% Read mat file
    data_mat = load('Data2\Parking\DATA.mat');
    data_mat = data_mat.sortedsensordatasetnew;
    data_mat(1,:) = [];
    data_mat(:,2:3) = [];

    %% GPS Data formated into { time (s) | x (m) | y (m) | z (m) }
    gps_mat = readmatrix('Data2/Parking/GPS_parking.csv');
    % gps_mat(any(isnan(gps_mat), 2), :) = [];
    gps_mat(:,4:6) = lla2ecef( gps_mat(:,4:6) ) - lla2ecef(gps_mat(1,4:6));
    gps_mat(:,1:2) = [];
    GPS.t = gps_mat(:,1); 
    GPS.Position.X = gps_mat(:,2); GPS.Position.Y = gps_mat(:,3); GPS.Position.Z = gps_mat(:,4);

    %% Encoder Data formated into { time (s) | Left Wheel Velocity v_l (m/s) | Right Wheel Velocity v_r (m/s) }
    enc_mat = readmatrix('Data2/Parking/Encoder_data_parking.csv');
    enc_mat(:,1:2) = [];
    ENCODER.t = enc_mat(:,1); 
    ENCODER.Velocity.Left = enc_mat(:,2); ENCODER.Velocity.Right = enc_mat(:,3);

    %% Camera Ground Truth Data formated into { time (s) | x (m) | y (m) | z (m) | phi (rad) | theta (rad) | psi (rad) }
    camera_mat = readmatrix('Data2/Parking/Camera_ground_truth_parking.csv');
    camera_mat(:,4:6) = camera_mat(:,4:6) - camera_mat(1,4:6);
    camera_mat(:,7:9) = quat2eul([camera_mat(:,10) camera_mat(:,7:9)]);
    camera_mat(:,10) = [];
    camera_mat(:,1:2) = [];
    camera_mat(end,:) = []; % Fit Data between GT and Odom
    CAMERA.t = camera_mat(:,1); 
    CAMERA.Position.X = camera_mat(:,2); CAMERA.Position.Y = camera_mat(:,3); CAMERA.Position.Z = camera_mat(:,4);
    CAMERA.Orientation.X = camera_mat(:,5); CAMERA.Orientation.Y = camera_mat(:,6); CAMERA.Orientation.Z = camera_mat(:,7);

    %% Camera Odometry Data formated into { time (s) | v_x (m/s) | v_y (m/s) | v_z (m/s) | w_x (rad/s) | w_y (rad/s) | w_z (rad/s) }
    camera_mat_2 = readmatrix('Data2/Parking/Camera_odometry_parking.csv');
    camera_mat_2(:,1:2) = [];
    CAMERA.Velocity.Linear.X = camera_mat_2(:,2); CAMERA.Velocity.Linear.Y = camera_mat_2(:,3); CAMERA.Velocity.Linear.Z = camera_mat_2(:,4);
    CAMERA.Velocity.Angular.X = camera_mat_2(:,5); CAMERA.Velocity.Angular.Y = camera_mat_2(:,6); CAMERA.Velocity.Angular.Z = camera_mat_2(:,7);

    %% IMU Acceleration and Angular Velocity Data formated into { time (s) | a_x (m/s^2) | a_y (m/s^2) | a_z (m/s^2) | w_x (rad/s) | w_y (rad/s) | w_z (rad/s) }
    imu_mat = readmatrix('Data2/Parking/gx5_1_IMU_data_parking.csv');
    imu_mat(:,1:2) = [];
    IMU.t = imu_mat(:,1);
    IMU.Accelerometer.X = imu_mat(:,2); IMU.Accelerometer.Y = imu_mat(:,3); IMU.Accelerometer.Z = imu_mat(:,4);
    IMU.Gyroscope.X = imu_mat(:,5); IMU.Gyroscope.Y = imu_mat(:,6); IMU.Gyroscope.Z = imu_mat(:,7);

end