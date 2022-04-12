function [GPS,IMU,CAMERA,ENCODER] = getDataFromCsvIndoor()
    %   GETDATAFROMCSV function reads the csv files of the gps, camera, imu and encoder data
    %   and returns the stored data
    %
    %   Author: Theodor Chakhachiro
    %   Date:   04/11/2022

    %% GPS Data formated into { time (s) | x (m) | y (m) | z (m) }
    gps_mat = readmatrix('Data2/Indoor/Mat_GPS.csv');
    gps_mat(:,2:4) = lla2ecef( gps_mat(:,2:4) ) - lla2ecef(gps_mat(1,2:4));
    GPS.t = gps_mat(:,1); 
    GPS.Position.X = gps_mat(:,2); GPS.Position.Y = gps_mat(:,3); GPS.Position.Z = gps_mat(:,4);

    %% Encoder Data formated into { time (s) | Left Wheel Velocity v_l (m/s) | Right Wheel Velocity v_r (m/s) }
    enc_mat = readmatrix('Data2/Indoor/Mat_Encoder.csv');
    ENCODER.t = enc_mat(:,1); 
    ENCODER.Velocity.Left = enc_mat(:,2); ENCODER.Velocity.Right = enc_mat(:,3);

    %% Camera Ground Truth Data formated into { time (s) | x (m) | y (m) | z (m) | phi (rad) | theta (rad) | psi (rad) }
    camera_mat = readmatrix('Data2/Indoor/Mat_Camera_Path.csv');
    camera_mat(:,5:7) = quat2eul(camera_mat(:,5:8));
    camera_mat(:,8) = [];
    CAMERA.t = camera_mat(:,1); 
    CAMERA.Position.X = camera_mat(:,2); CAMERA.Position.Y = camera_mat(:,3); CAMERA.Position.Z = camera_mat(:,4);
    CAMERA.Orientation.X = camera_mat(:,5); CAMERA.Orientation.Y = camera_mat(:,6); CAMERA.Orientation.Z = camera_mat(:,7);

    %% Camera Odometry Data formated into { time (s) | v_x (m/s) | v_y (m/s) | v_z (m/s) | w_x (rad/s) | w_y (rad/s) | w_z (rad/s) }
    camera_mat_2 = readmatrix('Data2/Parking/Camera_odometry_parking.csv');
    camera_mat_2(:,1:2) = [];
    CAMERA.Velocity.Linear.X = camera_mat_2(:,2); CAMERA.Velocity.Linear.Y = camera_mat_2(:,3); CAMERA.Velocity.Linear.Z = camera_mat_2(:,4);
    CAMERA.Velocity.Angular.X = camera_mat_2(:,5); CAMERA.Velocity.Angular.Y = camera_mat_2(:,6); CAMERA.Velocity.Angular.Z = camera_mat_2(:,7);

    %% IMU Acceleration and Angular Velocity Data formated into { time (s) | a_x (m/s^2) | a_y (m/s^2) | a_z (m/s^2) | w_x (rad/s) | w_y (rad/s) | w_z (rad/s) }
    imu_mat = readmatrix('Data2/Indoor/Mat_IMU.csv');
    IMU.t = imu_mat(:,1);
    temp = quat2eul(imu_mat(:,8:11));
    IMU.Accelerometer.X = imu_mat(:,2); IMU.Accelerometer.Y = imu_mat(:,3); IMU.Accelerometer.Z = imu_mat(:,4);
    IMU.Gyroscope.X = imu_mat(:,5); IMU.Gyroscope.Y = imu_mat(:,6); IMU.Gyroscope.Z = imu_mat(:,7);
    IMU.Orientation.X = temp(:,3); IMU.Orientation.Y = temp(:,2); IMU.Orientation.Z = temp(:,1);

end