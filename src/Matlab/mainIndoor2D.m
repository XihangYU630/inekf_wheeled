%% Description
% DecodeData.m decodes the csv files 
clear all
close all
clc
fsize = 10;
addpath('Utilities\')
%% Data
[gps,imu,camera,encoder] = getDataFromCsvIndoor(); % Need to multiply gps data by tf to bring to robot frame
temp = encoder.t;
state.g = [0; 0; 9.81];
state.r_wheel = [0; 0; 0.33/2];
state.robot_base = [0.556; 0; 0];
time = imu.t(1):0.01:imu.t(end);

%% Initialize state vector
state.R = eye(2);
state.v = 1e-2*ones(2,1);
state.p = [0;0];
state.bw = 1e-2*ones(2,1);
state.ba = 1e-2*ones(2,1);
state.Rc = eye(2);
state.pc = 1e-2*ones(2,1); % Multiply by R from imu to camera
state.P = 0.1*eye(14);
state.X = [state.R state.v state.p; zeros(1,2) 1 0; zeros(1,2) 0 1];
state.cov_w = 0.1*diag(ones(14,1));
state.cov_nf = diag([5 5]);

%% Initialize Results
results.orientation = [];
results.position = [];
results.linear_velocity = [];
results.bias_w = [];
results.bias_a = [];
results.camera_orientation = [];
results.camera_position = [];

%% Initialize Ground Truth
gt.orientation = [];
gt.position = [];
gt.linear_velocity = [];
gt.bias_w = [];
gt.bias_a = [];
gt.camera_orientation = [];
gt.camera_position = [];

%% Propagation + Correction
for i=2:length(time)
    if isempty(encoder.t)
        break
    end
    %% Get time
    [~,index_imu] = min(abs(time(i)-imu.t));
    [~,index_encoder] = min(abs(time(i)-encoder.t));
    [~,index_camera] = min(abs(time(i)-camera.t));
    state.dt = time(i) - time(i-1);
    %% Get Data
    gyro_vec = [imu.Gyroscope.X(index_imu); imu.Gyroscope.Y(index_imu); imu.Gyroscope.Z(index_imu)];
    acc_vec = [imu.Accelerometer.X(index_imu); imu.Accelerometer.Y(index_imu); imu.Accelerometer.Z(index_imu)];
    enc_vec = [encoder.Velocity.Left(index_encoder); encoder.Velocity.Right(index_encoder)];
    %% Propagate, correct and remove data
    state = propagate_state(state,gyro_vec,acc_vec);
    state = correct_encoder(state,gyro_vec,enc_vec);
    [imu,encoder] = clear_data(imu,encoder,index_imu,index_encoder);
    %% Update Results
    results.orientation = [results.orientation; acos(state.R(1,1))];
    results.position = [results.position; state.p'];
    results.linear_velocity = [results.linear_velocity; state.v'];
    results.bias_w = [results.bias_w; state.bw'];
    results.bias_a = [results.bias_a; state.ba'];
    results.camera_orientation = [results.camera_orientation; rotm2eul(state.Rc)];
    results.camera_position = [results.camera_position; state.pc'];
    %% Update Ground Truth
    gt.orientation = [gt.orientation; camera.Orientation.Z(index_camera)];
    gt.position = [gt.position; camera.Position.X(index_camera) camera.Position.Y(index_camera)];
    gt.linear_velocity = [gt.linear_velocity; camera.Velocity.Linear.X(index_camera) camera.Velocity.Linear.Y(index_camera)];
    gt.bias_w = [gt.bias_w; state.bw'];
    gt.bias_a = [gt.bias_a; state.ba'];
    gt.camera_orientation = [gt.camera_orientation; camera.Orientation.Z(index_camera)];
    gt.camera_position = [gt.camera_position; camera.Position.X(index_camera) camera.Position.Y(index_camera)];
    gt.camera_position = [gt.camera_position; camera.Velocity.Linear.X(index_camera) camera.Velocity.Linear.Y(index_camera)];
end
%% Plotting

time = temp;

figure(1)
hold on
rottest = eul2rotm(pi/1.5); % pi/1.3
test= [];
for i=1:length(gps.Position.X)
    test = [test;(rottest*[gps.Position.X(i);gps.Position.Y(i);gps.Position.Z(i)])'];
end
test(53:68,:) = [];
plot(results.position(:,1),results.position(:,2),'b');
plot(camera.Position.X,camera.Position.Y,'black')
plot(test(:,1),test(:,2),'green')
xlabel('$x \; (m)$', 'fontsize', fsize, 'Interpreter','latex')
ylabel('$y \; (m)$', 'fontsize', fsize, 'Interpreter','latex')
legend('Ours','Camera','GPS', 'fontsize', fsize, 'Interpreter','latex');
title('(a)','fontsize', fsize, 'Interpreter','latex');
axis equal tight 
grid on

figure(2)
subplot(7,3,1)
hold on
plot(time,results.orientation(:,1));
plot(time,gt.orientation(:,1));
xlabel('$t \; (s)$', 'fontsize', fsize, 'Interpreter','latex')
ylabel('$\phi \; (rad)$', 'fontsize', fsize, 'Interpreter','latex')

subplot(7,3,2)
hold on
plot(time,results.orientation(:,2));
plot(time,gt.orientation(:,2));
xlabel('$t \; (s)$', 'fontsize', fsize, 'Interpreter','latex')
ylabel('$\theta \; (rad)$', 'fontsize', fsize, 'Interpreter','latex')

subplot(7,3,3)
hold on
plot(time,results.orientation(:,3));
plot(time,gt.orientation(:,3));
xlabel('$t \; (s)$', 'fontsize', fsize, 'Interpreter','latex')
ylabel('$\psi \; (rad)$', 'fontsize', fsize, 'Interpreter','latex')

subplot(7,3,4)
hold on
plot(time,results.linear_velocity(:,1));
plot(time,gt.linear_velocity(:,1));
xlabel('$t \; (s)$', 'fontsize', fsize, 'Interpreter','latex')
ylabel('$v_x \; (m/s)$', 'fontsize', fsize, 'Interpreter','latex')

subplot(7,3,5)
hold on
plot(time,results.linear_velocity(:,2));
plot(time,gt.linear_velocity(:,2));
xlabel('$t \; (s)$', 'fontsize', fsize, 'Interpreter','latex')
ylabel('$v_y \; (m/s)$', 'fontsize', fsize, 'Interpreter','latex')

subplot(7,3,6)
hold on
plot(time,results.linear_velocity(:,3));
plot(time,gt.linear_velocity(:,3));
xlabel('$t \; (s)$', 'fontsize', fsize, 'Interpreter','latex')
ylabel('$v_z \; (m/s)$', 'fontsize', fsize, 'Interpreter','latex')

subplot(7,3,7)
hold on
plot(time,results.position(:,1));
plot(time,gt.position(:,1)*100);
xlabel('$t \; (s)$', 'fontsize', fsize, 'Interpreter','latex')
ylabel('$x \; (m)$', 'fontsize', fsize, 'Interpreter','latex')

subplot(7,3,8)
hold on
plot(time,results.position(:,2));
plot(time,gt.position(:,2)*100);
xlabel('$t \; (s)$', 'fontsize', fsize, 'Interpreter','latex')
ylabel('$y \; (m)$', 'fontsize', fsize, 'Interpreter','latex')

subplot(7,3,9)
hold on
plot(time,results.position(:,3));
plot(time,gt.position(:,3)*100);
xlabel('$t \; (s)$', 'fontsize', fsize, 'Interpreter','latex')
ylabel('$z \; (m)$', 'fontsize', fsize, 'Interpreter','latex')

subplot(7,3,10)
plot(time,results.bias_w(:,1));
xlabel('$t \; (s)$', 'fontsize', fsize, 'Interpreter','latex')
ylabel('$b_\omega (1) \; (rad/s)$', 'fontsize', fsize, 'Interpreter','latex')

subplot(7,3,11)
plot(time,results.bias_w(:,2));
xlabel('$t \; (s)$', 'fontsize', fsize, 'Interpreter','latex')
ylabel('$b_\omega (2) \; (rad/s)$', 'fontsize', fsize, 'Interpreter','latex')

subplot(7,3,12)
plot(time,results.bias_w(:,3));
xlabel('$t \; (s)$', 'fontsize', fsize, 'Interpreter','latex')
ylabel('$b_\omega (3) \; (rad/s)$', 'fontsize', fsize, 'Interpreter','latex')

subplot(7,3,13)
plot(time,results.bias_a(:,1));
xlabel('$t \; (s)$', 'fontsize', fsize, 'Interpreter','latex')
ylabel('$b_a (1) \; (m/s^2)$', 'fontsize', fsize, 'Interpreter','latex')

subplot(7,3,14)
plot(time,results.bias_a(:,2));
xlabel('$t \; (s)$', 'fontsize', fsize, 'Interpreter','latex')
ylabel('$b_a (2) \; (m/s^2)$', 'fontsize', fsize, 'Interpreter','latex')

subplot(7,3,15)
plot(time,results.bias_a(:,3));
xlabel('$t \; (s)$', 'fontsize', fsize, 'Interpreter','latex')
ylabel('$b_a (3) \; (m/s^2)$', 'fontsize', fsize, 'Interpreter','latex')

subplot(7,3,16)
plot(time,results.camera_orientation(:,1));
xlabel('$t \; (s)$', 'fontsize', fsize, 'Interpreter','latex')
ylabel('$\phi_c \; (rad)$', 'fontsize', fsize, 'Interpreter','latex')

subplot(7,3,17)
plot(time,results.camera_orientation(:,2));
xlabel('$t \; (s)$', 'fontsize', fsize, 'Interpreter','latex')
ylabel('$\theta_c \; (rad)$', 'fontsize', fsize, 'Interpreter','latex')

subplot(7,3,18)
plot(time,results.camera_orientation(:,3));
xlabel('$t \; (s)$', 'fontsize', fsize, 'Interpreter','latex')
ylabel('$\psi_c \; (rad)$', 'fontsize', fsize, 'Interpreter','latex')

subplot(7,3,19)
plot(time,results.camera_position(:,1));
xlabel('$t \; (s)$', 'fontsize', fsize, 'Interpreter','latex')
ylabel('$x_c \; (m)$', 'fontsize', fsize, 'Interpreter','latex')

subplot(7,3,20)
plot(time,results.camera_position(:,2));
xlabel('$t \; (s)$', 'fontsize', fsize, 'Interpreter','latex')
ylabel('$y_c \; (m)$', 'fontsize', fsize, 'Interpreter','latex')

subplot(7,3,21)
plot(time,results.camera_position(:,3));
xlabel('$t \; (s)$', 'fontsize', fsize, 'Interpreter','latex')
ylabel('$z_c \; (m)$', 'fontsize', fsize, 'Interpreter','latex')

%% 
function  state = propagate_state(state,gyro_vec,acc_vec)
    state.R = state.R * expm( wedge_se3( ( gyro_vec - state.bw ) * state.dt ) );
    state.v = state.v + state.R * ( acc_vec - state.ba ) * state.dt + state.g * state.dt;
    state.p = state.p + state.v * state.dt + 0.5 * state.R * ( acc_vec - state.ba ) * state.dt ^ 2 + 0.5 * state.g * state.dt ^ 2;
    state.X = [state.R state.v state.p; zeros(1,3) 1 0; zeros(1,3) 0 1];
    state.bw = state.bw;
    state.ba = state.ba;
    state.Rc = state.Rc;
    state.pc = state.pc;
    phi = expm( jacobian_A(state.X) * state.dt );
    Q = jacobian_B( state.X ) * state.cov_w * jacobian_B( state.X )';
    Q_disc = phi * Q * phi' *state.dt;
    state.P = phi * state.P * phi' + Q_disc;
end

function state = correct_encoder(state,gyro_vec,enc_vec)
    H = [zeros(3) eye(3) zeros(3,15)];
    N = state.R * state.cov_nf * state.R';
    S = H * state.P * H' + N;
    L = state.P * H' * ( S \ eye( size(S) ) );
    b = [ zeros(3,1); -1; 0 ];
    %% Left Update
    w_l = [enc_vec(1)/state.r_wheel(3); 0; 0];
    y_l = [wedge_se3(w_l)*state.r_wheel - 0.5*wedge_se3(gyro_vec)*state.robot_base; -1; 0];
    delta_vec_l =  L * [eye(3) zeros(3,2)] * ( state.X * y_l - b );
    state.X = expm( wedge_se23 ( delta_vec_l(1:9) ) ) * state.X;
    state.R = state.X(1:3,1:3);
    state.v = state.X(1:3,4);
    state.p = state.X(1:3,5);
    state.P = (eye(21)-L*H)*state.P*(eye(21)-L*H)' + L*N*L';
    state.ba = state.ba + delta_vec_l(13:15);
    state.bw = state.bw + delta_vec_l(10:12);
    state.Rc = expm(wedge_se3(delta_vec_l(16:18)))*state.Rc;
    state.pc = state.pc + delta_vec_l(19:21);
    %% right
    w_r = [enc_vec(2)/state.r_wheel(3); 0; 0];
    y_r = [wedge_se3(w_r)*state.r_wheel + 0.5*wedge_se3(gyro_vec)*state.robot_base; -1; 0];
    delta_vec_r =  L * [eye(3) zeros(3,2)] * ( state.X * y_r - b );
    state.X = expm( wedge_se23 ( delta_vec_r(1:9) ) ) * state.X;
    state.R = state.X(1:3,1:3);
    state.v = state.X(1:3,4);
    state.p = state.X(1:3,5);
    state.ba = state.ba + delta_vec_r(13:15);
    state.bw = state.bw + delta_vec_r(10:12);
    state.Rc = expm(wedge_se22(delta_vec_r(16:18)))*state.Rc;
    state.pc = state.pc + delta_vec_r(19:21);
end

function [imu,encoder] = clear_data(imu,encoder,index_imu,index_encoder)
    imu.t(index_imu) = [];
    imu.Gyroscope.X(index_imu) = [];
    imu.Gyroscope.Y(index_imu) = [];
    imu.Gyroscope.Z(index_imu) = [];
    imu.Accelerometer.X(index_imu) = [];
    imu.Accelerometer.Y(index_imu) = [];
    imu.Accelerometer.Z(index_imu) = [];
    encoder.t(index_encoder) = [];
    encoder.Velocity.Left(index_encoder) = [];
    encoder.Velocity.Right(index_encoder) = [];
end