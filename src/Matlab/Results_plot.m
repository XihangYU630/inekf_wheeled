clear all
close all
clc
%% 
fsize = 10;
Euler_imu = [];
RESULTS = [];
Euler_camera = [];
gps_res = [];
pc_world = [];
p_orbslam_world = [];
p_visual_odom = [];
Rc = [0 1 0;-1 0 0;0 0 1];
[gps,imu,camera,encoder] = getDataFromCsv(); 
Enc_only_res = readmatrix('Data2\Parking\encoder_only_res.csv');
Visual_odom = load('Data2\Parking\DATA.mat');
orbslam = readmatrix('Data2\Parking\augmented_parking_timed.csv');
vel_odom = readmatrix('Data2\Parking\Husky_vel_ctrl_odom_with_vw.csv');
t_lim = Enc_only_res(end,1);
indx = find( 5 < gps.t );
idx = find( gps.t < t_lim);
gps.t = gps.t(gps.t > 5);
gps.t = gps.t(indx(1):idx(end));

gps.Position.X = gps.Position.X(indx(1):idx(end));
gps.Position.Y = gps.Position.Y(indx(1):idx(end));
gps.Position.Z = gps.Position.Z(indx(1):idx(end));


idx = find(orbslam(:,1) < t_lim);
orbslam = orbslam(1:idx(end),:);
rotation_euler_subplot = [];


Visual_odom_position = [Visual_odom.sortedsensordatasetnew(:,4) Visual_odom.sortedsensordatasetnew(:,13:15)];
Visual_odom_position = Visual_odom_position{:,:};
Visual_odom_position(any(isnan(Visual_odom_position), 2), :) = [];
length_res_vis_odom = length(Visual_odom_position(Visual_odom_position(:,1) <= t_lim));
Visual_odom_position = Visual_odom_position(1:length_res_vis_odom,:);

Visual_odom_orientation = [Visual_odom.sortedsensordatasetnew(:,19) Visual_odom.sortedsensordatasetnew(:,16:18)];
Visual_odom_orientation = Visual_odom_orientation{:,:};
Visual_odom_orientation(any(isnan(Visual_odom_orientation), 2), :) = [];
length_res_vis_odom = length(Visual_odom_orientation(Visual_odom_orientation(:,1) <= t_lim));
Visual_odom_orientation = Visual_odom_orientation(1:length_res_vis_odom,:);

Visual_odom_velocity = Visual_odom.sortedsensordatasetnew(:,20:22);
Visual_odom_velocity = Visual_odom_velocity{:,:};
Visual_odom_velocity(any(isnan(Visual_odom_velocity), 2), :) = [];
length_res_vis_odom = length(Visual_odom_velocity(Visual_odom_velocity(:,1) <= t_lim));
Visual_odom_velocity = Visual_odom_velocity(1:length_res_vis_odom,:);

rotation_euler_subplot_camera = [];
orientation_visual_odom = [];
velocity_visual_odom = [];


for i=1:length(Enc_only_res)
    t_i = Enc_only_res(i,1);
    R_closest =  eye(3);
    [t_vis_odom,t_vis_odom_index] = min(abs(Visual_odom_position(:,1) - t_i));
    R_temp = quat2rotm([Enc_only_res(i,8) Enc_only_res(i,5:7)]);
    p_visual_odom = (R_closest*Rc*([Visual_odom_position(t_vis_odom_index,2);Visual_odom_position(t_vis_odom_index,3);Visual_odom_position(t_vis_odom_index,4)]))';
    orientation_visual_odom = [orientation_visual_odom ; quat2eul(Visual_odom_orientation(t_vis_odom_index,:),'XYZ')];
    velocity_visual_odom = [velocity_visual_odom ; (R_temp*Rc*Visual_odom_velocity(t_vis_odom_index,:)')'];
    [t_orb_slam,t_orb_slam_index] = min(abs(orbslam(:,1) - t_i));
    p_orbslam_world = (R_closest*Rc*([orbslam(t_orb_slam_index,2);orbslam(t_orb_slam_index,3);orbslam(t_orb_slam_index,4)]))';
    [t_gps,t_gps_index] = min(abs(gps.t - t_i));
    gps_res = (R_closest*Rc*eul2rotm([pi/1.5 0 pi/1.5])*[gps.Position.X(t_gps_index);gps.Position.Y(t_gps_index);gps.Position.Z(t_gps_index)])';
    Enc_temp = Enc_only_res(i,2:4)';
    RESULTS = [RESULTS; t_i p_visual_odom p_orbslam_world gps_res Enc_temp'];
    rotation_euler_subplot = [rotation_euler_subplot; quat2eul([Enc_only_res(i,8) Enc_only_res(i,5:7)] ,'XYZ')];
    rotation_euler_subplot_camera = [rotation_euler_subplot_camera; quat2eul([Enc_only_res(i,24) Enc_only_res(i,21:23)] ,'XYZ')];
end
p_orbslam_world = [];
p_visual_odom = [];
gps_res = [];
for i=1:length(Visual_odom_position)
    [t_min,t_index] = min(abs(Visual_odom_position(i,1) - Enc_only_res(:,1)));
%     t_index = i;
    R_closest =  eye(3);
    p_visual_odom = [p_visual_odom; (R_closest*Rc*([Visual_odom_position(i,2);Visual_odom_position(i,3);Visual_odom_position(i,4)]))'];
end

for i=1:length(Enc_only_res)
    Euler_imu = [Euler_imu;quat2eul([Enc_only_res(i,8) Enc_only_res(i,5:7)],'XYZ')];
    Euler_camera = [Euler_camera;quat2eul([Enc_only_res(i,24) Enc_only_res(i,21:23)],'XYZ')];
    temp_R = quat2rotm([Enc_only_res(i,8) Enc_only_res(i,5:7)]);
%     temp_Rc = quat2rotm([Enc_only_res(i,24) Enc_only_res(i,21:23)]);
    pc_world = [pc_world; (temp_R*Rc*[Enc_only_res(i,18);Enc_only_res(i,19);Enc_only_res(i,20)])'];
end

for i=1:length(gps.Position.X)
   [t_min,t_index] = min(abs(gps.t(i) - Enc_only_res(:,1)));
%    t_index = i;
    R_closest =  eye(3);
    gps_res = [gps_res; (R_closest*Rc*eul2rotm([pi/1.5 0 pi/1.5])*[gps.Position.X(i);gps.Position.Y(i);gps.Position.Z(i)])'];
end

for i=1:length(orbslam)
    [t_min,t_index] = min(abs(orbslam(i,1) - Enc_only_res(:,1)));
%     t_index = i;
    R_closest =  eye(3);
    p_orbslam_world = [p_orbslam_world; (R_closest*Rc*([orbslam(i,2);orbslam(i,3);orbslam(i,4)]))'];
end
 %% 
figure(1)
hold on
% plot3(Enc_only_res(1,2),Enc_only_res(1,3),Enc_only_res(1,4),'*');
% plot3(Enc_only_res(end,2),Enc_only_res(end,3),Enc_only_res(end,4),'^');

plot3(p_orbslam_world(:,1),p_orbslam_world(:,2),p_orbslam_world(:,3))
plot3(Enc_only_res(:,2),Enc_only_res(:,3),Enc_only_res(:,4));
plot3(gps_res(:,1),gps_res(:,2),gps_res(:,3))
plot3(p_visual_odom(:,1),p_visual_odom(:,2),p_visual_odom(:,3));
xlabel('$x \; (m)$', 'fontsize', fsize, 'Interpreter','latex')
ylabel('$y \; (m)$', 'fontsize', fsize, 'Interpreter','latex')
zlabel('$z \; (m)$', 'fontsize', fsize, 'Interpreter','latex')
legend('Orbslam','Ours','GPS', 'Visual Odom', 'fontsize', fsize, 'Interpreter','latex');
title('(a)','fontsize', fsize, 'Interpreter','latex');
axis equal tight 
grid on

time = Enc_only_res(:,1);

figure(2)
sgtitle('\textbf{Slip-Robust Encoder-only InEKF State Vector Results}','fontsize', 2*fsize, 'Interpreter','latex');
subplot(7,3,1)
hold on
plot(time,rotation_euler_subplot(:,1));
plot(time,orientation_visual_odom(:,1));
% plot(time_gt,gt.orientation(:,1));
xlabel('$t \; (s)$', 'fontsize', fsize, 'Interpreter','latex')
ylabel('$\phi \; (rad)$', 'fontsize', fsize, 'Interpreter','latex')
legend('Ours','Visual Odom','fontsize', 2*fsize, 'Interpreter','latex');


subplot(7,3,2)
hold on
plot(time,rotation_euler_subplot(:,2));
plot(time,orientation_visual_odom(:,2));
xlabel('$t \; (s)$', 'fontsize', fsize, 'Interpreter','latex')
ylabel('$\theta \; (rad)$', 'fontsize', fsize, 'Interpreter','latex')

subplot(7,3,3)
hold on
plot(time,rotation_euler_subplot(:,3));
plot(time,orientation_visual_odom(:,3));
xlabel('$t \; (s)$', 'fontsize', fsize, 'Interpreter','latex')
ylabel('$\psi \; (rad)$', 'fontsize', fsize, 'Interpreter','latex')

subplot(7,3,4)
hold on
plot(time,Enc_only_res(:,9));
plot(time,velocity_visual_odom(:,1));
xlabel('$t \; (s)$', 'fontsize', fsize, 'Interpreter','latex')
ylabel('$v_x \; (m/s)$', 'fontsize', fsize, 'Interpreter','latex')

subplot(7,3,5)
hold on
plot(time,Enc_only_res(:,10));
plot(time,velocity_visual_odom(:,2));
xlabel('$t \; (s)$', 'fontsize', fsize, 'Interpreter','latex')
ylabel('$v_y \; (m/s)$', 'fontsize', fsize, 'Interpreter','latex')

subplot(7,3,6)
hold on
plot(time,Enc_only_res(:,11));
plot(time,velocity_visual_odom(:,3));
xlabel('$t \; (s)$', 'fontsize', fsize, 'Interpreter','latex')
ylabel('$v_z \; (m/s)$', 'fontsize', fsize, 'Interpreter','latex')

subplot(7,3,7)
hold on
plot(time,Enc_only_res(:,2));
plot(time,RESULTS(:,2));
xlabel('$t \; (s)$', 'fontsize', fsize, 'Interpreter','latex')
ylabel('$x \; (m)$', 'fontsize', fsize, 'Interpreter','latex')

subplot(7,3,8)
hold on
plot(time,Enc_only_res(:,3));
plot(time,RESULTS(:,3));
xlabel('$t \; (s)$', 'fontsize', fsize, 'Interpreter','latex')
ylabel('$y \; (m)$', 'fontsize', fsize, 'Interpreter','latex')

subplot(7,3,9)
hold on
plot(time,Enc_only_res(:,4));
plot(time,RESULTS(:,4));
xlabel('$t \; (s)$', 'fontsize', fsize, 'Interpreter','latex')
ylabel('$z \; (m)$', 'fontsize', fsize, 'Interpreter','latex')

subplot(7,3,10)
plot(time,Enc_only_res(:,15));
xlabel('$t \; (s)$', 'fontsize', fsize, 'Interpreter','latex')
ylabel('$b_{\omega,x} \; (rad/s)$', 'fontsize', fsize, 'Interpreter','latex')

subplot(7,3,11)
plot(time,Enc_only_res(:,16));
xlabel('$t \; (s)$', 'fontsize', fsize, 'Interpreter','latex')
ylabel('$b_{\omega,y} \; (rad/s)$', 'fontsize', fsize, 'Interpreter','latex')

subplot(7,3,12)
plot(time,Enc_only_res(:,17));
xlabel('$t \; (s)$', 'fontsize', fsize, 'Interpreter','latex')
ylabel('$b_{\omega,z} \; (rad/s)$', 'fontsize', fsize, 'Interpreter','latex')

subplot(7,3,13)
plot(time,Enc_only_res(:,12));
xlabel('$t \; (s)$', 'fontsize', fsize, 'Interpreter','latex')
ylabel('$b_{a,x} \; (m/s^2)$', 'fontsize', fsize, 'Interpreter','latex')

subplot(7,3,14)
plot(time,Enc_only_res(:,13));
xlabel('$t \; (s)$', 'fontsize', fsize, 'Interpreter','latex')
ylabel('$b_{a,y} \; (m/s^2)$', 'fontsize', fsize, 'Interpreter','latex')

subplot(7,3,15)
plot(time,Enc_only_res(:,14));
xlabel('$t \; (s)$', 'fontsize', fsize, 'Interpreter','latex')
ylabel('$b_{a,z} \; (m/s^2)$', 'fontsize', fsize, 'Interpreter','latex')

subplot(7,3,16)
plot(time,rotation_euler_subplot_camera(:,1));
xlabel('$t \; (s)$', 'fontsize', fsize, 'Interpreter','latex')
ylabel('$\phi_c \; (rad)$', 'fontsize', fsize, 'Interpreter','latex')

subplot(7,3,17)
plot(time,rotation_euler_subplot_camera(:,2));
xlabel('$t \; (s)$', 'fontsize', fsize, 'Interpreter','latex')
ylabel('$\theta_c \; (rad)$', 'fontsize', fsize, 'Interpreter','latex')

subplot(7,3,18)
plot(time,rotation_euler_subplot_camera(:,3));
xlabel('$t \; (s)$', 'fontsize', fsize, 'Interpreter','latex')
ylabel('$\psi_c \; (rad)$', 'fontsize', fsize, 'Interpreter','latex')

subplot(7,3,19)
plot(time,Enc_only_res(:,18));
xlabel('$t \; (s)$', 'fontsize', fsize, 'Interpreter','latex')
ylabel('$x_c \; (m)$', 'fontsize', fsize, 'Interpreter','latex')

subplot(7,3,20)
plot(time,Enc_only_res(:,19));
xlabel('$t \; (s)$', 'fontsize', fsize, 'Interpreter','latex')
ylabel('$y_c \; (m)$', 'fontsize', fsize, 'Interpreter','latex')

subplot(7,3,21)
plot(time,Enc_only_res(:,20));
xlabel('$t \; (s)$', 'fontsize', fsize, 'Interpreter','latex')
ylabel('$z_c \; (m)$', 'fontsize', fsize, 'Interpreter','latex')

