%% Extracting v and w from the camera visual odometry

%Commands to load data from the rosbag

% bag_parking = rosbag('Husky_NewParking.bag');
% zed_odom_p = select(bag_parking,'Topic','/zed_node/odom');
% zed_odom_p_data = readMessages(zed_odom_p,'DataFormat','struct');

NSec = cell2mat(cellfun(@(m) double(m.Header.Stamp.Nsec),zed_odom_p_data,'UniformOutput',false));
Sec = cell2mat(cellfun(@(m) double(m.Header.Stamp.Sec),zed_odom_p_data,'UniformOutput',false));
Sec_tot = Sec + NSec*10^(-9);

X = cell2mat(cellfun(@(m) double(m.Pose.Pose.Position.X),zed_odom_p_data,'UniformOutput',false));
Y = cell2mat(cellfun(@(m) double(m.Pose.Pose.Position.Y),zed_odom_p_data,'UniformOutput',false));
Z = cell2mat(cellfun(@(m) double(m.Pose.Pose.Position.Z),zed_odom_p_data,'UniformOutput',false));

q_X = cell2mat(cellfun(@(m) double(m.Pose.Pose.Orientation.X),zed_odom_p_data,'UniformOutput',false));
q_Y = cell2mat(cellfun(@(m) double(m.Pose.Pose.Orientation.Y),zed_odom_p_data,'UniformOutput',false));
q_Z = cell2mat(cellfun(@(m) double(m.Pose.Pose.Orientation.Z),zed_odom_p_data,'UniformOutput',false));
q_W = cell2mat(cellfun(@(m) double(m.Pose.Pose.Orientation.W),zed_odom_p_data,'UniformOutput',false));

N = length(Sec);
v = zeros(N-1,3);
w = zeros(N-1,3);
new_time = zeros(N-1,1);
new_Sec = zeros(N-1,1);
new_NSec = zeros(N-1,1);

for i = 1:N-1
    Rk = quat2rotm([q_X(i) q_Y(i) q_Z(i) q_W(i)]);
    pk = [X(i); Y(i); Z(i)];
    Mk = [Rk, pk; zeros(1,3), 1];
    
    Rl = quat2rotm([q_X(i+1) q_Y(i+1) q_Z(i+1) q_W(i+1)]);
    pl = [X(i+1); Y(i+1); Z(i+1)];
    Ml = [Rl, pl; zeros(1,3), 1];
    
    del_t = Sec_tot(i+1) - Sec_tot(i);
    new_t = (Sec_tot(i+1) + Sec_tot(i))/2;
    new_time(i,1) = new_t;
    new_Sec(i,1) =floor(new_t);
    new_NSec(i,1) = (new_t - integ_t(i,1))*10^9;
    
    ksi_k = 1/del_t*logm(inv(Mk)*Ml);
    
    v(i,:) = ksi_k(1:3,4)';
    w(i,1) = ksi_k(3,2);
    w(i,2) = ksi_k(1,3);
    w(i,3) = ksi_k(2,1);
end

new_time_0 = new_time - new_time(1,1);
vis_odom_p = [new_Sec, new_NSec, new_time_0, v, w];
