%% Template for extracting data from the rosbag
%  Vlad Krokhmal 4/10/2022
​
% Specify rosbag_name, topic_path, mat_file_name, and CSV_file_name
% Note that location/path of the data and variable names might be different in different rosbags
​
bag_name = rosbag('rosbag_name.bag');
topic = select(bag_name,'Topic','/topic_path');
struct_data = readMessages(topic,'DataFormat','struct');
​
NSec = cell2mat(cellfun(@(m) double(m.Header.Stamp.Nsec),struct_data,'UniformOutput',false));
Sec = cell2mat(cellfun(@(m) double(m.Header.Stamp.Sec),struct_data,'UniformOutput',false));
Sec_tot = Sec + NSec*10^(-9);
Sec_tot_0 = Sec_tot - Sec_tot(1,1);
​
X = cell2mat(cellfun(@(m) double(m.Pose.Pose.Position.X),struct_data,'UniformOutput',false));
Y = cell2mat(cellfun(@(m) double(m.Pose.Pose.Position.Y),struct_data,'UniformOutput',false));
Z = cell2mat(cellfun(@(m) double(m.Pose.Pose.Position.Z),struct_data,'UniformOutput',false));
​
qX = cell2mat(cellfun(@(m) double(m.Pose.Pose.Orientation.X),struct_data,'UniformOutput',false));
qY = cell2mat(cellfun(@(m) double(m.Pose.Pose.Orientation.Y),struct_data,'UniformOutput',false));
qZ = cell2mat(cellfun(@(m) double(m.Pose.Pose.Orientation.Z),struct_data,'UniformOutput',false));
qW = cell2mat(cellfun(@(m) double(m.Pose.Pose.Orientation.W),struct_data,'UniformOutput',false));
​
v_x = cell2mat(cellfun(@(m) double(m.Twist.Twist.Linear.X),struct_data,'UniformOutput',false));
v_y = cell2mat(cellfun(@(m) double(m.Twist.Twist.Linear.Y),struct_data,'UniformOutput',false));
v_z = cell2mat(cellfun(@(m) double(m.Twist.Twist.Linear.Z),struct_data,'UniformOutput',false));
​
w_x = cell2mat(cellfun(@(m) double(m.Twist.Twist.Angular.X),struct_data,'UniformOutput',false));
w_y = cell2mat(cellfun(@(m) double(m.Twist.Twist.Angular.Y),struct_data,'UniformOutput',false));
w_z = cell2mat(cellfun(@(m) double(m.Twist.Twist.Angular.Z),struct_data,'UniformOutput',false));
  
​
mat_file_name = [Sec_tot_0, X, Y, Z, qX, qY, qZ, v_x, v_y, v_z, w_x, w_y, w_z];
​
dlmwrite('CSV_file_name.csv',mat_file_name, 'precision', 14)
%Need to manually label the columns in the CSV file afterwards