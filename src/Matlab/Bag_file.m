clear all
close all
clc
%% 
bagFile = rosbag('F:\Bag Files\Indoor.bag');
StartTime = bagFile.StartTime;
EndTime = bagFile.EndTime;
TotalTime = EndTime - StartTime;

bSel = select(bagFile,'Topic','/odometry/filtered');
cameraPoseSel = select(bagFile,'Topic','/zed_node/path_odom');
gpsPoseSel = select(bagFile,'Topic','/gps/fix');
jointStatesSel = select(bagFile,'Topic','/joint_states');
pathMapSel = select(bagFile,'Topic','/zed_node/path_map');
imuSel = select(bagFile,'Topic','/gx5_0/imu/data');
imuSel1 = select(bagFile,'Topic','/gx5_1/imu/data');

msgStructs = readMessages(bSel,'DataFormat','struct');
msgCameraPose = readMessages(cameraPoseSel,'DataFormat','struct');
gpsPose = readMessages(gpsPoseSel,'DataFormat','struct');
jointStates = readMessages(jointStatesSel,'DataFormat','struct');
pathMap = readMessages(pathMapSel,'DataFormat','struct');
imuData = readMessages(imuSel,'DataFormat','struct');

%% 
%imu
axPointIMU = cellfun(@(m) double(m.LinearAcceleration.X),imuData);
ayPointIMU = cellfun(@(m) double(m.LinearAcceleration.Y),imuData);
azPointIMU = cellfun(@(m) double(m.LinearAcceleration.Z),imuData);

wxPointIMU = cellfun(@(m) double(m.AngularVelocity.X),imuData);
wyPointIMU = cellfun(@(m) double(m.AngularVelocity.Y),imuData);
wzPointIMU = cellfun(@(m) double(m.AngularVelocity.Z),imuData);

qxPointIMU = cellfun(@(m) double(m.Orientation.X),imuData);
qyPointIMU = cellfun(@(m) double(m.Orientation.Y),imuData);
qzPointIMU = cellfun(@(m) double(m.Orientation.Z),imuData);
qwPointIMU = cellfun(@(m) double(m.Orientation.W),imuData);

NSec = cell2mat(cellfun(@(m) double(m.Header.Stamp.Nsec),imuData,'UniformOutput',false));
Sec = cell2mat(cellfun(@(m) double(m.Header.Stamp.Sec),imuData,'UniformOutput',false));
Sec_tot = Sec + NSec*10^(-9);
tIMU = Sec_tot - Sec_tot(1,1);

Mat_IMU = [tIMU axPointIMU ayPointIMU azPointIMU wxPointIMU wyPointIMU wzPointIMU qwPointIMU qxPointIMU qyPointIMU qzPointIMU];
writematrix(Mat_IMU,'Mat_IMU.csv') 

% Odom
xPoints = cellfun(@(m) double(m.Pose.Pose.Position.X),msgStructs);
yPoints = cellfun(@(m) double(m.Pose.Pose.Position.Y),msgStructs);
zPoints = cellfun(@(m) double(m.Pose.Pose.Position.Z),msgStructs);
xOrientation = cellfun(@(m) double(m.Pose.Pose.Orientation.X),msgStructs);
yOrientation = cellfun(@(m) double(m.Pose.Pose.Orientation.Y),msgStructs);
zOrientation = cellfun(@(m) double(m.Pose.Pose.Orientation.Z),msgStructs);
wOrientation = cellfun(@(m) double(m.Pose.Pose.Orientation.W),msgStructs);
NSec = cell2mat(cellfun(@(m) double(m.Header.Stamp.Nsec),msgStructs,'UniformOutput',false));
Sec = cell2mat(cellfun(@(m) double(m.Header.Stamp.Sec),msgStructs,'UniformOutput',false));
Sec_tot = Sec + NSec*10^(-9);
tOdom = Sec_tot - Sec_tot(1,1);

xAngularVel = cellfun(@(m) double(m.Twist.Twist.Angular.X),msgStructs);
yAngularVel = cellfun(@(m) double(m.Twist.Twist.Angular.Y),msgStructs);
zAngularVel = cellfun(@(m) double(m.Twist.Twist.Angular.Z),msgStructs);

xLinearVel = cellfun(@(m) double(m.Twist.Twist.Linear.X),msgStructs);
yLinearVel = cellfun(@(m) double(m.Twist.Twist.Linear.Y),msgStructs);
zLinearVel = cellfun(@(m) double(m.Twist.Twist.Linear.Z),msgStructs);

Mat_Odom = [tOdom xPoints yPoints zPoints wOrientation xOrientation yOrientation zOrientation xAngularVel yAngularVel zAngularVel xLinearVel yLinearVel zLinearVel];
writematrix(Mat_Odom,'Mat_Odom.csv') 

% Camera
msgCameraPoseTest = msgCameraPose(end);
msgCameraPoseTest = msgCameraPoseTest{1,1}.Poses;
msgCameraPoseTest = struct2cell(msgCameraPoseTest);

msgCameraPathTest = pathMap(end);
msgCameraPathTest = msgCameraPathTest{1,1}.Poses;
msgCameraPathTest = struct2cell(msgCameraPathTest);

xPointsCamera = [];
yPointsCamera = [];
zPointsCamera = [];
xOrientationCamera = [];
yOrientationCamera = [];
zOrientationCamera = [];
wOrientationCamera = [];
NSec = [];
Sec = [];
Sec_tot = [];
for i=1:length(msgCameraPoseTest)
    A = msgCameraPoseTest(3,1,i);
    A1 = msgCameraPoseTest(2,1,i);
    xPointsCamera = [xPointsCamera;A{1,1}.Position.X];
    yPointsCamera = [yPointsCamera;A{1,1}.Position.Y];
    zPointsCamera = [zPointsCamera;A{1,1}.Position.Z];
    xOrientationCamera = [xOrientationCamera;A{1,1}.Orientation.X];
    yOrientationCamera = [yOrientationCamera;A{1,1}.Orientation.Y];
    zOrientationCamera = [zOrientationCamera;A{1,1}.Orientation.Z];
    wOrientationCamera = [wOrientationCamera;A{1,1}.Orientation.W];
    NSec = [NSec;A1{1,1}.Stamp.Nsec];
    Sec = [Sec;A1{1,1}.Stamp.Sec];
end

Sec_tot = double(Sec) + double(NSec*10^(-9));
tPathOdom = double((Sec_tot - Sec_tot(1,1))/10);
Mat_Camera_Odom = [tPathOdom xPointsCamera yPointsCamera zPointsCamera wOrientationCamera xOrientationCamera yOrientationCamera zOrientationCamera];
writematrix(Mat_Camera_Odom,'Mat_Camera_Odom.csv') 

xPointsPath = [];
yPointsPath = [];
zPointsPath = [];
xOrientationPath = [];
yOrientationPath = [];
zOrientationPath = [];
wOrientationPath = [];
NSec = [];
Sec = [];
Sec_tot = [];

for i=1:length(msgCameraPathTest)
    A = msgCameraPathTest(3,1,i);
    A1 = msgCameraPathTest(2,1,i);
    xPointsPath = [xPointsPath;A{1,1}.Position.X];
    yPointsPath = [yPointsPath;A{1,1}.Position.Y];
    zPointsPath = [zPointsPath;A{1,1}.Position.Z];
    xOrientationPath = [xOrientationPath;A{1,1}.Orientation.X];
    yOrientationPath = [yOrientationPath;A{1,1}.Orientation.Y];
    zOrientationPath = [zOrientationPath;A{1,1}.Orientation.Z];
    wOrientationPath = [wOrientationPath;A{1,1}.Orientation.W];
    NSec = [NSec;A1{1,1}.Stamp.Nsec];
    Sec = [Sec;A1{1,1}.Stamp.Sec];
end

Sec_tot = double(Sec) + double(NSec*10^(-9));
tPathMap = double((Sec_tot - Sec_tot(1,1))/10);
Mat_Camera_Path = [tPathMap xPointsPath yPointsPath zPointsPath wOrientationPath xOrientationPath yOrientationPath zOrientationPath];
writematrix(Mat_Camera_Path,'Mat_Camera_Path.csv') 


% GPS

laGpsPose = cellfun(@(m) double(m.Latitude),gpsPose);
loGpsPose = cellfun(@(m) double(m.Longitude),gpsPose);
alGpsPose = cellfun(@(m) double(m.Altitude),gpsPose);
NSec = cell2mat(cellfun(@(m) double(m.Header.Stamp.Nsec),gpsPose,'UniformOutput',false));
Sec = cell2mat(cellfun(@(m) double(m.Header.Stamp.Sec),gpsPose,'UniformOutput',false));
Sec_tot = Sec + NSec*10^(-9);
tGPS = Sec_tot - Sec_tot(1,1);

Mat_GPS = [tGPS laGpsPose loGpsPose alGpsPose];
writematrix(Mat_GPS,'Mat_GPS.csv') 
% Encoder

WheelVel = cell2mat(cellfun(@(m) double(m.Velocity),jointStates,'UniformOutput',false));
LeftFrontWheelVel = WheelVel(1:4:end);
RightFrontWheelVel = WheelVel(2:4:end);
NSec = cell2mat(cellfun(@(m) double(m.Header.Stamp.Nsec),jointStates,'UniformOutput',false));
Sec = cell2mat(cellfun(@(m) double(m.Header.Stamp.Sec),jointStates,'UniformOutput',false));
Sec_tot = Sec + NSec*10^(-9);
tEnc = Sec_tot - Sec_tot(1,1);

Mat_Encoder = [tEnc LeftFrontWheelVel RightFrontWheelVel];
writematrix(Mat_Encoder,'Mat_Encoder.csv') 


%%
Rtemp = eul2rotm([0 pi pi]);
Rtemp2 = eul2rotm([0 0 0]);
test = [];
test2 = [];
for i=1:length(xPointsCamera)
    temp = Rtemp'*[xPointsCamera(i);yPointsCamera(i);zPointsCamera(i)];
    temp2 = Rtemp2*[xPointsPath(i);yPointsPath(i);zPointsPath(i)];
    test = [test;temp'];
    test2 = [test2;temp2'];
end
figure(1)
hold on
plot(test(:,1),test(:,2),'r');
% plot(test2(:,1),test2(:,2),'g');
plot(xPoints,yPoints,'b')

plot(test(1,1),test(1,2),'*r')
plot(xPoints(1),yPoints(1),'*b')
plot(test2(1,1),test2(1,2),'*g');

plot(test2(end,1),test2(end,2),'^g');
plot(test(end,1),test(end,2),'^r')
plot(xPoints(end),yPoints(end),'^b')
% legend('Path odom','Path map','Husky Odom', 'fontsize', 20, 'Interpreter','latex');
axis equal tight 
grid on
%%
figure(2)
hold on
plot3(test(:,1),test(:,2),test(:,3),'r')
plot3(test2(:,1),test2(:,2),test2(:,3),'g')
plot3(xPoints,yPoints,zPoints,'b')
% plot3(xPointsCamera,-yPointsCamera,zPointsCamera,'r')
plot3(test(1,1),test(1,2),test(1,3),'*r')
plot3(xPoints(1),yPoints(1),zPoints(1),'*b')
plot3(test(end,1),test(end,2),test(end,3),'^r')
plot3(xPoints(end),yPoints(end),zPoints(end),'^b')
axis equal tight 
grid on