function [ParamGlobal, IMU_img_struct] = mainExperiment_init_IMU_v2(tMin, tImagesMin, dirImage,data1)

%Load robot model
loadSTL;
eeName = 'Gripper';
numJoints = 7;

ParamGlobal.dirImage = dirImage;
%data1 = load('ZAXIS_1_filter_part1.mat');
%data2 = load('XAXIS_1_filter_part2.mat');

% IMU = readmatrix('imu.txt');
% t = IMU(:,1);
% t = t/10^9; % ns -> s
t = data1.imuData.timestamp;
%startTime = t(1);
%t = t-startTime; %fix time offset
g = 9.81*[0;0;-1]; %gravity field

% omega = IMU(tMin:end,5:7)'; 
% acc = IMU(tMin:end,2:4)';  

%omega = data1.imuData.gyroscope(:,tMin:end);
%acc = data1.imuData.accelerometer(:,tMin:end);
%try to reverse order of the axis below
omegaData = data1.imuData.gyroscope(:,tMin:end);
%omega(1,:) = omegaData(2,:);
%omega(2,:) = omegaData(3,:);
%omega(3,:) = omegaData(1,:);
omega= roty(-90)*rotx(180)*omegaData;

accData = data1.imuData.accelerometer(:,tMin:end);
%acc(1,:) = accData(2,:);
%acc(2,:) = accData(3,:);
%acc(3,:) = accData(1,:);
acc = roty(-90)*rotx(180)*accData;
t = t(tMin:end); 
startTime = t(1);
t = t-startTime; %fix time offset
tIMU = t;  
%tIMU = tIMU-tIMU(1); %fix time offset

%% Pre filter IMU data with median filter;
% % 
medFilt = dsp.MedianFilter(15); %window size
filtOmega = medFilt(omega');
omega = filtOmega';
filtAcc = medFilt(acc');
acc = filtAcc';

%%
% % 
acc(1,:) = -acc(1,:);
acc(2,:) = -acc(2,:);

%%

ParamGlobal.tIMU = tIMU;
NbSteps = length(tIMU);

% tImages = tImages/10^9; % ns -> s %ADDED THIS, DEPENDS ON DATASET
% fileImages = table2array(Images(:,2));
tImages = data1.imageData.timestamp;
tImages = tImages-startTime; %fix time offset
tImages = tImages(tImages > 0);
fileImages = data1.imageData.images;
ParamGlobal.fileImages = fileImages;


% groundtruth = readmatrix('groundtruth.txt');
% tReal = groundtruth(:,1);
% trajReal.x = groundtruth(:,2:4)';
% trajReal.x = trajReal.x / 10^8; %um to m
% trajReal.v = zeros(3,length(tReal));
% trajReal.a_b = zeros(3,length(tReal));
% trajReal.omega_b = zeros(3,length(tReal));
% trajReal.quat = [groundtruth(:,8) groundtruth(:,5) groundtruth(:,6) groundtruth(:,7)];
% trajReal.quat = [groundtruth(:,5) groundtruth(:,6) groundtruth(:,7)
% groundtruth(:,8)]; this line is wrong!!!
% ground_euler = quat2eul(trajReal.quat);

%  for j=1:size(ground_euler,1)
%     if ground_euler(j,1) < -pi/2
%         ground_euler(j,1) = ground_euler(j,1) + 2*pi;
%     end
%  end
%  ground_euler(:,1) = ground_euler(:,1) - pi;
%  ground_euler(:,2) = -ground_euler(:,2);
%  %ground_euler(:,3) = ground_euler(:,3) + pi/2;
%  ground_euler(:,3) = -(ground_euler(:,3) + pi/2); %for x-axis rotation

%%% VERIFICAR SE PSI PHE THETA BATEM CERTO COM XYZ
%%% 3 - X 2 - Y 1 - Z ;
%%% THETA E Y
% trajReal.psi = ground_euler(:,1)';
% trajReal.phi = ground_euler(:,3)';
% trajReal.theta = ground_euler(:,2)';


%%Ground-Truth Kinova
tReal = data1.jointData.timestamp;
tReal = tReal-startTime; %fix time offset
tReal = tReal(tReal > 0);
jointTraj = data1.jointData.angles;
for i=1:length(jointTraj)
    FBTransform(:,:,i) = getTransform(gen3,jointTraj(i,:)*pi/180,eeName);
    eeFBPos(:,i) = tform2trvec(FBTransform(:,:,i)); %XYZ position of the end-effector
    %eeFBOrientation(:,i) = tform2eul(FBTransform(:,:,i))*180/pi; %Orientation in Euler angles ZYX
    eeFBOrientation(:,i) = tform2eul(FBTransform(:,:,i)); %in radians
    eeFBquat(:,i) = tform2quat(FBTransform(:,:,i));
 end
trajReal.x = eeFBPos;
trajReal.quat = eeFBquat;
trajReal.psi = eeFBOrientation(1,:);
trajReal.phi = eeFBOrientation(3,:); 
trajReal.theta = eeFBOrientation(2,:);
%Keep the velocity like this for now - substitute by the joint velocity
%later
trajReal.v = zeros(3,length(tReal));
%Keep the biases like this for now
trajReal.a_b = zeros(3,length(tReal));
trajReal.omega_b = zeros(3,length(tReal));

IMU_img_struct.tIMU = tIMU;
IMU_img_struct.t = t;
IMU_img_struct.acc = acc;
IMU_img_struct.omega = omega;
IMU_img_struct.NbSteps = NbSteps;
IMU_img_struct.tImages = tImages;
IMU_img_struct.tReal = tReal;
IMU_img_struct.trajReal = trajReal;
IMU_img_struct.g = g;
