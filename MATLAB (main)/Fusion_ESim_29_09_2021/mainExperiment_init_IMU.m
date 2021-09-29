function [ParamGlobal, IMU_img_struct] = mainExperiment_init_IMU(tMin, tImagesMin)

dirImage = 'C:\MSc\git\FUSION2018\data_event\shapes_rotation\';
ParamGlobal.dirImage = dirImage;

IMU = readmatrix('imu.txt');
t = IMU(:,1);
%t = t/10^9; % ns -> s
g = 9.81*[0;0;-1]; %gravity field

omega = IMU(tMin:end,5:7)'; 
acc = IMU(tMin:end,2:4)';  
t = t(tMin:end);
tIMU = t;  

ParamGlobal.tIMU = tIMU;
NbSteps = length(tIMU);

Images = readtable('images.txt');
tImages = table2array(Images(:,1));
tImages = tImages(tImagesMin:end);
fileImages = table2array(Images(:,2));

ParamGlobal.fileImages = fileImages;


groundtruth = readmatrix('groundtruth.txt');
tReal = groundtruth(:,1);
trajReal.x = groundtruth(:,1:3)';
trajReal.v = zeros(3,length(tReal));
trajReal.a_b = zeros(3,length(tReal));
trajReal.omega_b = zeros(3,length(tReal));
trajReal.quat = groundtruth(:,5:end)';
ground_euler = quat2eul(groundtruth(:,5:end));
trajReal.psi = ground_euler(:,1)';
trajReal.phi = ground_euler(:,2)';
trajReal.theta = ground_euler(:,3)';

IMU_img_struct.tIMU = tIMU;
IMU_img_struct.t = t;
IMU_img_struct.acc = acc;
IMU_img_struct.omega = omega;
IMU_img_struct.NbSteps = NbSteps;
IMU_img_struct.tImages = tImages;
IMU_img_struct.tReal = tReal;
IMU_img_struct.trajReal = trajReal;
IMU_img_struct.g = g;
