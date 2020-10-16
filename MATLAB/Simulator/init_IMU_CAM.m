function [ParamGlobal,IMU_data,CAM_data,GT_data] = init_IMU_CAM(tMin,tImagesMin,fileData,fileImages)

%Load IMU and ground-truth data
fileData = load(fileData);
%Load image data
fileImages = load(fileImages);

ParamGlobal.dirImage = '../V1_02_medium/mav0/cam0/data/';

tIMU = fileData.tIMU; %time instants
omega = fileData.omega; %gyroscope data
acc = fileData.acc; %accelerometer data
%Ground-truth data
tReal = fileData.tReal; 
trajReal = fileData.trajReal;

t = tIMU;
t = t/10^9; % ns -> s
g = 9.81*[0;0;-1]; %gravity field
omega = omega(:,tMin:end);
acc = acc(:,tMin:end);
t = t(tMin:end);
tIMU = tIMU(tMin:end);
NbSteps = length(tIMU);

tImages = fileImages.tImages(tImagesMin:end);
fileImages = fileImages.fileImages(tImagesMin:end);

ParamGlobal.tIMU = tIMU;
ParamGlobal.tImages = tImages;
ParamGlobal.t = t;
ParamGlobal.NbSteps = NbSteps;
ParamGlobal.g = g;

%Create IMU structure
IMU_data.tIMU = tIMU;
IMU_data.acc = acc;
IMU_data.omega = omega;

%Create Camera structure
CAM_data.tImages = tImages;
CAM_data.fileImages = fileImages;

%Create Ground-truth structure
GT_data.tReal = tReal;
GT_data.trajReal = trajReal;

end

