function [ParamGlobal, IMU_img_struct] = initIMUnCam(tMin, tImagesMin, dirImage, data)

%Load robot model
loadSTL;
eeName = 'Gripper';
numJoints = 7;

ParamGlobal.dirImage = dirImage;
%data = load('XAXIS_1_filter_part1.mat');

%Pre-filter data
[data.imageData,data.imuData,data.jointData] = helperPreFilterData(data.imageData,data.imuData,data.jointData);

t = data.imuData.timestamp;
omegaData = data.imuData.gyroscope(:,tMin:end);
accData = data.imuData.accelerometer(:,tMin:end);
g = 9.81*[0;0;-1]; %gravity field 

%Correct offset in orientation
omega= roty(-90)*rotx(180)*omegaData;
acc = roty(-90)*rotx(180)*accData;

%Correct time offsets
%IMU
t = t(tMin:end); 
startTime = t(1);
t = t-startTime; %fix time offset
tIMU = t;
%Camera
tImages = data.imageData.timestamp;
tImages = tImages-startTime; %fix time offset
tImages = tImages(tImages > 0);
%Kinova - Ground-truth
tReal = data.jointData.timestamp;
tReal = tReal-startTime; %fix time offset
tReal = tReal(tReal > 0);

ParamGlobal.tIMU = tIMU;
NbSteps = length(tIMU);
fileImages = data.imageData.images;
ParamGlobal.fileImages = fileImages;

%%Ground-Truth Kinova
jointTraj = data.jointData.angles;
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

end
