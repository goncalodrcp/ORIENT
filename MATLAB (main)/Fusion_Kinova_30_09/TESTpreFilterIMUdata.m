clear;
close all;

%helper functions
addpath('D:\IST\ORIENT_repos\ORIENT\MATLAB (main)\Helpers\ProcessRosbag');
%location of the data
dirDataset = 'D:\IST\ORIENT_repos\Tests\ThesisSW\Data collected\Experiments_14_10\Feedback\Y-Axis';
addpath(dirDataset);
%Image directory 
imageDir = 'D:\IST\ORIENT_repos\Tests\ThesisSW\Data collected\Experiments_14_10\Feedback\Y-Axis\im_Y_A20_v15';

%Load rosbag
%bag = rosbag('Z_A10_v13_Sample1.bag');
bag = rosbag('Y_A20_v15_Sample1.bag');
%Select topics 
jointState_Topic = select(bag,'Topic','/my_gen3/joint_states');
image_Topic = select(bag,'Topic','/camera/image_raw');
imu_topic = select(bag,'Topic','/imu');
%Get ROS messages
%[imageData, imuData, jointData] = helperGetRosMessages(image_Topic,imu_topic,jointState_Topic);
[imageData, imuData, jointData] = helperGetRosMessages_v2(imageDir,image_Topic,imu_topic,jointState_Topic);
data.imageData = imageData;
data.imuData = imuData;
data.jointData = jointData;

t = imuData.timestamp;
t = t(1:end); 
startTime = t(1);
t = t-startTime; %fix time offset
tIMU = t;  


%Rotate axis to match orientation of the camera
omegaData = imuData.gyroscope(:,1:end);
omega= roty(-90)*rotx(180)*omegaData;

accData = imuData.accelerometer(:,1:end);
acc = roty(-90)*rotx(180)*accData;

%% Filter IMU data

medFilt = dsp.MedianFilter(5);
filtOmega = medFilt(omega');
filtAcc = medFilt(acc');


%% Plot IMU sensor data
% Acceleration
figure;
sgtitle('Accelerometer')
subplot(3,1,1);
plot(tIMU,acc(1,:)); hold on;
plot(tIMU,filtAcc(:,1))
title('X-Axis');
subplot(3,1,2); 
plot(tIMU,acc(2,:)); hold on;
plot(tIMU,filtAcc(:,2))
title('Y-Axis');
subplot(3,1,3); 
plot(tIMU,acc(3,:)); hold on;
plot(tIMU,filtAcc(:,3))
title('Z-Axis');

% Angular Velocity
figure;
sgtitle('Gyroscope')
subplot(3,1,1);
plot(tIMU,omega(1,:)); hold on;
plot(tIMU,filtOmega(:,1))
title('X-Axis');
subplot(3,1,2); 
plot(tIMU,omega(2,:)); hold on;
plot(tIMU,filtOmega(:,2))
title('Y-Axis');
subplot(3,1,3); 
plot(tIMU,omega(3,:)); hold on;
plot(tIMU,filtOmega(:,3))
title('Z-Axis');
