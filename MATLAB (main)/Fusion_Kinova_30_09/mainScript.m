clear;
close all;

%%
%add UKF functions
addpath('myToolbox');
addpath('filters');
%helper functions
addpath('D:\IST\ORIENT_repos\ORIENT\MATLAB (main)\Helpers\ProcessRosbag');
%location of the data
dirDataset = 'D:\IST\ORIENT_repos\Tests\ThesisSW\Data collected\Experiments_14_10\Feedback\Y-Axis';
addpath(dirDataset);
%Image directory 
imageDir = 'D:\IST\ORIENT_repos\Tests\ThesisSW\Data collected\Experiments_14_10\Feedback\Y-Axis\im_Y_A20_v13';
%imageDir = 'D:\IST\ORIENT_repos\Tests\ThesisSW\Data collected\Experiments_14_10\Feedback\Random\im_randomSample1';

freqIMU = 100; %Hz
freqCam = 20; %Hz
offset = 1; % groundtruth offset
tMin = 1; % starting IMU time
tImagesMin = 1; % starting camera time
%NbStepsMax = 4000; %Number of steps o the filter

%% Load data from experiment

%Load feedback from MATLAB API
load('Y_A20_v13_Sample1.mat')
%Load rosbag
%bag = rosbag('Z_A10_v13_Sample1.bag');
bag = rosbag('Y_A20_v13_Sample1.bag');
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

%% Test data filtering
%Pre filter data
% [imageDataFilt,imuDataFilt,jointDataFilt] = helperPreFilterData(data.imageData,data.imuData,data.jointData);
% dataFilt.imageData = imageDataFilt;
% dataFilt.imuData = imuDataFilt;
% dataFilt.jointData = jointDataFilt;
% 
% t_joints = jointDataFilt.timestamp;
% t_IMU = imuDataFilt.timestamp;
% t_images = imageDataFilt.timestamp;
% 
% [flag,t_sorted,t_IMU_new,t_joints_new,t_images_new,occurrences] = sortVectors(t_IMU,t_joints,t_images);
% if(~flag)
%    disp('Something went wrong when aligning the vectors'); 
% end

%%


[ParamGlobal, IMU_img_struct] = mainExperiment_init_IMU_v2(tMin, tImagesMin, imageDir,data);
%[ParamGlobal, IMU_img_struct] = initIMUnCam(tMin, tImagesMin, imageDir,data);

[IMU_img_struct.tReal, IMU_img_struct.trajReal] = mainExperiment_correct_offset(IMU_img_struct.tReal, IMU_img_struct.trajReal, offset);


obsTimes = mainExperiment_define_obsTimes(IMU_img_struct.tIMU, IMU_img_struct.tImages);
% obsTimes(:)=0;

[ParamFilter, state_camera_struct] = mainExperiment_init_state_camera;

NbStepsMax = length(obsTimes);

%%
% Initialisation of the state is obtained following [Mur-Artal,2017],
% orb_slam = load('data/ORB_SLAM_init.mat');  
% orb_slam = orb_slam.orb_slam;

[orb_slam, initHelper] = mainExperiment_init_orbSlam(ParamFilter, ParamGlobal);
%mainExperiment_map_init_tst(tracker, ParamFilter, ParamGlobal, initHelper)%
%trackerTest(ParamGlobal, orb_slam, ParamFilter);


tIMU = IMU_img_struct.tIMU;
acc = IMU_img_struct.acc;
omega = IMU_img_struct.omega;
orientation = imuData.orientation;
orientation = orientation';

%plotIMUdata(tIMU,acc,omega,orientation);

%% Plot IMU raw sensor data
% Acceleration
figure;
sgtitle('Accelerometer rotated')
subplot(3,1,1);
plot(tIMU,acc(1,:))
title('X-Axis');
subplot(3,1,2); 
plot(tIMU,acc(2,:))
title('Y-Axis');
subplot(3,1,3); 
plot(tIMU,acc(3,:))
title('Z-Axis');

% Angular Velocity
figure;
sgtitle('Gyroscope rotated')
subplot(3,1,1);
plot(tIMU,omega(1,:))
title('X-Axis');
subplot(3,1,2); 
plot(tIMU,omega(2,:))
title('Y-Axis');
subplot(3,1,3); 
plot(tIMU,omega(3,:))
title('Z-Axis');

%%

[trajs, i] = mainExperiment_loop(orb_slam, IMU_img_struct, state_camera_struct, NbStepsMax, obsTimes, ParamFilter, ParamGlobal);

%% Plots

t = IMU_img_struct.t;
tReal = IMU_img_struct.tReal;

estTrajectories = adjustTrajectories(trajs,t,tReal);

%mainExperiment_plot(trajs, i, IMU_img_struct.trajReal, IMU_img_struct.t)
mainExperiment_plot(estTrajectories, length(tReal), IMU_img_struct.trajReal, IMU_img_struct.t)


traj_est = [trajs.trajL.psi; trajs.trajL.theta; trajs.trajL.phi];
traj_ground = [IMU_img_struct.trajReal.psi; IMU_img_struct.trajReal.theta; IMU_img_struct.trajReal.phi];

figure;
plot(t,traj_est(1,:)); hold on;
plot(tReal,traj_ground(1,:));
figure;
plot(t,traj_est(2,:)); hold on;
plot(tReal,traj_ground(2,:));
figure;
plot(t,traj_est(3,:)); hold on;
plot(tReal,traj_ground(3,:));

%% Save results
% 
% path = 'D:\IST\ORIENT_repos\Tests\ThesisSW\ESIM_test\Fusion_Kinova_30_09\Results\';
% fileName = 'res_Y_A20_v13_MF10.mat';
% save([path fileName]);

