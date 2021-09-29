clear;
close all;

%%

addpath('myToolbox');
addpath('filters');


freqIMU = 100; %Hz
freqCam = 10; %Hz

offset = 1; % groundtruth offset
tMin = 1; % starting IMU time
tImagesMin = 1; % starting camera time

%NbStepsMax = 4000;

%helper functions
addpath('D:\IST\ORIENT_repos\ORIENT\MATLAB (main)\Helpers\ProcessRosbag');

dirDataset = 'D:\IST\ORIENT_repos\Tests\ThesisSW\Data collected\Experiments_24_07\Feedback\Gonçalo\';
addpath(dirDataset);

%data = load('XAXIS_1_filter_part1.mat');
[ParamGlobal, IMU_img_struct] = mainExperiment_init_IMU_v2(tMin, tImagesMin, dirDataset);
%[ParamGlobal, IMU_img_struct] = initIMUnCam(tMin, tImagesMin, dirDataset,data);

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

%plotIMUdata(tIMU,acc,omega);

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


traj_est = [trajs.trajR.psi; trajs.trajR.theta; trajs.trajR.phi];
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

%  path = 'D:\IST\ORIENT_repos\Tests\ThesisSW\ESIM_test\201007_Fusion_2018_event_image_Dataset\Results\';
%  fileName = 'rotZ_29_09.mat';
%  save([path fileName]);

