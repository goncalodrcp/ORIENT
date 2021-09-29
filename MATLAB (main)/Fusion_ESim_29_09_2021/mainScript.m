clear;
close all;

%%

addpath('myToolbox');
addpath('filters');

freqIMU = 1000; %Hz
freqCam = 67; %Hz

offset = 1; % groundtruth offset
tMin = 1; % starting IMU time
tImagesMin = 1; % starting camera time

NbStepsMax = 4000;

%helper functions
addpath('D:\IST\ORIENT_repos\ORIENT\MATLAB (main)\Helpers\ProcessRosbag');

dirDataset = 'D:\IST\ORIENT_repos\Tests\ThesisSW\Data collected\rotations\rotation_18_y\';
%dirDataset = 'D:\IST\ORIENT_repos\Tests\ThesisSW\Data collected\rotations\rotation_18_z\';
%dirDataset = 'D:\IST\ORIENT_repos\Tests\ThesisSW\Data collected\rotations\rotation_18_x\';
%High speed data
%dirDataset = 'D:\IST\ORIENT_repos\Tests\ThesisSW\Data collected\rotations\rotation_18_y_hs\';
%dirDataset = 'D:\IST\ORIENT_repos\Tests\ThesisSW\Data collected\rotations\rotation_18_z_hs\';
%dirDataset = 'D:\IST\ORIENT_repos\Tests\ThesisSW\Data collected\rotations\rotation_18_x_hs\';

addpath(dirDataset);

[ParamGlobal, IMU_img_struct] = mainExperiment_init_IMU_v2(tMin, tImagesMin);

[IMU_img_struct.tReal, IMU_img_struct.trajReal] = mainExperiment_correct_offset(IMU_img_struct.tReal, IMU_img_struct.trajReal, offset);

obsTimes = mainExperiment_define_obsTimes(IMU_img_struct.tIMU, IMU_img_struct.tImages);
% obsTimes(:)=0;

[ParamFilter, state_camera_struct] = mainExperiment_init_state_camera;

%%
% Initialisation of the state is obtained following [Mur-Artal,2017],
% orb_slam = load('data/ORB_SLAM_init.mat');  
% orb_slam = orb_slam.orb_slam;

[orb_slam, initHelper] = mainExperiment_init_orbSlam(ParamFilter, ParamGlobal);

tIMU = IMU_img_struct.tIMU;
acc = IMU_img_struct.acc;
omega = IMU_img_struct.omega;

plotIMUdata(tIMU,acc,omega);


% mainExperiment_map_init_tst(tracker, ParamFilter, ParamGlobal, initHelper)%
% trackerTest(ParamGlobal, orb_slam, ParamFilter);

%%

[trajs, i] = mainExperiment_loop(orb_slam, IMU_img_struct, state_camera_struct, NbStepsMax, obsTimes, ParamFilter, ParamGlobal);

%% Plots
mainExperiment_plot(trajs, i, IMU_img_struct.trajReal, IMU_img_struct.t)

traj_est = [trajs.trajR.psi; trajs.trajR.theta; trajs.trajR.phi];
traj_ground = [IMU_img_struct.trajReal.psi; IMU_img_struct.trajReal.theta; IMU_img_struct.trajReal.phi];
figure;
plot(traj_est(2,:)); hold on;
plot(traj_ground(2,:));


%% Save results

%  path = 'D:\IST\ORIENT_repos\Tests\ThesisSW\ESIM_test\201007_Fusion_2018_event_image\Results\rotZ\';
%  fileName = 'rotZ_18_HighSpeed_results.mat';
%  save([path fileName]);


