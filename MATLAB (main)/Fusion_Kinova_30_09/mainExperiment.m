function mainExperiment

freqIMU = 200; %Hz
freqCam = 20; %Hz

%%shapes
% offset = 2; % groundtruth offset
% tMin = 14; % starting IMU time
% tImagesMin = 1; % starting camera time

%%box 
offset = 3; % groundtruth offset
tMin = 24; % starting IMU time
tImagesMin = 2; % starting camera time


NbStepsMax = 500;

[ParamGlobal, IMU_img_struct] = mainExperiment_init_IMU(tMin, tImagesMin);

[IMU_img_struct.tReal, IMU_img_struct.trajReal] = mainExperiment_correct_offset(IMU_img_struct.tReal, IMU_img_struct.trajReal, offset);

obsTimes = mainExperiment_define_obsTimes(IMU_img_struct.tIMU, IMU_img_struct.tImages);

[ParamFilter, state_camera_struct] = mainExperiment_init_state_camera;

%%
% Initialisation of the state is obtained following [Mur-Artal,2017],
% orb_slam = load('data/ORB_SLAM_init.mat');
% orb_slam = orb_slam.orb_slam;

orb_slam = mainExperiment_init_orbSlam(ParamFilter, ParamGlobal);

[trajs, i] = mainExperiment_loop(orb_slam, IMU_img_struct, state_camera_struct, NbStepsMax, obsTimes, ParamFilter, ParamGlobal);

%% Plots
mainExperiment_plot(trajs, i, IMU_img_struct.trajReal, IMU_img_struct.t)

