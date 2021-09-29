function mainExperiment_2

freqIMU = 1000; %Hz
freqCam = 67; %Hz

offset = 1; % groundtruth offset
tMin = 1; % starting IMU time
tImagesMin = 1; % starting camera time

NbStepsMax = 2000;

% check if this works, choosing the dataId in this main file:
% global dataId
% %dataId= 'ev_shapes_rotation';
% dataId= 'ev_boxes_rotation';
%neurocams3_data( 'ev_rotation_18_y' ); % save internaly the "dataId"

dirDataset = 'D:\IST\ORIENT_repos\Tests\ThesisSW\Data collected\rotations\rotation_18_y\';
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


% mainExperiment_map_init_tst(tracker, ParamFilter, ParamGlobal, initHelper)%
% trackerTest(ParamGlobal, orb_slam, ParamFilter);

[trajs, i] = mainExperiment_loop(orb_slam, IMU_img_struct, state_camera_struct, NbStepsMax, obsTimes, ParamFilter, ParamGlobal);

%% Plots
mainExperiment_plot(trajs, i, IMU_img_struct.trajReal, IMU_img_struct.t)

