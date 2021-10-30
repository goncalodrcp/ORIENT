clear;
close all;

%% Initialization

addpath('myToolbox');
addpath('filters');


freqIMU = 100; %Hz
freqCam = 20; %Hz

offset = 1; % groundtruth offset
tMin = 1; % starting IMU time
tImagesMin = 1; % starting camera time

NbStepsMax = 1000;

%helper functions
addpath('D:\IST\ORIENT_repos\ORIENT\MATLAB (main)\Helpers\ProcessRosbag');

%location of the data
dirDataset = 'D:\IST\ORIENT_repos\Tests\ThesisSW\Data collected\Experiments_14_10\Feedback';
addpath(dirDataset);
%Image directory 
imageDir = 'D:\IST\ORIENT_repos\Tests\ThesisSW\Data collected\Experiments_14_10\Feedback\X-Axis\im_X_A10_v9';

%% Load data from experiment

%Load feedback from MATLAB API
load('X_A10_v9_Sample2.mat')
%Load rosbag
bag = rosbag('X_A10_v9_Sample2.bag');
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

%% 
[ParamGlobal, IMU_img_struct] = mainExperiment_init_IMU_v2(tMin, tImagesMin, imageDir,data);

[IMU_img_struct.tReal, IMU_img_struct.trajReal] = mainExperiment_correct_offset(IMU_img_struct.tReal, IMU_img_struct.trajReal, offset);


obsTimes = mainExperiment_define_obsTimes(IMU_img_struct.tIMU, IMU_img_struct.tImages);
% obsTimes(:)=0;

[ParamFilter, state_camera_struct] = mainExperiment_init_state_camera;

%% Tracker initialisation - ORB-SLAM

num_tracks = 200; %it was 100

orb_slam.omega_b = [0;0;0];
orb_slam.a_b = [0;0;0];

cameraParams = ParamFilter.cameraParams;
dirImage = ParamGlobal.dirImage;
fileImages = ParamGlobal.fileImages;

%image = undistortImage( imread( [dirImage fileImages{1}] ), cameraParams );
%image = undistortImage(fileImages{1},cameraParams);
image = rgb2gray(imread(fileImages{1}));
image = undistortImage(image,cameraParams);
init_points = detectMinEigenFeatures(image); %detetar novos pontos de interesse
%init_points = detectSURFFeatures(image); %try another features
init_points = selectUniform(init_points,num_tracks+30,size(image)); %selcioanr novos potnos

for i=1:num_tracks
    orb_slam.myTracks(i) = pointTrack(1, init_points(i).Location);
end

orb_slam.trackerBis = vision.PointTracker('NumPyramidLevels',10,'MaxBidirectionalError',3); %Typical value between 0 and 3 pixels.
initialize(orb_slam.trackerBis,init_points(1:num_tracks).Location,image);

%init_points = init_points(num_tracks + 1 : end); %WHY THIS???
init_points = selectUniform(init_points,30,size(image)); %selcioanr novos potnos


orb_slam.trackerMain = vision.PointTracker('NumPyramidLevels',10,'MaxBidirectionalError',3); %Typical value between 0 and 3 pixels.
initialize(orb_slam.trackerMain,init_points.Location,image);

initHelper = init_points.Location;

orb_slam.PosAmers = zeros(3,30);

for i=1:30
    orb_slam.PosAmers(1,i) = (init_points.Location(i,1) - ParamFilter.Pi(1,3)) / ParamFilter.Pi(1,1);
    orb_slam.PosAmers(2,i) = (init_points.Location(i,2) - ParamFilter.Pi(2,3)) / ParamFilter.Pi(2,2);
    
%     orb_slam.PosAmers(1,i) = init_points.Location(i,1) / ParamFilter.Pi(1,1);
%     orb_slam.PosAmers(2,i) = init_points.Location(i,2) / ParamFilter.Pi(2,2);

    orb_slam.PosAmers(3,i) = 1;

end

%% Tracker test

figure;

% TrackerMain
for i = 2:300

    image = rgb2gray(imread(fileImages{i}));  
    image = undistortImage(image,ParamFilter.cameraParams);

    [pointsMain,validityMain] = orb_slam.trackerMain.step(image);

    pointImage = insertMarker(image,pointsMain,'+','Color','green');

    imshow(pointImage);
end

%% TrackerBis

figure;
for i = 2:300

    image = rgb2gray(imread(fileImages{i}));      
    image = undistortImage(image,ParamFilter.cameraParams);

    [pointsMain,validityMain] = orb_slam.trackerBis.step(image);

    pointImage = insertMarker(image,pointsMain,'+','Color','red');

    imshow(pointImage);
end
