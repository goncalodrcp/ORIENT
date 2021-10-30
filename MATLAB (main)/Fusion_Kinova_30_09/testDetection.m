%%
clc
clear;
close all;

%%
%add UKF functions
addpath('myToolbox');
addpath('filters');
%helper functions
addpath('D:\IST\ORIENT_repos\ORIENT\MATLAB (main)\Helpers\ProcessRosbag');
%location of the data
dirDataset = 'D:\IST\ORIENT_repos\Tests\ThesisSW\Data collected\Experiments_14_10\Feedback\Z-Axis';
addpath(dirDataset);
%Image directory 
imageDir = 'D:\IST\ORIENT_repos\Tests\ThesisSW\Data collected\Experiments_14_10\Feedback\Z-Axis\im_Z_A20_v18';
%imageDir = 'D:\IST\ORIENT_repos\Tests\ThesisSW\Data collected\Experiments_14_10\Feedback\Random\im_randomSample1';

%% Load data from experiment

%Load feedback from MATLAB API
load('Z_A20_v18_Sample1.mat')
%Load rosbag
%bag = rosbag('Z_A10_v13_Sample1.bag');
bag = rosbag('Z_A20_v18_Sample1.bag');
%Select topics 
jointState_Topic = select(bag,'Topic','/my_gen3/joint_states');
image_Topic = select(bag,'Topic','/camera/image_raw');
imu_topic = select(bag,'Topic','/imu');
%Get ROS messages
%[imageData, imuData, jointData] = helperGetRosMessages(image_Topic,imu_topic,jointState_Topic);
[imageData, imuData, jointData] = helperGetRosMessages_v2(imageDir,image_Topic,imu_topic,jointState_Topic);

fileImages = imageData.images;

%% Camera parameters

%From the experiments of 14/10
cameraparameters = [1129.723097 1130.712255 978.656074 773.305876];
distortionCoeffs = [-0.269803 0.068608 0.000538 0.000525];
calib_param = [cameraparameters, distortionCoeffs];

fx = calib_param(1);
fy = calib_param(2);
cx = calib_param(3);
cy = calib_param(4);

k1 = calib_param(5);
k2 = calib_param(6);
p1 = calib_param(7);
p2 = calib_param(8);

Pi = [fx 0 0; 0 fy 0; cx, cy 1]';%camera calibration matrix
cameraParams = cameraParameters('IntrinsicMatrix', Pi',...
    'RadialDistortion',[k1, k2],...
    'TangentialDistortion',[p1, p2]);

%Focal length
focalLength = [fx fy];
%Principal point
PrincipalPoint = [cx cy];
%distortion parameters
rad_dist = [k1 k2];
tan_dist = [p1 p2];

%% Detect apriltags in first frame

image1 = rgb2gray(imread(fileImages{1}));
image1 = undistortImage(image1,cameraParams);

%Image dimensions
width = size(image1,2);
height = size(image1,1);
imageSize = [height width];

%Specify tag size in meters
tagSize = 0.1060;

%Create intrinsics object
intrinsics = cameraIntrinsics(focalLength,PrincipalPoint,imageSize,'RadialDistortion',rad_dist,'TangentialDistortion',tan_dist);

%Detect tags
[id,loc,pose] = readAprilTag(image1,"tag36h11",intrinsics,tagSize);

worldPoints = [0 0 0; tagSize/2 0 0; 0 tagSize/2 0; 0 0 tagSize/2];

for i = 1:length(pose)
    % Get images coordinates for axes.
    imagePoints = worldToImage(intrinsics,pose(i).Rotation, ...
                  pose(i).Translation,worldPoints);

    % Draw colored axes.
    image1 = insertShape(image1,"Line",[imagePoints(1,:) imagePoints(2,:); ...
        imagePoints(1,:) imagePoints(3,:); imagePoints(1,:) imagePoints(4,:)], ...
        "Color",["red","green","blue"],"LineWidth",7);

    image1 = insertText(image1,loc(1,:,i),id(i),"BoxOpacity",1,"FontSize",25);
end

imshow(image1)

firstPose = pose;
firstLocation = loc;
firstIDs = id;

%% Apriltag tracking

numImages = length(fileImages);

tag_ids = cell(1,numImages-1);
tag_locations = cell(1,numImages-1);
tag_poses = cell(1,numImages-1);

j=1;
for i = 2:numImages

    image = rgb2gray(imread(fileImages{i}));  
    image = undistortImage(image,cameraParams);
    
    %Detect tags
    [id,loc,pose] = readAprilTag(image,"tag36h11",intrinsics,tagSize);
    numTags = length(pose);
    disp("Detected number of tags:");
    disp(numTags);
    
    if(numTags == 0)
        disp('No tags detected, getting last detection');
        lastId = tag_ids{lastDetection};
        lastLocation = tag_locations{lastDetection};
        lastPose = tag_poses{lastDetection};
        disp('Last detection:');
        disp(lastDetection);
    else
        lastDetection = i;
        imageIDwTag(j) = i;
        j = j +1;
    end
    
    tag_ids{i-1} = id;
    tag_locations{i-1} = loc;
    tag_poses{i-1} = pose;

end

%%

%estimate relative rotations
w = 1;
for i=1:100
        R1 = firstPose(1).Rotation;
        R2 = tag_poses{imageIDwTag(i)-1}(1).Rotation;
        R = R1'*R2;
       relativeRotation(w).rotation = R;
       relativeRotation(w).euler = rotm2eul(R);
       euler(i,:) = relativeRotation(w).euler;
       w = w +1;
   
end


