%data comes from the EUROC MAV dataset V1_01_medium
clear;
close all;
addpath ../LieGroupToolbox
addpath ../filters
addpath ../data
addpath ORBSLAM_EXAMPLE

%Assign directory where images can be found
dirImage = '../V1_02_medium/mav0/cam0/data/';
%IMU and ground-truth data
fileData = 'DATA_MEDIUM.mat';
%Image names and time instants
fileImages = 'fileImages_MEDIUM.mat';

%Frequency of acquisition
freqIMU = 200; %Hz
freqCam = 20; %Hz

offset = 881;
tMin = 1081; % starting IMU time
tImagesMin = 109; % starting camera time

NbStepsMax = 8000;

%% Definition of global parameters and data structures

[ParamGlobal,IMU_data,CAM_data,GT_data] = init_IMU_CAM(tMin,tImagesMin,fileData,fileImages);

%% Correct offset

[GT_data.tReal, GT_data.trajReal] = correctOffset(GT_data.tReal, GT_data.trajReal, offset);

%%

obsTimes = zeros(length(ParamGlobal.t),1);
obsTimes(1:freqIMU/freqCam:end) = 1; % IMU is 10 times faster than camera

%% Initialze Filter

%Use 'readmatrix' function with combination with
%'delimitedTextImportOptions' to set import requirements
%import sensor parameters from .yaml files. First convert them into a .txt
%file
%Example:
%opts = delimitedTextImportOptions('CommentStyle','%');
%opts = setvartype(opts,'double');
%A = readmatrix('modelParam.txt',opts);

%General parameters
NLand = 30; %number of landmarks
NLandMin = NLand; %minimum number of landmarks
pixelErr = 20; %pixel error threshold

%Import parameters
opts = delimitedTextImportOptions('CommentStyle','%');
opts = setvartype(opts,'double');
noiseIMU = readmatrix('IMU_noise_param.txt',opts); %array with IMU noise parameters
T_IC = readmatrix('extrinsics.txt',opts); %extrinsics -> Transformation from camera frame to IMU (body) frame
camParam = readmatrix('camParameters.txt',opts); %instrincs and distortion model
%Camera resolution
res = [752 480];

%Initialize filter
ParamFilter = init_filter(NLand,NLandMin,pixelErr,noiseIMU,camParam,T_IC);

%% State initialization

%Number of steps
NbSteps = ParamGlobal.NbSteps;
%Ground truth trajectory
trajReal = GT_data.trajReal;

%Initial covariance
P0amers = diag([1;1;1]*1.e-3); %initial landmark covariance
p0Rot = (0.01*pi/180)^2;
p0v =  1.e-4;
p0x =  1.e-8;
p0omegab = 1.e-6;
p0ab = 1.e-6;
P0 = diag([p0Rot*ones(3,1);p0v*ones(3,1);p0x*ones(3,1);...
    p0omegab*ones(3,1);p0ab*ones(3,1)]);
P0 = blkdiag(P0,kron(eye(ParamFilter.NbAmers),P0amers));

%Initialisation of the state is obtained following [Mur-Artal,2017],
%Map initialisation
[inState,imds,mapPointSet,vSetKeyFrames] = initORBSLAM(ParamGlobal,ParamFilter,GT_data);


