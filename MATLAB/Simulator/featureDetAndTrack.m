clear;
close all;
addpath ../LieGroupToolbox
addpath ../filters
addpath ../data

load exampleImage.mat
load('DATA_MEDIUM.mat')
load('fileImages_MEDIUM.mat')
load('ORB_SLAM_init.mat')

K = [458.654 0 0; 0 457.296 0; 367.215, 248.375 1]';
cam_param = cameraParameters('IntrinsicMatrix', K',...
    'RadialDistortion',[-0.28340811, 0.07395907],...
    'TangentialDistortion',[0.00019359, 1.76187114e-05]);