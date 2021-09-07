clear;
close all;

%% Import data

%add path to ESim test
path_to_sw = 'D:\IST\ORIENT_repos\Tests\ThesisSW\ESIM_test\201007_Fusion_2018_event_image\';
addpath(genpath(path_to_sw));

%path = 'D:\IST\ORIENT_repos\Tests\ThesisSW\ESIM_test\201007_Fusion_2018_event_image\Results\rotZ\';
%path = 'D:\IST\ORIENT_repos\Tests\ThesisSW\ESIM_test\201007_Fusion_2018_event_image\Results\rotY\'; %rotation on Y - yes thats correct
path = 'D:\IST\ORIENT_repos\Tests\ThesisSW\ESIM_test\201007_Fusion_2018_event_image\Results\rotX\'; %rotation on X - yes thats correct
addpath(path);

%load('rotZ_18_results.mat');
%load('rotX_18_results.mat'); %rotation on Y - yes thats correct
load('rotY_18_results.mat'); %rotation on X - yes thats correct

%note:the order of rotation is ZYX(1,2,3) or(psi,theta,phi), however, there's an offset
%rotation noticeable in the image frames and so the Y rotation is 3-theta
%and the X rotation is 2-phi
%The z axis rotation remains the same for this convention

%% Plot errors

%Size of the sample
Nmax = NbStepsMax;
N = Nmax;

%Relevant trajectories to analyse
trajSE3 = trajs.trajL;
trajSO3 = trajs.trajU;
%Ground truth trajectory
trajGT = IMU_img_struct.trajReal;
%Time instants
t = IMU_img_struct.t;

%Compute errors
errorSE3 = computeError(trajSE3,trajGT,N);
errorSO3 = computeError(trajSO3,trajGT,N);

%Plot orientation error(º)
figure;hold on;
plot(t(2:N-1)-t(2),errorSE3.errorR);
plot(t(2:N-1)-t(2),errorSO3.errorR);
disp(sqrt(mean([errorSE3.errorR errorSO3.errorR ].^2)));
legend('SE(3) - UKF','SO(3) - UKF')
xlabel('t (s)')
ylabel('RMSE attitdude (°)')
title('RMSE on attitude as function of time')

%Plot error in position(m)
figure; hold on;
plot(t(2:N-1)-t(2),errorSE3.errorX);
plot(t(2:N-1)-t(2),errorSO3.errorX);
disp(sqrt(mean([errorSE3.errorX errorSO3.errorX ].^2)));
legend('SE(3) - UKF','SO(3) - UKF')
xlabel('t (s)')
ylabel('RMSE position (m)')
title('RMSE on position as function of time')

%% Compare estimated trajectories with ground truth

% Estimated orientation in euler angles (º)
trajSE3.psi = trajSE3.psi*180/pi;
trajSE3.theta = trajSE3.theta*180/pi;
trajSE3.phi = trajSE3.phi*180/pi;
trajSO3.psi = trajSO3.psi*180/pi;
trajSO3.theta = trajSO3.theta*180/pi;
trajSO3.phi = trajSO3.phi*180/pi;


% Estimated position (m)
estPosSE3 = trajSE3.x;
estPosSO3 = trajSO3.x;

%Grount-truth
trajGT.psi = trajGT.psi*180/pi;
trajGT.theta = trajGT.theta*180/pi;
trajGT.phi = trajGT.phi*180/pi;
PositionGT = trajGT.x;
tGT = IMU_img_struct.tReal/10^9;

% Plot orientation - Euler angles
figure;
sgtitle('Orientation Euler angles')
subplot(3,1,1); 
plot(t,trajSE3.psi); hold on;
title('Z-Axis'); 
plot(t,trajSO3.psi);
plot(tGT,trajGT.psi);
legend('SE(3)','SO(3)','Ground-truth');
xlabel('t(s)');
ylabel('Angle (º)');

subplot(3,1,2); 
plot(t,trajSE3.phi); hold on;
title('Y-Axis');
plot(t,trajSO3.phi);
plot(tGT,trajGT.phi); 
legend('SE(3)','SO(3)','Ground-truth');
xlabel('t(s)');
ylabel('Angle (º)');

subplot(3,1,3); 
plot(t,trajSE3.theta); hold on;
title('X-Axis');
plot(t,trajSO3.theta);
plot(tGT,trajGT.theta); 
legend('SE(3)','SO(3)','Ground-truth');
xlabel('t(s)');
ylabel('Angle (º)');

